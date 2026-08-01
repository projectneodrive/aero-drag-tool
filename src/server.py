"""Local web server for the aero drag GUI.

Holds one active scene in memory (this is a single-user desktop tool), serves
the browser front end, and runs solver jobs on a background thread so the UI
stays responsive during a CFD run.

    python server.py            # then open http://127.0.0.1:8000

Scenes saved into the ``scenes/`` directory are ordinary scene files, so the
offline round trip is just:

    (GUI) save  ->  scenes/case.aero.json
    python runner.py run scenes/case.aero.json
    (GUI) load  ->  results appear
"""

from __future__ import annotations

import argparse
import io
import json
import struct
import threading
import time
import uuid
import webbrowser
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

import estimates
import fairing as fairing_module
from scene import KNOWN_BACKENDS, FairingSpec, Geometry, ResultSet, Scene
from solvers import available_solvers, run_scene


SRC_DIR = Path(__file__).parent
PROJECT_ROOT = SRC_DIR.parent
WEB_DIR = SRC_DIR / "web"
# User data lives beside the project, not inside the source tree.
SCENES_DIR = PROJECT_ROOT / "scenes"
SAMPLE_STL = PROJECT_ROOT / "sample.stl"


class SceneState:
    """The active scene, guarded so the job thread and requests can share it."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._scene: Scene | None = None
        self.source_path: Path | None = None

    def set(self, scene: Scene, source_path: Path | None = None) -> None:
        with self._lock:
            self._scene = scene
            self.source_path = source_path

    def get(self) -> Scene:
        with self._lock:
            if self._scene is None:
                raise HTTPException(status_code=409, detail="No scene loaded")
            return self._scene

    def get_optional(self) -> Scene | None:
        with self._lock:
            return self._scene


class Job:
    def __init__(self, backends: list[str], kind: str = "run") -> None:
        self.id = uuid.uuid4().hex[:12]
        self.kind = kind
        self.backends = backends
        self.status = "running"
        self.events: list[dict] = []
        self.error: str | None = None
        self.started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        self.started_at = time.time()
        self.results: dict | None = None
        self.estimate: dict | None = None
        self.progress: dict | None = None
        self._lock = threading.Lock()

    def add_event(self, event: dict) -> None:
        with self._lock:
            if event.get("progress"):
                self.progress = event["progress"]
            if event.get("estimate"):
                self.estimate = event["estimate"]
            self.events.append({**event, "at": datetime.now(timezone.utc).isoformat(timespec="seconds")})

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "id": self.id,
                "kind": self.kind,
                "status": self.status,
                "backends": self.backends,
                "started": self.started,
                "elapsed_seconds": time.time() - self.started_at,
                "events": list(self.events),
                "error": self.error,
                "results": self.results,
                "estimate": self.estimate,
                "progress": self.progress,
            }


class FairingState:
    """The packaging analysis for the current payload.

    Candidate meshes are held in memory rather than in the scene: only the one
    the user picks becomes the scene's hull. The rest are proposals.
    """

    def __init__(self) -> None:
        self.sweep: dict | None = None
        self.candidates: list = []
        self.results: list[dict | None] = []
        self.warnings: list[str] = []
        self.selected: int | None = None
        self.title: str = ""
        self.description: str = ""

    def clear(self) -> None:
        self.sweep = None
        self.candidates = []
        self.results = []
        self.warnings = []
        self.selected = None
        self.title = ""
        self.description = ""

    def to_dict(self) -> dict:
        entries = []
        for index, candidate in enumerate(self.candidates):
            entry = candidate.to_dict()
            entry["index"] = index
            entry["selected"] = index == self.selected
            entry["results"] = self.results[index] if index < len(self.results) else None
            entries.append(entry)
        return {
            "sweep": self.sweep,
            "candidates": entries,
            "warnings": list(self.warnings),
            "selected": self.selected,
            "title": self.title,
            "description": self.description,
        }


state = SceneState()
fairings = FairingState()
jobs: dict[str, Job] = {}
jobs_lock = threading.Lock()

app = FastAPI(title="Aero drag tool")


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def scene_payload(scene: Scene) -> dict:
    """Scene plus everything the UI derives from it, minus the STL blob."""
    metrics = scene.metrics()
    advice = scene.reynolds_advice(metrics.streamwise_length)
    mode, _ = scene.resolved_sweep_mode(metrics.streamwise_length)
    return {
        "scene": scene.to_dict(include_geometry=False),
        "metrics": metrics.to_dict(),
        "reynolds": advice.to_dict(),
        "resolved_mode": mode,
        "computed": scene.computed,
        "has_payload": scene.payload is not None,
        "estimate": estimates.estimate_scene(scene),
        "source_path": str(state.source_path) if state.source_path else None,
        "fairing": fairings.to_dict(),
        # So a browser that loads mid-run picks the job up and locks its inputs
        # instead of showing an idle panel over a solve in progress.
        "active_job": active_job(),
    }


def active_job() -> dict | None:
    with jobs_lock:
        running = next((job for job in jobs.values() if job.status == "running"), None)
    return {"id": running.id, "kind": running.kind} if running else None


def encode_mesh(mesh) -> bytes:
    """Pack a mesh as [uint32 triangles][float32 positions][float32 normals].

    Sent once per geometry change; the browser applies the attitude and ride
    height itself so dragging a slider costs nothing on the wire.
    """
    triangles = np.asarray(mesh.triangles, dtype=np.float32)  # (T, 3, 3)
    count = int(triangles.shape[0])
    face_normals = np.asarray(mesh.face_normals, dtype=np.float32)  # (T, 3)
    normals = np.repeat(face_normals[:, None, :], 3, axis=1).astype(np.float32)
    return struct.pack("<I", count) + triangles.tobytes() + normals.tobytes()


def centered_mesh(scene: Scene):
    """The raw mesh centred on its centroid: the client applies the rest."""
    mesh = scene.geometry.raw_mesh()
    mesh.apply_translation(-np.asarray(mesh.centroid, dtype=float))
    return mesh


def apply_patch(scene: Scene, patch: dict) -> Scene:
    """Merge a partial update from the UI into the scene."""
    merged = scene.to_dict(include_geometry=False)
    for section in ("orientation", "wind", "road", "fluid", "solver", "packaging"):
        if section in patch and isinstance(patch[section], dict):
            current = dict(merged.get(section) or {})
            # A wind vector in the patch would win over the angles; drop it so
            # the angles the UI sends stay authoritative.
            current.pop("vector", None)
            current.update(patch[section])
            merged[section] = current
    if "name" in patch:
        merged["name"] = patch["name"]
    if "notes" in patch:
        merged["notes"] = patch["notes"]

    # to_dict(include_geometry=False) strips the embedded STLs, so both have to
    # be put back or a patch of any unrelated field would silently drop them.
    merged["geometry"] = scene.geometry.to_dict()
    merged["payload"] = scene.payload.to_dict() if scene.payload else None
    merged["results"] = scene.results.to_dict() if scene.results else None
    return Scene.from_dict(merged)


def safe_scene_name(name: str) -> str:
    cleaned = "".join(char for char in name if char.isalnum() or char in "-_. ").strip()
    if not cleaned:
        raise HTTPException(status_code=400, detail="Invalid scene name")
    if not cleaned.endswith(".json"):
        cleaned += ".aero.json"
    return cleaned


# --------------------------------------------------------------------------
# API
# --------------------------------------------------------------------------


@app.get("/api/solvers")
def get_solvers() -> dict:
    return {"solvers": [info.to_dict() for info in available_solvers()]}


@app.get("/api/scene")
def get_scene() -> dict:
    scene = state.get_optional()
    if scene is None:
        return {"scene": None, "computed": False}
    return scene_payload(scene)


def _adopt_geometry(geometry: Geometry, name: str) -> Scene:
    """Load an STL as both the hull and the payload.

    One import path serves both jobs: the STL can be flown as-is to get its
    drag, or treated as the thing a fairing has to enclose. Which one happens
    is decided by the button pressed afterwards, not by the import.
    """
    existing = state.get_optional()
    scene = existing.without_results() if existing is not None else Scene(geometry=geometry)
    scene.geometry = geometry
    scene.payload = geometry
    scene.fairing = None
    scene.name = name
    scene.run_index = 0
    fairings.clear()
    state.set(scene, None)
    return scene


@app.post("/api/scene/stl")
async def upload_stl(file: UploadFile = File(...)) -> dict:
    data = await file.read()
    try:
        geometry = Geometry.from_bytes(data, source_name=file.filename or "hull.stl")
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read STL: {error}") from error
    return scene_payload(_adopt_geometry(geometry, Path(file.filename or "hull").stem))


@app.post("/api/scene/file")
async def upload_scene(file: UploadFile = File(...)) -> dict:
    data = await file.read()
    try:
        scene = Scene.from_dict(json.loads(data.decode("utf-8")))
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read scene: {error}") from error
    state.set(scene, None)
    return scene_payload(scene)


# Bundled example payloads: a unit cube for checking numbers against a known
# Cd, and a mock tadpole trike whose four separate bodies exercise the
# packaging sweep.
SAMPLES = {
    "cube": ("sample.stl", "cube"),
    "trike": ("sample2.stl", "trike"),
}


@app.post("/api/scene/sample")
def load_sample(body: dict | None = None) -> dict:
    key = str((body or {}).get("name") or "cube")
    if key not in SAMPLES:
        raise HTTPException(status_code=404, detail=f"No sample named {key!r}")
    filename, scene_name = SAMPLES[key]
    path = PROJECT_ROOT / filename
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"{filename} is missing")
    geometry = Geometry.from_bytes(path.read_bytes(), source_name=filename)
    return scene_payload(_adopt_geometry(geometry, scene_name))


@app.patch("/api/scene")
def patch_scene(patch: dict) -> dict:
    scene = state.get()
    try:
        updated = apply_patch(scene, patch)
    except Exception as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    state.set(updated, state.source_path)
    return scene_payload(updated)


@app.get("/api/scene/mesh")
def get_mesh() -> Response:
    scene = state.get()
    return Response(content=encode_mesh(centered_mesh(scene)), media_type="application/octet-stream")


@app.get("/api/scene/payload-mesh")
def get_payload_mesh() -> Response:
    """The payload in the hull's frame, so the browser can place both alike."""
    scene = state.get()
    if scene.payload is None:
        raise HTTPException(status_code=404, detail="No payload loaded")
    hull = scene.geometry.raw_mesh()
    mesh = scene.payload.raw_mesh()
    # Offset by the *hull's* centroid, not its own: the client applies one
    # placement transform to both, and it is derived from the hull.
    mesh.apply_translation(-np.asarray(hull.centroid, dtype=float))
    return Response(content=encode_mesh(mesh), media_type="application/octet-stream")


# --------------------------------------------------------------------------
# Payload and fairing generation
# --------------------------------------------------------------------------


@app.post("/api/fairing/analyze")
def analyze_fairing(body: dict | None = None) -> dict:
    """Sweep the closing radius and build one fairing per topology plateau."""
    scene = state.get()
    if scene.payload is None:
        raise HTTPException(status_code=409, detail="Load a payload STL first")

    body = body or {}
    clearance = scene.packaging.clearance
    anisotropy = scene.packaging.anisotropy
    resolution = scene.packaging.resolution
    limit = int(body.get("limit", 4))
    run_index, title = scene.next_title("packaging")
    scene.run_index = run_index

    with jobs_lock:
        if any(job.status == "running" for job in jobs.values()):
            raise HTTPException(status_code=409, detail="Something is already running")
        job = Job([], kind="fairing")
        jobs[job.id] = job

    payload_mesh = scene.payload.raw_mesh()
    direction = scene.wind.direction()

    def worker() -> None:
        try:
            job.add_event({"phase": "voxelise", "message": "Voxelising the payload"})
            grid = fairing_module.build_grid(
                payload_mesh, direction=direction, resolution=max(resolution // 2, 32),
                anisotropy=anisotropy,
            )
            result = grid_sweep = fairing_module.sweep(
                grid, anisotropy=anisotropy, progress=job.add_event, clearance=clearance
            )

            job.add_event({"phase": "build", "message": "Building the candidate fairings"})
            fine = fairing_module.build_grid(
                payload_mesh, direction=direction, resolution=resolution, anisotropy=anisotropy
            )
            candidates = fairing_module.candidates_from_sweep(
                grid,
                payload_mesh,
                grid_sweep,
                direction=direction,
                clearance=clearance,
                limit=limit,
                progress=job.add_event,
                build_grid_override=fine,
            )
            # Most separate bodies first: that is the natural progression from
            # tight local pods to one merged shell.
            candidates.sort(key=lambda item: -item.components)

            fairings.clear()
            fairings.title = title
            fairings.sweep = result.to_dict()
            fairings.candidates = candidates
            fairings.results = [None] * len(candidates)
            fairings.warnings = list(grid.warnings) + list(fine.warnings)
            for candidate in candidates:
                if candidate.choked:
                    fairings.warnings.append(
                        f"The {candidate.components}-body option leaves only "
                        f"{candidate.min_gap * 1000:.0f} mm between bodies. That channel chokes; "
                        "prefer a merged option unless you can open the gap."
                    )
                if candidate.contains_payload is False:
                    fairings.warnings.append(
                        f"The {candidate.components}-body option does not fully enclose the payload."
                    )

            job.results = fairings.to_dict()
            job.status = "done"
            job.add_event({"phase": "done", "message": f"{len(candidates)} candidate fairings ready"})
        except Exception as error:
            job.status = "failed"
            job.error = f"{type(error).__name__}: {error}"
            job.add_event({"phase": "error", "message": job.error})

    threading.Thread(target=worker, name=f"aero-fairing-{job.id}", daemon=True).start()
    return {"job": job.snapshot()}


@app.get("/api/fairing")
def get_fairing() -> dict:
    return fairings.to_dict()


def _select_candidate(scene: Scene, index: int) -> Scene:
    if index < 0 or index >= len(fairings.candidates):
        raise HTTPException(status_code=404, detail="No such candidate")
    candidate = fairings.candidates[index]
    updated = scene.without_results()
    updated.geometry = Geometry.from_bytes(
        candidate.mesh.export(file_type="stl"),
        source_name=f"fairing_{candidate.components}body.stl",
    )
    updated.fairing = FairingSpec(
        closing_radius=candidate.radius,
        clearance=candidate.clearance,
        anisotropy=float((fairings.sweep or {}).get("anisotropy", fairing_module.DEFAULT_ANISOTROPY)),
        components=candidate.components,
        plateau_width=candidate.plateau_width,
    )
    fairings.selected = index
    return updated


@app.post("/api/fairing/select")
def select_fairing(body: dict) -> dict:
    scene = state.get()
    updated = _select_candidate(scene, int(body.get("index", -1)))
    state.set(updated, state.source_path)
    return scene_payload(updated)


@app.post("/api/quality")
def set_quality(body: dict) -> dict:
    scene = state.get()
    name = str(body.get("quality") or "balanced")
    try:
        scene.solver.apply_preset(name)
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    state.set(scene, state.source_path)
    return scene_payload(scene)


@app.get("/api/scene/download")
def download_scene() -> Response:
    scene = state.get()
    body = json.dumps(scene.to_dict(), indent=2)
    suffix = "solved" if scene.computed else "scene"
    filename = f"{scene.name}.{suffix}.aero.json"
    return Response(
        content=body,
        media_type="application/json",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@app.get("/api/scene/hull.stl")
def download_hull() -> Response:
    """The hull as the solvers see it: placed on the road, ready for CAD."""
    scene = state.get()
    body = scene.placed_mesh().export(file_type="stl")
    suffix = f"_{scene.fairing.components}body" if scene.fairing else ""
    return Response(
        content=body,
        media_type="model/stl",
        headers={"Content-Disposition": f'attachment; filename="{scene.name}{suffix}.stl"'},
    )


@app.patch("/api/results")
def patch_results(body: dict) -> dict:
    """Edit the title or description of the stored computation."""
    scene = state.get()
    if scene.results is None:
        raise HTTPException(status_code=409, detail="Nothing computed yet")
    if "title" in body:
        scene.results.title = str(body["title"])
    if "description" in body:
        scene.results.description = str(body["description"])
    state.set(scene, state.source_path)
    return scene_payload(scene)


@app.patch("/api/fairing/meta")
def patch_fairing_meta(body: dict) -> dict:
    """Edit the title or description of the packaging analysis."""
    if not fairings.candidates:
        raise HTTPException(status_code=409, detail="No packaging analysis yet")
    if "title" in body:
        fairings.title = str(body["title"])
    if "description" in body:
        fairings.description = str(body["description"])
    return fairings.to_dict()


@app.get("/api/library")
def list_library() -> dict:
    SCENES_DIR.mkdir(exist_ok=True)
    entries = []
    for path in sorted(SCENES_DIR.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        results = data.get("results")
        entries.append(
            {
                "name": path.name,
                "scene_name": data.get("name") or path.stem,
                "computed": bool(results and results.get("runs")),
                "modified": datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat(
                    timespec="seconds"
                ),
                "size": path.stat().st_size,
            }
        )
    return {"scenes": entries, "directory": str(SCENES_DIR)}


@app.post("/api/library/save")
def save_to_library(body: dict) -> dict:
    scene = state.get()
    name = safe_scene_name(body.get("name") or scene.name)
    SCENES_DIR.mkdir(exist_ok=True)
    path = SCENES_DIR / name
    scene.save(path)
    state.set(scene, path)
    return {"saved": str(path), "library": list_library()["scenes"]}


@app.post("/api/library/load")
def load_from_library(body: dict) -> dict:
    name = safe_scene_name(body.get("name") or "")
    path = SCENES_DIR / name
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"{name} not found")
    try:
        scene = Scene.load(path)
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read {name}: {error}") from error
    state.set(scene, path)
    return scene_payload(scene)


@app.post("/api/run")
def start_run(body: dict | None = None) -> dict:
    scene = state.get()
    body = body or {}
    backends = body.get("backends") or scene.solver.backends
    backends = [name for name in backends if name in KNOWN_BACKENDS]
    if not backends:
        raise HTTPException(status_code=400, detail="No solvers selected")
    # Record what this scene was actually run with, so a saved scene says so.
    scene.solver.backends = backends
    run_index, default_title = scene.next_title("drag")
    scene.run_index = run_index

    with jobs_lock:
        running = [job for job in jobs.values() if job.status == "running"]
        if running:
            raise HTTPException(status_code=409, detail="A run is already in progress")
        job = Job(backends)
        jobs[job.id] = job

    def worker() -> None:
        try:
            results = run_scene(scene, backends=backends, progress=job.add_event)
            results.title = default_title
            scene.results = results
            state.set(scene, state.source_path)
            job.results = results.to_dict()
            job.status = "done"
            job.add_event({"phase": "done", "message": "Run complete"})
        except Exception as error:  # surfaced to the UI rather than swallowed
            job.status = "failed"
            job.error = f"{type(error).__name__}: {error}"
            job.add_event({"phase": "error", "message": job.error})

    thread = threading.Thread(target=worker, name=f"aero-run-{job.id}", daemon=True)
    thread.start()
    return {"job": job.snapshot()}


@app.post("/api/fairing/compare")
def compare_candidates(body: dict | None = None) -> dict:
    """Run every candidate fairing through the solvers and rank them.

    This is the step that answers the actual question -- one lump or several --
    with numbers rather than geometry heuristics. It defaults to the screening
    preset because ranking candidates against each other needs consistency far
    more than it needs absolute accuracy.
    """
    scene = state.get()
    if not fairings.candidates:
        raise HTTPException(status_code=409, detail="Analyse the payload first")

    body = body or {}
    backends = body.get("backends") or scene.solver.backends
    backends = [name for name in backends if name in KNOWN_BACKENDS]
    if not backends:
        raise HTTPException(status_code=400, detail="No solvers selected")
    quality = str(body.get("quality") or "screening")
    include_choked = bool(body.get("include_choked", False))
    compare_index, compare_title = scene.next_title("comparison")
    scene.run_index = compare_index

    indices = [
        index
        for index, candidate in enumerate(fairings.candidates)
        if include_choked or not candidate.choked
    ]
    if not indices:
        raise HTTPException(
            status_code=409,
            detail="Every candidate has a choked gap. Re-analyse with more clearance, "
            "or compare anyway with include_choked.",
        )

    with jobs_lock:
        if any(job.status == "running" for job in jobs.values()):
            raise HTTPException(status_code=409, detail="Something is already running")
        job = Job(backends, kind="compare")
        jobs[job.id] = job

    def worker() -> None:
        try:
            original = state.get()
            for position, index in enumerate(indices):
                candidate = fairings.candidates[index]
                job.add_event(
                    {
                        "phase": "candidate",
                        "message": f"Candidate {position + 1}/{len(indices)}: "
                        f"{candidate.components}-body fairing",
                    }
                )
                trial = _select_candidate(original, index)
                trial.solver.apply_preset(quality)
                trial.solver.backends = backends
                results = run_scene(trial, backends=backends, progress=job.add_event)
                results.title = f"{compare_title} - {candidate.components}-body"
                fairings.results[index] = results.to_dict()
                job.results = fairings.to_dict()

            # Leave the best one selected, with its own results attached, so
            # the user lands on a working scene showing the winning curve
            # rather than an apparently uncomputed one.
            fairings.title = compare_title
            best = _rank_candidates()
            if best:
                winner = _select_candidate(original, best[0]["index"])
                stored = fairings.results[best[0]["index"]]
                if stored:
                    winner.results = ResultSet.from_dict(stored)
                state.set(winner, state.source_path)

            job.status = "done"
            job.add_event({"phase": "done", "message": "Comparison complete"})
        except Exception as error:
            job.status = "failed"
            job.error = f"{type(error).__name__}: {error}"
            job.add_event({"phase": "error", "message": job.error})

    threading.Thread(target=worker, name=f"aero-compare-{job.id}", daemon=True).start()
    return {"job": job.snapshot()}


def _rank_candidates() -> list[dict]:
    """Order compared candidates by drag area, best first.

    Cd x A rather than Cd: a shape can post a flattering coefficient purely by
    being bigger, since Cd is normalised by the frontal area it is quoted on.
    """
    ranked = []
    for index, results in enumerate(fairings.results):
        if not results:
            continue
        candidate = fairings.candidates[index]
        for run in results.get("runs", []):
            if run.get("status") != "ok":
                continue
            points = run.get("points") or []
            solved = next((p for p in points if p.get("source") == "solved"), points[0] if points else None)
            if not solved:
                continue
            ranked.append(
                {
                    "index": index,
                    "components": candidate.components,
                    "solver": run["solver"],
                    "drag_coefficient": solved["drag_coefficient"],
                    "frontal_area": solved["frontal_area"],
                    "drag_area": solved["drag_coefficient"] * solved["frontal_area"],
                }
            )
            break
    return sorted(ranked, key=lambda item: item["drag_area"])


@app.get("/api/fairing/ranking")
def get_ranking() -> dict:
    return {"ranking": _rank_candidates()}


@app.get("/api/estimate")
def get_estimate(backends: str | None = None) -> dict:
    scene = state.get()
    selected = backends.split(",") if backends else None
    return estimates.estimate_scene(scene, selected)


@app.get("/api/jobs/{job_id}")
def get_job(job_id: str) -> dict:
    job = jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Unknown job")
    payload = job.snapshot()
    if payload["status"] == "done":
        scene = state.get_optional()
        if scene is not None:
            payload["scene"] = scene_payload(scene)
    return payload


@app.get("/api/results")
def get_results() -> dict:
    scene = state.get()
    return {"results": scene.results.to_dict() if scene.results else None}


@app.delete("/api/results")
def clear_results() -> dict:
    scene = state.get()
    scene.results = None
    state.set(scene, state.source_path)
    return scene_payload(scene)


# --------------------------------------------------------------------------
# Static front end
# --------------------------------------------------------------------------


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


app.mount("/web", StaticFiles(directory=WEB_DIR), name="web")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the aero drag GUI server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true", help="Do not open a browser window")
    parser.add_argument("--scene", help="Scene file to load at startup")
    args = parser.parse_args(argv)

    if args.scene:
        state.set(Scene.load(args.scene), Path(args.scene))

    import uvicorn

    url = f"http://{args.host}:{args.port}"
    print(f"Aero drag GUI on {url}")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
