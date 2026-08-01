"""Local web server for the aero drag GUI.

Holds a set of open :class:`~runs.Run` records in memory (this is a
single-user desktop tool), serves the browser front end, and runs solver jobs
on a background thread so the UI stays responsive during a CFD run.

    python server.py            # then open http://127.0.0.1:8000

The unit of work is the run, not "the scene". A run is a shape, the
parameters it was given and the results that came back, and it stops changing
once it has been solved -- computing again on a finished run forks a new one
carrying your edits rather than overwriting what is already there. Two verbs
drive everything:

    compute   solve this run's shape at these conditions
    derive    wrap this run's shape in a single-body fairing, as a new run

The freeze while a solve is in flight is **per run**: only the working run
locks its parameters, so every other open run stays readable and editable.

Runs saved into the ``scenes/`` directory are ordinary scene files, so the
offline round trip still works:

    (GUI) save  ->  scenes/case.aero.json
    python runner.py run scenes/case.aero.json
    (GUI) open  ->  a new run with the results in it
"""

from __future__ import annotations

import argparse
import json
import struct
import threading
import time
import uuid
import webbrowser
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

import estimates
import fairing as fairing_module
from execution import available_cores, default_processes
from runs import Run, RunStore, base_name, fork
from scene import KNOWN_BACKENDS, FairingSpec, Geometry, Scene
from solvers import available_solvers, run_scene


SRC_DIR = Path(__file__).parent
PROJECT_ROOT = SRC_DIR.parent
WEB_DIR = SRC_DIR / "web"
# User data lives beside the project, not inside the source tree.
SCENES_DIR = PROJECT_ROOT / "scenes"

# Bundled example payloads: a unit cube for checking numbers against a known
# Cd, and a mock tadpole trike whose four separate bodies are what the shape
# search exists for.
SAMPLES = {
    "cube": ("sample.stl", "cube"),
    "trike": ("sample2.stl", "trike"),
}


class Job:
    """One background piece of work, attached to the run that owns it."""

    def __init__(self, run_id: str, kind: str = "drag", backends: list[str] | None = None) -> None:
        self.id = uuid.uuid4().hex[:12]
        self.run_id = run_id
        self.kind = kind
        self.backends = list(backends or [])
        self.status = "running"
        self.events: list[dict] = []
        self.error: str | None = None
        self.started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        self.started_at = time.time()
        self.estimate: dict | None = None
        self.progress: dict | None = None
        self._lock = threading.Lock()

    def add_event(self, event: dict) -> None:
        with self._lock:
            if event.get("progress"):
                self.progress = event["progress"]
            if event.get("estimate"):
                self.estimate = event["estimate"]
            self.events.append(
                {**event, "at": datetime.now(timezone.utc).isoformat(timespec="seconds")}
            )

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "id": self.id,
                "run_id": self.run_id,
                "kind": self.kind,
                "status": self.status,
                "backends": self.backends,
                "started": self.started,
                "elapsed_seconds": time.time() - self.started_at,
                "events": list(self.events),
                "error": self.error,
                "estimate": self.estimate,
                "progress": self.progress,
            }


store = RunStore()
jobs: dict[str, Job] = {}
jobs_lock = threading.Lock()

app = FastAPI(title="Aero drag tool")


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def require_run(run_id: str) -> Run:
    run = store.get(run_id)
    if run is None:
        raise HTTPException(status_code=404, detail="No such run")
    return run


def require_idle(run: Run) -> None:
    """Refuse to mutate a run the solver is currently reading."""
    if run.status == "running":
        raise HTTPException(
            status_code=409,
            detail=f"{run.title} is solving. Its parameters are frozen until it finishes.",
        )


def claim_job_slot() -> None:
    """One solve at a time, so two jobs never fight over the same cores."""
    busy = store.running()
    if busy is not None:
        raise HTTPException(
            status_code=409,
            detail=f"{busy.title} is already running. One solve at a time — "
            "reading and editing other runs is unaffected.",
        )


def display_scene(run: Run) -> Scene:
    """The scene as it is shown: a finished shape run displays its shell.

    A shape run carries the payload in ``scene.geometry`` while it works, and
    the shell it produced separately. Swapping the shell in here means the
    viewport, the metrics and the payload placement all derive from one
    geometry, so the picture of the payload sitting inside the shell cannot
    disagree with the numbers beside it.
    """
    if run.kind == "shape" and run.shell_geometry is not None:
        scene = run.scene.copy()
        scene.geometry = run.shell_geometry
        return scene
    return run.scene


def reference_run(run: Run) -> dict | None:
    """The nearest solved ancestor, so a result can be shown as a change.

    Walks the parent chain rather than looking only one step up, because a
    shape run sits between a payload's drag run and the shell's, and the
    comparison worth making skips over it.
    """
    seen = set()
    current = run
    while current.parent_id and current.parent_id not in seen:
        seen.add(current.parent_id)
        parent = store.get(current.parent_id)
        if parent is None:
            return None
        if parent.kind == "drag" and parent.scene.computed:
            point = _reference_point(parent)
            if point is not None:
                return {
                    "id": parent.id,
                    "title": parent.title,
                    "drag_area": point["drag_coefficient"] * point["frontal_area"],
                    "drag_coefficient": point["drag_coefficient"],
                    "frontal_area": point["frontal_area"],
                }
        current = parent
    return None


def _reference_point(run: Run) -> dict | None:
    results = run.scene.results
    if results is None:
        return None
    for solver_run in results.runs:
        if solver_run.status != "ok":
            continue
        point = solver_run.reference_point()
        if point is not None:
            return point.to_dict()
    return None


def run_payload(run: Run) -> dict:
    """Everything one run's panels need, minus the STL blobs."""
    scene = display_scene(run)
    metrics = scene.metrics()
    advice = scene.reynolds_advice(metrics.streamwise_length)
    mode, _ = scene.resolved_sweep_mode(metrics.streamwise_length)

    payload = run.to_dict()
    payload.update(
        {
            "metrics": metrics.to_dict(),
            "reynolds": advice.to_dict(),
            "resolved_mode": mode,
            "estimate": estimates.estimate_scene(scene),
            "results": scene.results.to_dict() if scene.results else None,
            "drag_area": None,
            "reference": reference_run(run),
            # The payload is only worth drawing once there is a shell to see
            # it through; before that it and the hull are the same mesh.
            "show_payload": bool(
                run.scene.payload is not None
                and (run.scene.fairing is not None or run.shell_geometry is not None)
            ),
            "job": jobs[run.job_id].snapshot() if run.job_id and run.job_id in jobs else None,
        }
    )
    point = _reference_point(run)
    if point is not None:
        payload["drag_area"] = point["drag_coefficient"] * point["frontal_area"]
    return payload


def state_payload() -> dict:
    busy = store.running()
    processes = None
    active = store.get(store.active_id) if store.active_id else None
    if active is not None:
        processes = active.scene.solver.processes
    return {
        "runs": store.summaries(),
        "active_id": store.active_id,
        "running_id": busy.id if busy else None,
        "running_job": busy.job_id if busy else None,
        "solvers": [info.to_dict() for info in available_solvers(processes)],
        "cores": {
            "available": available_cores(),
            "default_processes": default_processes(),
            "processes": processes,
        },
    }


def encode_mesh(mesh, crease_deg: float = 30.0) -> bytes:
    """Pack a mesh as [uint32 triangles][float32 positions][float32 normals].

    Sent once per geometry change; the browser applies the attitude and ride
    height itself so dragging a slider costs nothing on the wire.

    Normals are smooth-shaded with creases preserved: each corner takes the
    averaged vertex normal unless that disagrees with its own face by more
    than ``crease_deg``, in which case the corner stays flat. A generated
    fairing renders as the smooth surface it is, while a payload cube keeps
    its sharp edges instead of looking inflated.
    """
    triangles = np.asarray(mesh.triangles, dtype=np.float32)  # (T, 3, 3)
    count = int(triangles.shape[0])
    face_normals = np.asarray(mesh.face_normals, dtype=np.float64)  # (T, 3)

    corner = np.asarray(mesh.vertex_normals, dtype=np.float64)[mesh.faces]  # (T, 3, 3)
    agreement = np.einsum("tcj,tj->tc", corner, face_normals)  # cos per corner
    flat = agreement < np.cos(np.radians(crease_deg))
    if flat.any():
        corner[flat] = np.repeat(face_normals[:, None, :], 3, axis=1)[flat]

    return struct.pack("<I", count) + triangles.tobytes() + corner.astype(np.float32).tobytes()


def apply_patch(scene: Scene, patch: dict) -> Scene:
    """Merge a partial update from the UI into a scene."""
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


def new_drag_run(geometry: Geometry, base: str, origin: str, description: str) -> Run:
    """A fresh draft run around an imported shape."""
    scene = Scene(geometry=geometry, name=base)
    # Every import is both: a shape that can be flown as-is, and a payload a
    # fairing could be derived around. Which one happens is the button.
    scene.payload = geometry
    index, title = store.title_for(base, "drag")
    return store.add(
        Run(
            scene=scene,
            kind="drag",
            index=index,
            title=title,
            description=description,
            origin=origin,
        )
    )


# --------------------------------------------------------------------------
# State and run listing
# --------------------------------------------------------------------------


@app.get("/api/state")
def get_state() -> dict:
    return state_payload()


@app.get("/api/solvers")
def get_solvers() -> dict:
    return {
        "solvers": state_payload()["solvers"],
        "cores": state_payload()["cores"],
    }


@app.get("/api/runs/{run_id}")
def get_run(run_id: str) -> dict:
    return run_payload(require_run(run_id))


@app.post("/api/runs/{run_id}/activate")
def activate_run(run_id: str) -> dict:
    require_run(run_id)
    store.active_id = run_id
    return state_payload()


@app.delete("/api/runs/{run_id}")
def close_run(run_id: str) -> dict:
    run = require_run(run_id)
    require_idle(run)
    store.remove(run_id)
    return state_payload()


# --------------------------------------------------------------------------
# Opening shapes
# --------------------------------------------------------------------------


@app.post("/api/runs/import")
async def import_stl(file: UploadFile = File(...)) -> dict:
    data = await file.read()
    name = file.filename or "hull.stl"
    try:
        geometry = Geometry.from_bytes(data, source_name=name)
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read STL: {error}") from error
    run = new_drag_run(
        geometry, Path(name).stem, "imported", f"Imported from {name}, not solved yet."
    )
    return {"run": run_payload(run), "state": state_payload()}


@app.post("/api/runs/sample")
def load_sample(body: dict | None = None) -> dict:
    key = str((body or {}).get("name") or "cube")
    if key not in SAMPLES:
        raise HTTPException(status_code=404, detail=f"No sample named {key!r}")
    filename, base = SAMPLES[key]
    path = PROJECT_ROOT / filename
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"{filename} is missing")
    geometry = Geometry.from_bytes(path.read_bytes(), source_name=filename)
    run = new_drag_run(geometry, base, "sample", f"The bundled {base} sample.")
    return {"run": run_payload(run), "state": state_payload()}


def _run_from_scene(scene: Scene, origin: str, description: str) -> Run:
    base = base_name(scene.name)
    index, title = store.title_for(base, "drag")
    run = Run(
        scene=scene,
        kind="drag",
        index=index,
        title=(scene.results.title if scene.results and scene.results.title else title),
        description=(
            scene.results.description if scene.results and scene.results.description else description
        ),
        origin=origin,
        status="done" if scene.computed else "draft",
    )
    if scene.computed:
        # An opened file already carries its results, so the parameters in it
        # are by definition the ones they were produced with.
        run.solved_params = run.snapshot_parameters()
        run.solved_at = scene.results.computed_at
    return store.add(run)


@app.post("/api/runs/open")
async def open_scene_file(file: UploadFile = File(...)) -> dict:
    data = await file.read()
    try:
        scene = Scene.from_dict(json.loads(data.decode("utf-8")))
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read scene: {error}") from error
    run = _run_from_scene(scene, "opened", f"Opened from {file.filename}.")
    return {"run": run_payload(run), "state": state_payload()}


# --------------------------------------------------------------------------
# Editing a run
# --------------------------------------------------------------------------


@app.patch("/api/runs/{run_id}")
def patch_run(run_id: str, patch: dict) -> dict:
    run = require_run(run_id)
    require_idle(run)
    try:
        run.scene = apply_patch(run.scene, patch)
    except Exception as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    return run_payload(run)


@app.patch("/api/runs/{run_id}/meta")
def patch_run_meta(run_id: str, body: dict) -> dict:
    """Title and description are the run's own, and always editable.

    Deliberately not part of the tracked parameters: renaming a run does not
    make its results stale, so it never triggers the changed-parameter
    warning or a fork.
    """
    run = require_run(run_id)
    if "title" in body:
        run.title = str(body["title"]) or run.title
    if "description" in body:
        run.description = str(body["description"])
    if run.scene.results is not None:
        run.scene.results.title = run.title
        run.scene.results.description = run.description
    return run_payload(run)


@app.post("/api/runs/{run_id}/quality")
def set_quality(run_id: str, body: dict) -> dict:
    run = require_run(run_id)
    require_idle(run)
    try:
        run.scene.solver.apply_preset(str(body.get("quality") or "balanced"))
    except ValueError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error
    return run_payload(run)


@app.post("/api/runs/{run_id}/processes")
def set_processes(run_id: str, body: dict) -> dict:
    """Pin the MPI rank count, or clear it back to the 80%-of-cores default."""
    run = require_run(run_id)
    require_idle(run)
    requested = body.get("processes")
    if requested in (None, "", 0):
        run.scene.solver.processes = None
    else:
        try:
            count = int(requested)
        except (TypeError, ValueError) as error:
            raise HTTPException(status_code=400, detail="processes must be a number") from error
        if count < 1:
            raise HTTPException(status_code=400, detail="processes must be at least 1")
        if count > available_cores():
            raise HTTPException(
                status_code=400, detail=f"Only {available_cores()} cores are visible here"
            )
        run.scene.solver.processes = count
    return run_payload(run)


# --------------------------------------------------------------------------
# Compute: solve this run's shape
# --------------------------------------------------------------------------


@app.post("/api/runs/{run_id}/compute")
def compute_run(run_id: str, body: dict | None = None) -> dict:
    """Solve this run, or fork a new one if it has already been solved.

    The fork is the whole point of the run model: a finished run is a record,
    so the edits you made after reading it go into a new run rather than
    overwriting the numbers you were reading.
    """
    run = require_run(run_id)
    require_idle(run)
    claim_job_slot()

    body = body or {}
    backends = body.get("backends") or run.scene.solver.backends
    backends = [name for name in backends if name in KNOWN_BACKENDS]
    if not backends:
        raise HTTPException(status_code=400, detail="No solvers selected")

    target = run
    forked = False
    if run.kind == "shape":
        raise HTTPException(
            status_code=400,
            detail="A shape run produces geometry, not drag. Open its shell as a run first.",
        )
    if run.scene.computed:
        target = fork(run, store)
        store.add(target)
        forked = True

    target.scene.solver.backends = backends
    scene = target.scene

    with jobs_lock:
        job = Job(target.id, kind="drag", backends=backends)
        jobs[job.id] = job

    target.status = "running"
    target.job_id = job.id
    target.error = None
    # Snapshot now, not at the end: these are the values the solver is being
    # handed, and the panel is frozen from this moment so they cannot drift.
    target.solved_params = target.snapshot_parameters()
    started = time.time()

    def worker() -> None:
        try:
            results = run_scene(scene, backends=backends, progress=job.add_event)
            results.title = target.title
            results.description = target.description
            scene.results = results
            target.status = "done"
            target.solved_at = results.computed_at
            target.duration_s = time.time() - started
            job.status = "done"
            job.add_event({"phase": "done", "message": "Run complete"})
        except Exception as error:  # surfaced to the UI rather than swallowed
            target.status = "failed"
            target.error = f"{type(error).__name__}: {error}"
            # A failed run has nothing to be stale against, so drop the
            # snapshot and let it be edited and retried in place.
            target.solved_params = None
            job.status = "failed"
            job.error = target.error
            job.add_event({"phase": "error", "message": target.error})

    threading.Thread(target=worker, name=f"aero-drag-{job.id}", daemon=True).start()
    return {
        "run_id": target.id,
        "forked": forked,
        "job": job.snapshot(),
        "run": run_payload(target),
        "state": state_payload(),
    }


# --------------------------------------------------------------------------
# Derive: wrap this run's shape in a single-body fairing
# --------------------------------------------------------------------------


@app.post("/api/runs/{run_id}/derive")
def derive_shape(run_id: str) -> dict:
    """Open a shape run that wraps this run's geometry in one closed shell.

    Deriving *from a shape run* re-wraps the same payload with whatever the
    packaging knobs now say, rather than wrapping the shell it already built.
    Wrapping a shell in a shell is never what "derive again" means, and it
    would quietly compound the clearance on every pass.
    """
    parent = require_run(run_id)
    claim_job_slot()

    if parent.kind == "shape" and parent.scene.payload is not None:
        payload_geometry = parent.scene.payload
    else:
        payload_geometry = display_scene(parent).geometry
    scene = parent.scene.without_results()
    scene.geometry = payload_geometry
    scene.payload = payload_geometry
    scene.fairing = None

    base = base_name(parent.title)
    index, title = store.title_for(base, "shape")
    run = store.add(
        Run(
            scene=scene,
            kind="shape",
            index=index,
            title=title,
            description=f"Single-body fairing around {parent.title}.",
            parent_id=parent.id,
            parent_label=parent.title,
            origin="derived",
        )
    )

    with jobs_lock:
        job = Job(run.id, kind="shape")
        jobs[job.id] = job

    run.status = "running"
    run.job_id = job.id
    run.solved_params = run.snapshot_parameters()
    started = time.time()

    packaging = scene.packaging
    payload_mesh = payload_geometry.raw_mesh()
    direction = scene.wind.direction()
    streamline = (
        (packaging.nose_angle_deg, packaging.tail_angle_deg) if packaging.streamline else None
    )

    def worker() -> None:
        try:
            job.add_event({"phase": "voxelise", "message": "Voxelising the payload"})
            coarse = fairing_module.build_grid(
                payload_mesh,
                direction=direction,
                resolution=max(packaging.resolution // 2, 32),
                anisotropy=packaging.anisotropy,
            )
            sweep = fairing_module.sweep(
                coarse,
                anisotropy=packaging.anisotropy,
                progress=job.add_event,
                clearance=packaging.clearance,
            )

            job.add_event(
                {
                    "phase": "build",
                    "message": "Building the shell at the smallest radius that merges it",
                }
            )
            # The skin grid gets the streamwise room the nose and tail cones
            # need; the sweep grid above does not, since merging needs no tail.
            fine = fairing_module.build_grid(
                payload_mesh,
                direction=direction,
                resolution=packaging.resolution,
                anisotropy=packaging.anisotropy,
                streamline=streamline,
                clearance=packaging.clearance,
            )
            shell = fairing_module.build_single_shell(
                coarse,
                payload_mesh,
                sweep,
                direction=direction,
                clearance=packaging.clearance,
                progress=job.add_event,
                build_grid_override=fine,
                streamline=streamline,
            )

            warnings = list(coarse.warnings) + list(fine.warnings)
            if shell.bodies > 1:
                warnings.append(
                    f"The shell still came out as {shell.bodies} separate bodies at the largest "
                    "radius the grid can hold. Raise the streamwise bias or the clearance."
                )
            if shell.contains_payload is False:
                warnings.append("The shell does not fully enclose the payload.")
            elif shell.contains_payload is None:
                warnings.append(
                    "The containment check could not run here, so the fit is unverified."
                )
            if shell.attempts > 1:
                warnings.append(
                    f"The sweep's radius produced a split skin on the fine grid; the shell was "
                    f"opened to {shell.radius * 1000:.0f} mm over {shell.attempts} attempts."
                )

            run.shell_geometry = Geometry.from_bytes(
                shell.mesh.export(file_type="stl"), source_name=f"{base}_shell.stl"
            )
            run.shell = shell.to_dict()
            run.sweep = sweep.to_dict()
            run.shell_warnings = warnings
            run.scene.fairing = FairingSpec(
                closing_radius=shell.radius,
                clearance=shell.clearance,
                anisotropy=shell.anisotropy,
                components=1,
                resolution=packaging.resolution,
                streamlined=shell.streamlined,
                nose_angle_deg=shell.nose_angle_deg,
                tail_angle_deg=shell.tail_angle_deg,
            )
            run.status = "done"
            run.solved_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
            run.duration_s = time.time() - started
            job.status = "done"
            job.add_event(
                {
                    "phase": "done",
                    "message": f"Shell ready: r = {shell.radius * 1000:.0f} mm, "
                    f"frontal area {shell.frontal_area:.4f} m²",
                }
            )
        except Exception as error:
            run.status = "failed"
            run.error = f"{type(error).__name__}: {error}"
            run.solved_params = None
            job.status = "failed"
            job.error = run.error
            job.add_event({"phase": "error", "message": run.error})

    threading.Thread(target=worker, name=f"aero-shape-{job.id}", daemon=True).start()
    return {
        "run_id": run.id,
        "job": job.snapshot(),
        "run": run_payload(run),
        "state": state_payload(),
    }


@app.post("/api/runs/{run_id}/adopt")
def adopt_shell(run_id: str) -> dict:
    """Open a shape run's shell as a drag run, ready to solve."""
    parent = require_run(run_id)
    if parent.kind != "shape" or parent.shell_geometry is None:
        raise HTTPException(status_code=409, detail="That run has no shell to open")

    scene = parent.scene.without_results()
    scene.geometry = parent.shell_geometry
    scene.payload = parent.scene.payload
    scene.fairing = parent.scene.fairing

    base = f"{base_name(parent.title)} shell"
    scene.name = base
    index, title = store.title_for(base, "drag")
    shell = parent.shell or {}
    run = store.add(
        Run(
            scene=scene,
            kind="drag",
            index=index,
            title=title,
            description=(
                f"The single-body shell from {parent.title}: closing radius "
                f"{shell.get('radius', 0) * 1000:.0f} mm, clearance "
                f"{shell.get('clearance', 0) * 1000:.0f} mm."
            ),
            parent_id=parent.id,
            parent_label=parent.title,
            origin="derived",
        )
    )
    return {"run": run_payload(run), "state": state_payload()}


# --------------------------------------------------------------------------
# Geometry out
# --------------------------------------------------------------------------


@app.get("/api/runs/{run_id}/mesh")
def get_mesh(run_id: str) -> Response:
    """The displayed shape, centred on its centroid: the client does the rest."""
    run = require_run(run_id)
    mesh = display_scene(run).geometry.raw_mesh()
    mesh.apply_translation(-np.asarray(mesh.centroid, dtype=float))
    return Response(content=encode_mesh(mesh), media_type="application/octet-stream")


@app.get("/api/runs/{run_id}/payload-mesh")
def get_payload_mesh(run_id: str) -> Response:
    """The payload in the hull's frame, so the browser can place both alike."""
    run = require_run(run_id)
    scene = display_scene(run)
    if scene.payload is None:
        raise HTTPException(status_code=404, detail="No payload on this run")
    hull = scene.geometry.raw_mesh()
    mesh = scene.payload.raw_mesh()
    # Offset by the *hull's* centroid, not its own: the client applies one
    # placement transform to both, and it is derived from the hull.
    mesh.apply_translation(-np.asarray(hull.centroid, dtype=float))
    return Response(content=encode_mesh(mesh), media_type="application/octet-stream")


@app.get("/api/runs/{run_id}/download")
def download_run(run_id: str) -> Response:
    """The run as a scene file, which the CLI reads and writes."""
    run = require_run(run_id)
    scene = display_scene(run)
    body = json.dumps(scene.to_dict(), indent=2)
    suffix = "solved" if scene.computed else "scene"
    stem = base_name(run.title).replace(" ", "_")
    return Response(
        content=body,
        media_type="application/json",
        headers={"Content-Disposition": f'attachment; filename="{stem}.{suffix}.aero.json"'},
    )


@app.get("/api/runs/{run_id}/hull.stl")
def download_hull(run_id: str) -> Response:
    """The shape as the solvers see it: placed on the road, ready for CAD."""
    run = require_run(run_id)
    scene = display_scene(run)
    stem = base_name(run.title).replace(" ", "_")
    return Response(
        content=scene.placed_mesh().export(file_type="stl"),
        media_type="model/stl",
        headers={"Content-Disposition": f'attachment; filename="{stem}.stl"'},
    )


# --------------------------------------------------------------------------
# Jobs
# --------------------------------------------------------------------------


@app.get("/api/jobs/{job_id}")
def get_job(job_id: str) -> dict:
    job = jobs.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Unknown job")
    payload = job.snapshot()
    run = store.get(job.run_id)
    if run is not None and payload["status"] != "running":
        payload["run"] = run_payload(run)
        payload["state"] = state_payload()
    return payload


# --------------------------------------------------------------------------
# Library
# --------------------------------------------------------------------------


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
    run = require_run(str(body.get("run_id") or ""))
    scene = display_scene(run)
    name = safe_scene_name(body.get("name") or base_name(run.title))
    SCENES_DIR.mkdir(exist_ok=True)
    path = SCENES_DIR / name
    scene.save(path)
    return {"saved": str(path), "library": list_library()["scenes"]}


@app.post("/api/library/open")
def open_from_library(body: dict) -> dict:
    name = safe_scene_name(body.get("name") or "")
    path = SCENES_DIR / name
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"{name} not found")
    try:
        scene = Scene.load(path)
    except Exception as error:
        raise HTTPException(status_code=400, detail=f"Could not read {name}: {error}") from error
    run = _run_from_scene(scene, "opened", f"Opened from {name}.")
    return {"run": run_payload(run), "state": state_payload()}


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
    parser.add_argument("--scene", help="Scene file to open as a run at startup")
    args = parser.parse_args(argv)

    if args.scene:
        _run_from_scene(Scene.load(args.scene), "opened", f"Opened from {args.scene}.")

    import uvicorn

    url = f"http://{args.host}:{args.port}"
    print(f"Aero drag GUI on {url}")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
