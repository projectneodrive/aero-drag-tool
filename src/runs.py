"""Runs: the unit of work the whole tool is organised around.

A **run** is one shape, the parameters it was given, and the results that came
back. Once it has been solved it never changes again -- editing a parameter
afterwards does not rewrite it, and pressing compute forks a new run carrying
the edits. The history is therefore the record: every number on screen can be
traced to the exact inputs that produced it, and a run you solved an hour ago
still says what it said then.

Two kinds exist:

``drag``
    A shape flown at a set of conditions. ``scene.geometry`` is the shape,
    ``scene.results`` is what the solvers returned.

``shape``
    A payload wrapped in a single-body fairing. ``scene.geometry`` is the
    payload going in and :attr:`Run.shell_geometry` is the shell coming out;
    adopting that shell opens a new ``drag`` run, which is how the design loop
    closes.

The as-run snapshot in :attr:`Run.solved_params` is what makes editing a
solved run safe. It is taken at the moment the solver is handed the scene, so
:func:`Run.changed_parameters` can always say precisely which knobs have moved
since -- and the UI can show the old value beside the new one rather than
quietly presenting a curve that belongs to different inputs.
"""

from __future__ import annotations

import threading
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone

from scene import Geometry, Scene


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# Every parameter that changes what a solve produces, with how to present it.
# Anything not in here (the run's own title, say) can be edited freely without
# making the stored results stale, which is exactly the distinction the UI
# needs to draw.
TRACKED_PARAMETERS: tuple[tuple[str, str, str, str | None, int | None], ...] = (
    ("wind", "speed", "Wind speed", "m/s", 2),
    ("wind", "azimuth_deg", "Wind azimuth", "°", 1),
    ("wind", "elevation_deg", "Wind elevation", "°", 1),
    ("orientation", "yaw_deg", "Yaw", "°", 1),
    ("orientation", "pitch_deg", "Pitch", "°", 1),
    ("orientation", "roll_deg", "Roll", "°", 1),
    ("road", "enabled", "Road present", None, None),
    ("road", "ride_height", "Ride height", "m", 3),
    ("road", "moving", "Road moves with the flow", None, None),
    ("fluid", "density", "Air density", "kg/m³", 4),
    ("fluid", "viscosity", "Viscosity", "Pa·s", None),
    ("solver", "backends", "Solvers", None, None),
    ("solver", "quality", "Quality", None, None),
    ("solver", "reference_speed", "Reference speed", "m/s", 2),
    ("solver", "speed_min", "Curve from", "m/s", 2),
    ("solver", "speed_max", "Curve to", "m/s", 2),
    ("solver", "speed_points", "Curve points", None, 0),
    ("solver", "sweep_mode", "Speed handling", None, None),
    ("solver", "turbulence", "Turbulence", None, None),
    ("solver", "iterations", "Iterations", None, 0),
    ("solver", "mesh_resolution", "Mesh resolution", None, 0),
    ("solver", "processes", "MPI ranks", None, None),
)

# The packaging knobs, tracked the same way but only for shape runs.
TRACKED_PACKAGING: tuple[tuple[str, str, str, str | None, int | None], ...] = (
    ("packaging", "clearance", "Payload clearance", "m", 3),
    ("packaging", "anisotropy", "Streamwise bias", None, 1),
    ("packaging", "resolution", "Voxel resolution", None, 0),
    ("packaging", "streamline", "Streamlined envelope", None, None),
    ("packaging", "nose_angle_deg", "Nose angle", "°", 0),
    ("packaging", "tail_angle_deg", "Tail angle", "°", 0),
    ("packaging", "shape_solver", "Shape solver", None, None),
)


def _format_value(value, unit: str | None, decimals: int | None) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "on" if value else "off"
    if isinstance(value, (list, tuple)):
        return ", ".join(str(item) for item in value) or "none"
    if isinstance(value, (int, float)) and decimals is not None:
        text = f"{float(value):.{decimals}f}"
    elif isinstance(value, float):
        text = f"{value:.4g}"
    else:
        text = str(value)
    return f"{text} {unit}" if unit else text


def _same(left, right) -> bool:
    if isinstance(left, (list, tuple)) or isinstance(right, (list, tuple)):
        return list(left or []) == list(right or [])
    if isinstance(left, bool) or isinstance(right, bool):
        return bool(left) == bool(right)
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right)) < 1e-9
    return left == right


@dataclass
class Run:
    """One shape, its parameters and its results."""

    scene: Scene
    kind: str = "drag"  # "drag" or "shape"
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    index: int = 0
    title: str = ""
    description: str = ""
    status: str = "draft"  # draft | running | done | failed
    created_at: str = field(default_factory=_utc_now)
    solved_at: str | None = None
    duration_s: float | None = None

    # Where this run's shape came from, in words the UI can print directly.
    parent_id: str | None = None
    parent_label: str = ""
    origin: str = "imported"  # imported | sample | opened | forked | derived

    job_id: str | None = None
    error: str | None = None

    # The parameters the solver was actually handed. None until it has run.
    solved_params: dict | None = None

    # Shape runs only.
    shell: dict | None = None
    sweep: dict | None = None
    shell_geometry: Geometry | None = None
    shell_warnings: list[str] = field(default_factory=list)

    # ------------------------------------------------------------- parameters

    def tracked(self) -> tuple:
        return TRACKED_PARAMETERS + (TRACKED_PACKAGING if self.kind == "shape" else ())

    def snapshot_parameters(self) -> dict:
        """The current parameter values, flattened to section.key."""
        data = self.scene.to_dict(include_geometry=False)
        snapshot: dict = {}
        for section, key, _label, _unit, _decimals in self.tracked():
            snapshot[f"{section}.{key}"] = (data.get(section) or {}).get(key)
        return snapshot

    def changed_parameters(self) -> list[dict]:
        """Which knobs have moved since this run was solved.

        Empty for a draft (nothing to differ from) and for a solved run nobody
        has touched. Anything in here means the results on screen belong to
        different inputs than the panel shows, which the UI has to say out
        loud rather than let the reader assume.
        """
        if not self.solved_params:
            return []
        current = self.snapshot_parameters()
        changes = []
        for section, key, label, unit, decimals in self.tracked():
            name = f"{section}.{key}"
            was = self.solved_params.get(name)
            now = current.get(name)
            if _same(was, now):
                continue
            changes.append(
                {
                    "section": section,
                    "key": key,
                    "label": label,
                    "as_run": was,
                    "as_run_text": _format_value(was, unit, decimals),
                    "current": now,
                    "current_text": _format_value(now, unit, decimals),
                }
            )
        return changes

    def as_run_lines(self) -> list[str]:
        """The immutable record of what produced the results, for display."""
        if not self.solved_params:
            return []
        grouped: dict[str, list[str]] = {}
        for section, key, label, unit, decimals in self.tracked():
            value = self.solved_params.get(f"{section}.{key}")
            if value is None and section == "solver" and key == "processes":
                value = "auto"
            grouped.setdefault(section, []).append(
                f"{label} {_format_value(value, unit, decimals)}"
            )
        return [" · ".join(items) for items in grouped.values()]

    # ------------------------------------------------------------------ state

    @property
    def computed(self) -> bool:
        if self.kind == "shape":
            return self.shell is not None
        return self.scene.computed

    def summary(self) -> dict:
        """The little that the tab bar needs, for every open run."""
        return {
            "id": self.id,
            "kind": self.kind,
            "title": self.title,
            "status": self.status,
            "index": self.index,
            "parent_id": self.parent_id,
            "computed": self.computed,
            "created_at": self.created_at,
            "changed_count": len(self.changed_parameters()),
        }

    def to_dict(self) -> dict:
        """Everything but the derived metrics, which the server computes."""
        return {
            **self.summary(),
            "description": self.description,
            "parent_label": self.parent_label,
            "origin": self.origin,
            "job_id": self.job_id,
            "error": self.error,
            "solved_at": self.solved_at,
            "duration_s": self.duration_s,
            "scene": self.scene.to_dict(include_geometry=False),
            "changed": self.changed_parameters(),
            "as_run": self.as_run_lines(),
            "shell": self.shell,
            "sweep": self.sweep,
            "shell_warnings": list(self.shell_warnings),
            "shell_source": self.shell_geometry.source_name if self.shell_geometry else None,
        }


class RunStore:
    """Every open run, in tab order.

    Guarded by a lock because solver threads write results into runs while
    request handlers read them.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._runs: list[Run] = []
        self._counter = 0
        self.active_id: str | None = None

    # --------------------------------------------------------------- indexing

    def next_index(self) -> int:
        with self._lock:
            self._counter += 1
            return self._counter

    def title_for(self, base: str, kind: str) -> tuple[int, str]:
        index = self.next_index()
        operation = "shape" if kind == "shape" else "drag"
        return index, f"{base} · {operation} #{index}"

    # ------------------------------------------------------------------ crud

    def add(self, run: Run, activate: bool = True) -> Run:
        with self._lock:
            self._runs.append(run)
            if activate:
                self.active_id = run.id
            return run

    def get(self, run_id: str) -> Run | None:
        with self._lock:
            return next((run for run in self._runs if run.id == run_id), None)

    def list(self) -> list[Run]:
        with self._lock:
            return list(self._runs)

    def remove(self, run_id: str) -> bool:
        with self._lock:
            run = self.get(run_id)
            if run is None:
                return False
            position = self._runs.index(run)
            self._runs.remove(run)
            if self.active_id == run_id:
                if self._runs:
                    self.active_id = self._runs[min(position, len(self._runs) - 1)].id
                else:
                    self.active_id = None
            return True

    def running(self) -> Run | None:
        with self._lock:
            return next((run for run in self._runs if run.status == "running"), None)

    def summaries(self) -> list[dict]:
        return [run.summary() for run in self.list()]


def base_name(name: str) -> str:
    """Strip the ' · drag #3' suffix so forks keep the original stem."""
    return name.split(" · ")[0].strip() or "untitled"


def fork(run: Run, store: RunStore, description: str = "") -> Run:
    """A new draft run carrying this one's shape and its edited parameters.

    The parent keeps its results untouched; the child starts clean. This is
    what makes a solved run safe to poke at -- the edits go somewhere new
    rather than invalidating what is already on screen.
    """
    scene = run.scene.without_results()
    base = base_name(run.title)
    index, title = store.title_for(base, "drag")
    changed = run.changed_parameters()
    if not description:
        if changed:
            names = ", ".join(item["label"].lower() for item in changed[:3])
            more = f" and {len(changed) - 3} more" if len(changed) > 3 else ""
            description = f"Re-run of {run.title} with a different {names}{more}."
        else:
            description = f"Repeat of {run.title} with the same parameters."
    return Run(
        scene=scene,
        kind="drag",
        index=index,
        title=title,
        description=description,
        parent_id=run.id,
        parent_label=run.title,
        origin="forked",
    )
