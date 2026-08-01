"""Runtime estimation for solver jobs.

A CFD run can take anywhere from under a minute to several hours, and the
difference is entirely in settings the user just changed. Telling them which
one they are about to start is the difference between a usable tool and one
that appears to have hung.

The model is deliberately simple -- cost is roughly linear in cells x
iterations -- but it **calibrates itself**: every completed solve appends a
sample, and later estimates are fitted to this machine's actual measurements
rather than to numbers baked in by whoever wrote the file.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# Measurements are machine-local data, so they sit beside the project rather
# than inside the source tree.
HISTORY_PATH = Path(__file__).parent.parent / "runtime_history.json"
MAX_SAMPLES = 400

# Seconds per (million cells x thousand iterations), before any local
# measurements exist. Both were taken from first runs on a laptop-class
# machine and are only used until real samples accumulate.
DEFAULT_RATE = {
    "openfoam": 55.0,
    "su2": 90.0,
}

# Fixed overhead per evaluation: meshing, case writing, process start.
DEFAULT_OVERHEAD = {
    "openfoam": 25.0,
    "su2": 30.0,
}

_lock = threading.Lock()


def estimated_cells(solver: str, mesh_resolution: int, refinement_level: int = 3) -> float:
    """Rough cell count for a case, in millions.

    The background grid is ``mesh_resolution`` cells along the longest axis.
    snappyHexMesh then refines near the body, which multiplies the count by
    something like 8^level over a surface-proportional subset -- empirically
    a factor of a few, not a few hundred.
    """
    background = max(float(mesh_resolution), 1.0) ** 3
    refinement_factor = 1.0 + 0.35 * max(int(refinement_level), 0) ** 2
    if solver == "su2":
        # Tetrahedra from gmsh, finer near the surface: denser per unit volume
        # than the hex background grid.
        refinement_factor *= 2.0
    return background * refinement_factor / 1e6


@dataclass
class Sample:
    solver: str
    cells_millions: float
    iterations: int
    seconds: float

    def to_dict(self) -> dict:
        return {
            "solver": self.solver,
            "cells_millions": self.cells_millions,
            "iterations": self.iterations,
            "seconds": self.seconds,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Sample":
        return cls(
            solver=str(data.get("solver") or ""),
            cells_millions=float(data.get("cells_millions") or 0.0),
            iterations=int(data.get("iterations") or 0),
            seconds=float(data.get("seconds") or 0.0),
        )


def load_history(path: Path | None = None) -> list[Sample]:
    path = Path(path or HISTORY_PATH)
    if not path.exists():
        return []
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    return [Sample.from_dict(item) for item in raw.get("samples", [])]


def record(solver: str, cells_millions: float, iterations: int, seconds: float, path: Path | None = None) -> None:
    """Append one measurement. Called after every real solve."""
    if seconds <= 0:
        return
    path = Path(path or HISTORY_PATH)
    with _lock:
        samples = load_history(path)
        samples.append(
            Sample(
                solver=solver,
                cells_millions=float(cells_millions),
                iterations=int(iterations),
                seconds=float(seconds),
            )
        )
        samples = samples[-MAX_SAMPLES:]
        try:
            path.write_text(
                json.dumps({"samples": [item.to_dict() for item in samples]}, indent=1),
                encoding="utf-8",
            )
        except OSError:
            pass


def _fit(samples: list[Sample]) -> tuple[float, float] | None:
    """Least-squares fit of seconds = overhead + rate * work."""
    if len(samples) < 3:
        return None
    work = np.array([item.cells_millions * item.iterations / 1000.0 for item in samples])
    seconds = np.array([item.seconds for item in samples])
    if np.ptp(work) < 1e-9:
        # Every sample at the same settings: no slope to fit, but the mean is
        # still a much better constant than the built-in default.
        return float(np.mean(seconds)), 0.0
    design = np.column_stack([np.ones_like(work), work])
    solution, *_ = np.linalg.lstsq(design, seconds, rcond=None)
    overhead, rate = float(solution[0]), float(solution[1])
    if rate <= 0:
        return float(max(np.mean(seconds), 0.0)), 0.0
    return max(overhead, 0.0), rate


def estimate_evaluation(
    solver: str,
    mesh_resolution: int,
    iterations: int,
    refinement_level: int = 3,
    history: list[Sample] | None = None,
) -> float:
    """Predicted seconds for one solve."""
    cells = estimated_cells(solver, mesh_resolution, refinement_level)
    work = cells * max(int(iterations), 1) / 1000.0

    samples = [item for item in (history if history is not None else load_history()) if item.solver == solver]
    fitted = _fit(samples)
    if fitted is not None:
        overhead, rate = fitted
    else:
        overhead = DEFAULT_OVERHEAD.get(solver, 30.0)
        rate = DEFAULT_RATE.get(solver, 60.0)

    return float(max(overhead + rate * work, 1.0))


def estimate_scene(scene, backends: list[str] | None = None) -> dict:
    """Predicted seconds for a whole run, broken down per backend.

    Accounts for the speed-curve strategy: a sweep solves every point, while
    scaling solves once and extrapolates.
    """
    backends = list(backends if backends is not None else scene.solver.backends)
    history = load_history()

    try:
        mode, _ = scene.resolved_sweep_mode()
    except Exception:
        mode = scene.solver.sweep_mode if scene.solver.sweep_mode in {"scale", "sweep"} else "sweep"

    points = len(scene.solver.speeds())
    per_backend: dict[str, dict] = {}
    total = 0.0

    for backend in backends:
        evaluations = points if mode == "sweep" else 1
        each = estimate_evaluation(
            backend,
            scene.solver.mesh_resolution,
            scene.solver.iterations,
            scene.solver.refinement_level,
            history=history,
        )
        seconds = each * evaluations
        per_backend[backend] = {
            "evaluations": evaluations,
            "seconds_each": each,
            "seconds": seconds,
        }
        total += seconds

    return {
        "mode": mode,
        "total_seconds": total,
        "per_backend": per_backend,
        "calibrated": len(history) >= 3,
        "samples": len(history),
    }


def format_duration(seconds: float) -> str:
    """Human-facing duration: '45 s', '4 min', '2 h 10 min'."""
    seconds = max(float(seconds), 0.0)
    if seconds < 90:
        return f"{seconds:.0f} s"
    minutes = seconds / 60.0
    if minutes < 90:
        return f"{minutes:.0f} min"
    hours = int(minutes // 60)
    remainder = int(minutes % 60)
    return f"{hours} h {remainder:02d} min" if remainder else f"{hours} h"


class ProgressTracker:
    """Turns per-unit completion into a live remaining-time estimate.

    Starts from the fitted prediction and rescales it by how the run is
    actually going, so a machine that is twice as slow as the model converges
    onto a correct ETA after the first completed solve instead of insisting on
    the original guess.
    """

    def __init__(self, planned: list[tuple[str, float]]):
        # planned: (label, predicted_seconds) per unit of work, in order.
        self.planned = list(planned)
        self.total_predicted = sum(seconds for _, seconds in self.planned) or 1.0
        self.completed = 0
        self.elapsed_completed = 0.0

    @property
    def total_units(self) -> int:
        return len(self.planned)

    def complete_unit(self, seconds: float) -> None:
        self.completed = min(self.completed + 1, self.total_units)
        self.elapsed_completed += max(float(seconds), 0.0)

    def _correction(self) -> float:
        if self.completed == 0:
            return 1.0
        predicted_done = sum(seconds for _, seconds in self.planned[: self.completed]) or 1.0
        ratio = self.elapsed_completed / predicted_done
        # Clamp: one freak slow mesh should not triple the estimate.
        return float(min(max(ratio, 0.25), 4.0))

    def remaining_seconds(self) -> float:
        outstanding = sum(seconds for _, seconds in self.planned[self.completed:])
        return float(outstanding * self._correction())

    def snapshot(self) -> dict:
        return {
            "units_total": self.total_units,
            "units_done": self.completed,
            "fraction": self.completed / self.total_units if self.total_units else 0.0,
            "elapsed_seconds": self.elapsed_completed,
            "remaining_seconds": self.remaining_seconds(),
            "remaining_text": format_duration(self.remaining_seconds()),
        }
