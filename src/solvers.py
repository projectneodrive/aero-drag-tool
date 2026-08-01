"""Solver registry and the scale-versus-sweep orchestration.

Every backend is handed the same placed mesh, the same flow domain and the
same reference area, so the coefficients they report can be compared directly
instead of being apples and oranges.

A run produces one :class:`~scene.SolverRun` per backend. Depending on the
Reynolds analysis it either solves once and scales the speed curve as V^2, or
solves at every speed on the curve.
"""

from __future__ import annotations

import shutil
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np
import trimesh

import estimates
from metrics import geometry_metrics, reynolds
from scene import ResultSet, Scene, SolverRun, SpeedPoint


ProgressCallback = Callable[[dict], None]


@dataclass
class SolverInfo:
    name: str
    label: str
    available: bool
    detail: str
    kind: str  # "analytical" or "cfd"

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "label": self.label,
            "available": self.available,
            "detail": self.detail,
            "kind": self.kind,
        }


def _openfoam_info(processes: int | None = None) -> SolverInfo:
    try:
        from openfoam import detect_openfoam

        runner = detect_openfoam(processes)
        detail = (
            runner.describe()
            if runner is not None
            else "OpenFOAM 13 not found natively, in Docker or in WSL. Build the image with docker/build.sh."
        )
        available = runner is not None
    except Exception as error:  # pragma: no cover - import/env dependent
        available, detail = False, f"Unavailable: {error}"
    return SolverInfo(
        name="openfoam",
        label="OpenFOAM",
        available=available,
        detail=detail,
        kind="cfd",
    )


def _su2_info(processes: int | None = None) -> SolverInfo:
    try:
        from su2 import detect_su2, gmsh_available

        runner = detect_su2(processes)
        if runner is None:
            detail = "SU2_CFD not found natively, in Docker or in WSL. Build the image with docker/build.sh."
        elif not gmsh_available():
            detail = "SU2_CFD found but the gmsh module is missing (pip install gmsh)."
        else:
            detail = runner.describe()
        available = runner is not None and gmsh_available()
    except Exception as error:  # pragma: no cover - import/env dependent
        available, detail = False, f"Unavailable: {error}"
    return SolverInfo(name="su2", label="SU2", available=available, detail=detail, kind="cfd")


SOLVER_NAMES = ("openfoam", "su2")


def available_solvers(processes: int | None = None) -> list[SolverInfo]:
    return [_openfoam_info(processes), _su2_info(processes)]


def solver_info(name: str) -> SolverInfo:
    for info in available_solvers():
        if info.name == name:
            return info
    raise ValueError(f"Unknown solver {name!r}")


# --------------------------------------------------------------------------
# Individual solves
# --------------------------------------------------------------------------


@dataclass
class PointSolution:
    drag_coefficient: float
    lift_coefficient: float | None
    reference_area: float
    log_excerpt: str = ""
    settings: dict | None = None
    converged: bool = True


def _solve_openfoam(
    scene: Scene,
    mesh: trimesh.Trimesh,
    speed: float,
    reference_area: float,
    work_dir: Path | None = None,
    keep_case: bool = False,
) -> PointSolution:
    from openfoam import run_openfoam_drag

    result = run_openfoam_drag(
        mesh,
        scene.wind_vector(speed),
        density=scene.fluid.density,
        viscosity=scene.fluid.viscosity,
        ground_offset=scene.ground_offset(),
        work_dir=work_dir,
        keep_case=keep_case,
        turbulent=scene.solver.turbulence != "laminar",
        iterations=scene.solver.iterations,
        mesh_resolution=scene.solver.mesh_resolution,
        refinement_level=scene.solver.refinement_level,
        moving_ground=scene.road.enabled and scene.road.moving,
        n_processors=scene.solver.processes,
        reference_area=reference_area,
    )
    return PointSolution(
        drag_coefficient=result.drag_coefficient,
        lift_coefficient=result.lift_coefficient,
        reference_area=result.reference_area,
        log_excerpt=result.log_excerpt,
        settings=result.settings,
        converged=result.converged,
    )


def _solve_su2(
    scene: Scene,
    mesh: trimesh.Trimesh,
    speed: float,
    reference_area: float,
    work_dir: Path | None = None,
    keep_case: bool = False,
) -> PointSolution:
    from su2 import run_su2_drag

    result = run_su2_drag(
        mesh,
        scene.wind_vector(speed),
        density=scene.fluid.density,
        viscosity=scene.fluid.viscosity,
        ground_offset=scene.ground_offset(),
        work_dir=work_dir,
        keep_case=keep_case,
        turbulent=scene.solver.turbulence != "laminar",
        iterations=scene.solver.iterations,
        surface_cells=max(scene.solver.mesh_resolution // 2, 8),
        refinement_level=scene.solver.refinement_level,
        processes=scene.solver.processes,
        reference_area=reference_area,
    )
    return PointSolution(
        drag_coefficient=result.drag_coefficient,
        lift_coefficient=result.lift_coefficient,
        reference_area=result.reference_area,
        log_excerpt=result.log_excerpt,
        settings=result.settings,
        converged=result.converged,
    )


_SOLVE_FUNCTIONS = {
    "openfoam": _solve_openfoam,
    "su2": _solve_su2,
}


# --------------------------------------------------------------------------
# Orchestration
# --------------------------------------------------------------------------


def _make_point(
    scene: Scene,
    speed: float,
    solution: PointSolution,
    reference_length: float,
    source: str,
) -> SpeedPoint:
    dynamic_pressure = 0.5 * scene.fluid.density * speed * speed
    area = solution.reference_area
    return SpeedPoint(
        speed=float(speed),
        drag_force=float(dynamic_pressure * area * solution.drag_coefficient),
        drag_coefficient=float(solution.drag_coefficient),
        frontal_area=float(area),
        reynolds=reynolds(speed, reference_length, scene.fluid.density, scene.fluid.viscosity),
        lift_force=(
            None
            if solution.lift_coefficient is None
            else float(dynamic_pressure * area * solution.lift_coefficient)
        ),
        source=source,
    )


def run_backend(
    scene: Scene,
    backend: str,
    mesh: trimesh.Trimesh,
    reference_area: float,
    reference_length: float,
    mode: str,
    progress: ProgressCallback | None = None,
    keep_cases: bool = False,
    case_root: Path | None = None,
    tracker: "estimates.ProgressTracker | None" = None,
) -> SolverRun:
    """Run one backend over the scene's speed range."""
    solve = _SOLVE_FUNCTIONS.get(backend)
    if solve is None:
        return SolverRun(solver=backend, status="failed", message=f"Unknown solver {backend!r}")

    speeds = scene.solver.speeds()

    info = solver_info(backend)
    if not info.available:
        if tracker is not None:
            tracker.complete_unit(0.0)  # keep the progress bar moving
        return SolverRun(solver=backend, status="unavailable", message=info.detail)
    effective_mode = mode
    targets = speeds if effective_mode == "sweep" else [float(scene.solver.reference_speed)]

    started = time.time()
    points: list[SpeedPoint] = []
    solutions: list[tuple[float, PointSolution]] = []

    def emit(event: dict) -> None:
        if progress is not None:
            payload = {"solver": backend, **event}
            if tracker is not None:
                payload["progress"] = tracker.snapshot()
            progress(payload)

    for index, speed in enumerate(targets):
        remaining = ""
        if tracker is not None:
            remaining = f" -- about {estimates.format_duration(tracker.remaining_seconds())} left"
        emit(
            {
                "phase": "solving",
                "index": index,
                "total": len(targets),
                "speed": speed,
                "message": f"{info.label}: solving at {speed:.4g} m/s "
                f"({index + 1}/{len(targets)}){remaining}",
            }
        )
        work_dir = None
        if case_root is not None:
            work_dir = Path(case_root) / f"{backend}_{index:02d}"
        unit_started = time.time()
        try:
            solution = solve(
                scene,
                mesh,
                speed,
                reference_area,
                work_dir=work_dir,
                keep_case=keep_cases,
            )
        except Exception as error:
            if tracker is not None:
                # Still consume the remaining units, or the bar sticks forever.
                for _ in range(len(targets) - index):
                    tracker.complete_unit(time.time() - unit_started)
            return SolverRun(
                solver=backend,
                status="failed",
                mode=effective_mode,
                points=points,
                wall_time_s=time.time() - started,
                message=f"{type(error).__name__}: {error}",
                log_excerpt="".join(traceback.format_exc()).strip()[-4000:],
            )
        elapsed = time.time() - unit_started
        if tracker is not None:
            tracker.complete_unit(elapsed)
        # Feed the timing back so the next estimate is fitted to this machine
        # rather than to the numbers shipped with the tool.
        estimates.record(
            backend,
            estimates.estimated_cells(backend, scene.solver.mesh_resolution, scene.solver.refinement_level),
            scene.solver.iterations,
            elapsed,
        )

        solutions.append((speed, solution))
        points.append(_make_point(scene, speed, solution, reference_length, "solved"))

    log_excerpt = solutions[-1][1].log_excerpt if solutions else ""
    settings = solutions[-1][1].settings or {} if solutions else {}
    converged = all(solution.converged for _, solution in solutions)

    if effective_mode == "scale" and solutions:
        # One solve gives Cd; the rest of the curve follows from 0.5 rho V^2 A Cd.
        _, reference_solution = solutions[0]
        reference_speed = solutions[0][0]
        scaled: list[SpeedPoint] = []
        for speed in speeds:
            source = "solved" if abs(speed - reference_speed) < 1e-9 else "scaled"
            scaled.append(_make_point(scene, speed, reference_solution, reference_length, source))
        if not any(abs(point.speed - reference_speed) < 1e-9 for point in scaled):
            scaled.append(points[0])
        points = sorted(scaled, key=lambda point: point.speed)

    message = ""
    if not converged:
        message = (
            "The coefficient was still oscillating at the end of the run; "
            "treat it as approximate and raise the iteration count."
        )

    return SolverRun(
        solver=backend,
        status="ok",
        mode=effective_mode,
        points=points,
        wall_time_s=time.time() - started,
        message=message,
        log_excerpt=log_excerpt,
        settings=settings,
    )


def run_scene(
    scene: Scene,
    backends: Iterable[str] | None = None,
    progress: ProgressCallback | None = None,
    keep_cases: bool = False,
    case_root: str | Path | None = None,
) -> ResultSet:
    """Compute a scene with every requested backend and return the results."""
    backend_list = list(backends) if backends is not None else list(scene.solver.backends)
    if not backend_list:
        raise ValueError("No solver backends requested")

    def emit(event: dict) -> None:
        if progress is not None:
            progress(event)

    emit({"phase": "geometry", "message": "Placing the hull and measuring the frontal area"})
    mesh = scene.placed_mesh()
    metrics = geometry_metrics(mesh, scene.wind.direction())
    reference_area = metrics.frontal_area
    reference_length = metrics.streamwise_length

    mode, advice = scene.resolved_sweep_mode(reference_length)
    warnings = list(advice.warnings)
    if scene.solver.sweep_mode in {"scale", "sweep"} and scene.solver.sweep_mode != advice.mode:
        warnings.append(
            f"Speed handling was forced to '{scene.solver.sweep_mode}' but the Reynolds "
            f"analysis recommends '{advice.mode}'."
        )
    if not metrics.watertight:
        warnings.append(
            "The STL is not watertight. Meshing may fail and the frontal area is computed "
            "from every face rather than the windward side only."
        )

    # Plan the run before starting it, so the user is told what they are in
    # for rather than watching an indeterminate spinner.
    forecast = estimates.estimate_scene(scene, backend_list)
    plan: list[tuple[str, float]] = []
    for backend in backend_list:
        entry = forecast["per_backend"].get(backend, {})
        evaluations = int(entry.get("evaluations") or 1)
        each = float(entry.get("seconds_each") or 1.0)
        plan.extend((backend, each) for _ in range(evaluations))
    tracker = estimates.ProgressTracker(plan)

    emit(
        {
            "phase": "plan",
            "message": f"Speed handling: {mode}. Estimated "
            f"{estimates.format_duration(forecast['total_seconds'])} total.",
            "mode": mode,
            "reynolds": advice.to_dict(),
            "geometry": metrics.to_dict(),
            "estimate": forecast,
            "progress": tracker.snapshot(),
        }
    )

    temporary_root: Path | None = None
    if case_root is None and keep_cases:
        temporary_root = Path(tempfile.mkdtemp(prefix="aero_cases_"))
        case_root = temporary_root

    runs: list[SolverRun] = []
    try:
        for backend in backend_list:
            run = run_backend(
                scene,
                backend,
                mesh,
                reference_area,
                reference_length,
                mode,
                progress=progress,
                keep_cases=keep_cases,
                case_root=Path(case_root) if case_root else None,
                tracker=tracker,
            )
            runs.append(run)
            emit(
                {
                    "phase": "backend-done",
                    "solver": backend,
                    "status": run.status,
                    "message": f"{backend}: {run.status}"
                    + (f" ({run.message})" if run.message else ""),
                    "progress": tracker.snapshot(),
                }
            )
    finally:
        if temporary_root is not None and not keep_cases:
            shutil.rmtree(temporary_root, ignore_errors=True)

    warnings.extend(_comparison_warnings(runs))

    return ResultSet(
        geometry=metrics.to_dict(),
        reynolds=advice.to_dict(),
        runs=runs,
        warnings=warnings,
    )


def _comparison_warnings(runs: list[SolverRun]) -> list[str]:
    """Flag disagreement between the solvers."""
    cfd = {run.solver: run for run in runs if run.status == "ok" and run.points}
    if len(cfd) < 2:
        return []

    coefficients = {}
    for name, run in cfd.items():
        point = run.reference_point()
        if point is not None:
            coefficients[name] = point.drag_coefficient

    if len(coefficients) < 2:
        return []

    values = list(coefficients.values())
    mean = float(np.mean(values))
    if abs(mean) < 1e-9:
        return []
    deviation = (max(values) - min(values)) / abs(mean)
    detail = ", ".join(f"{name} Cd={value:.4g}" for name, value in coefficients.items())

    if deviation > 0.25:
        return [
            f"The solvers disagree by {deviation * 100:.0f}% ({detail}). "
            "Check mesh resolution and iteration count on both before trusting either."
        ]
    if deviation > 0.10:
        return [f"The solvers differ by {deviation * 100:.0f}% ({detail})."]
    return [f"The solvers agree within {deviation * 100:.0f}% ({detail})."]
