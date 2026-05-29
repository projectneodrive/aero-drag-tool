from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import trimesh

from openfoam import openfoam_available, run_openfoam_drag


def normalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("Wind vector must be non-zero")
    return vector / norm


def projected_area(mesh: trimesh.Trimesh, wind_vector: np.ndarray) -> float:
    direction = normalize(wind_vector)
    normals = np.asarray(mesh.face_normals, dtype=float)
    areas = np.asarray(mesh.area_faces, dtype=float)
    facing = np.clip(normals @ direction, 0.0, None)
    return float(np.sum(areas * facing))


def estimate_drag(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float = 1.225,
    viscosity: float = 1.8e-5,
    drag_coefficient: float = 0.5,
    solver: str = "proxy",
    ground_offset: float | None = None,
    work_dir: str | Path | None = None,
) -> float:
    solver = solver.lower().strip()
    if solver not in {"proxy", "openfoam", "auto"}:
        raise ValueError("solver must be 'proxy', 'openfoam', or 'auto'")

    if solver == "openfoam" or (solver == "auto" and openfoam_available()):
        result = run_openfoam_drag(
            mesh,
            wind_vector,
            density=density,
            viscosity=viscosity,
            ground_offset=ground_offset,
            work_dir=work_dir,
            keep_case=False,
        )
        return float(result.drag_force)

    speed = float(np.linalg.norm(wind_vector))
    if speed < 1e-12:
        return 0.0

    area = projected_area(mesh, wind_vector)
    dynamic_pressure = 0.5 * density * speed * speed
    return float(dynamic_pressure * area * drag_coefficient)


def run_su2_drag(mesh_file: str | Path, config_file: str | Path) -> tuple[float, str]:
    executable = shutil.which("SU2_CFD")
    if executable is None:
        raise RuntimeError("SU2_CFD is not available on PATH")

    completed = subprocess.run(
        [executable, str(config_file)],
        check=True,
        capture_output=True,
        text=True,
    )
    return 0.0, completed.stdout + completed.stderr


def finite_difference_gradient(objective, parameters: np.ndarray, epsilon: float = 1e-4) -> np.ndarray:
    parameters = np.asarray(parameters, dtype=float)
    gradient = np.zeros_like(parameters)
    for index in range(len(parameters)):
        step = np.zeros_like(parameters)
        step[index] = epsilon
        value_plus = objective(parameters + step)
        value_minus = objective(parameters - step)
        gradient[index] = (value_plus - value_minus) / (2.0 * epsilon)
    return gradient