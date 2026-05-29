from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
from scipy.optimize import lsq_linear


def _normalize(vectors: np.ndarray) -> np.ndarray:
    vectors = np.asarray(vectors, dtype=float)
    norms = np.linalg.norm(vectors, axis=-1, keepdims=True)
    norms = np.clip(norms, 1e-12, None)
    return vectors / norms


def fibonacci_sphere(count: int) -> np.ndarray:
    if count <= 0:
        raise ValueError("count must be positive")

    indices = np.arange(count, dtype=float)
    phi = np.pi * (3.0 - np.sqrt(5.0))
    y = 1.0 - 2.0 * (indices + 0.5) / count
    radius = np.sqrt(np.clip(1.0 - y * y, 0.0, 1.0))
    theta = phi * indices
    x = np.cos(theta) * radius
    z = np.sin(theta) * radius
    return np.column_stack([x, y, z])


def load_stl(path: str | Path) -> trimesh.Trimesh:
    mesh = trimesh.load_mesh(str(path), force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected a mesh from {path}")
    if mesh.is_empty:
        raise ValueError(f"Mesh loaded from {path} is empty")
    return mesh


def center_mesh(mesh: trimesh.Trimesh) -> tuple[trimesh.Trimesh, np.ndarray]:
    centered = mesh.copy()
    centroid = np.asarray(mesh.centroid, dtype=float)
    centered.apply_translation(-centroid)
    return centered, centroid


def convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    hull = mesh.convex_hull
    if hull.is_empty:
        raise ValueError("Convex hull computation failed")
    return hull


def support_values(vertices: np.ndarray, directions: np.ndarray) -> np.ndarray:
    vertices = np.asarray(vertices, dtype=float)
    directions = _normalize(directions)
    return np.max(vertices @ directions.T, axis=0)


def wendland_kernel(distances: np.ndarray) -> np.ndarray:
    scaled = np.clip(1.0 - 0.5 * np.asarray(distances, dtype=float), 0.0, None)
    return scaled**6


def wendland_kernel_gradient(direction: np.ndarray, center: np.ndarray) -> np.ndarray:
    direction = _normalize(direction)
    center = _normalize(center)
    delta = direction - center
    distance = np.linalg.norm(delta)
    if distance < 1e-12:
        return np.zeros(3, dtype=float)

    scaled = 1.0 - 0.5 * distance
    if scaled <= 0.0:
        return np.zeros(3, dtype=float)

    derivative = -3.0 * scaled**5
    return derivative * delta / distance


def basis_matrix(directions: np.ndarray, centers: np.ndarray, include_bias: bool = True) -> np.ndarray:
    directions = _normalize(directions)
    centers = _normalize(centers)
    distances = np.linalg.norm(directions[:, None, :] - centers[None, :, :], axis=-1)
    kernels = wendland_kernel(distances)
    if include_bias:
        return np.column_stack([np.ones(len(directions)), kernels])
    return kernels


@dataclass
class ConvexSupportModel:
    bias: float
    weights: np.ndarray
    centers: np.ndarray

    def support_function(self, direction: np.ndarray) -> float:
        direction = _normalize(direction)
        basis = basis_matrix(direction[None, :], self.centers, include_bias=True)[0]
        params = np.concatenate([[self.bias], self.weights])
        return float(basis @ params)

    def support_gradient(self, direction: np.ndarray) -> np.ndarray:
        direction = _normalize(direction)
        gradient = np.zeros(3, dtype=float)
        for weight, center in zip(self.weights, self.centers):
            gradient += weight * wendland_kernel_gradient(direction, center)
        gradient -= np.dot(gradient, direction) * direction
        return gradient

    def surface_point(self, direction: np.ndarray) -> np.ndarray:
        direction = _normalize(direction)
        h = self.support_function(direction)
        tangent_gradient = self.support_gradient(direction)
        return h * direction + tangent_gradient


def fit_support_model(
    sample_directions: np.ndarray,
    sample_values: np.ndarray,
    centers: np.ndarray,
    enclosure_vertices: np.ndarray | None = None,
    enclosure_margin: float = 1e-4,
) -> ConvexSupportModel:
    sample_directions = _normalize(sample_directions)
    sample_values = np.asarray(sample_values, dtype=float)
    centers = _normalize(centers)

    design = basis_matrix(sample_directions, centers, include_bias=True)
    bounds_lower = np.zeros(design.shape[1], dtype=float)
    bounds_upper = np.full(design.shape[1], np.inf, dtype=float)
    result = lsq_linear(design, sample_values, bounds=(bounds_lower, bounds_upper), lsmr_tol="auto")
    params = np.maximum(result.x, 0.0)

    if enclosure_vertices is not None and len(enclosure_vertices) > 0:
        enclosure_vertices = np.asarray(enclosure_vertices, dtype=float)
        enclosure_dirs = _normalize(enclosure_vertices)
        enclosure_basis = basis_matrix(enclosure_dirs, centers, include_bias=True)
        target = np.sum(enclosure_vertices * enclosure_dirs, axis=1)
        shortfall = target - enclosure_basis @ params
        max_shortfall = float(np.max(shortfall)) if len(shortfall) else 0.0
        if max_shortfall > 0.0:
            params[0] += max_shortfall + enclosure_margin

    return ConvexSupportModel(bias=float(params[0]), weights=params[1:], centers=centers)


def sample_convex_hull_support(mesh: trimesh.Trimesh, n_directions: int, n_centers: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    hull = convex_hull(mesh)
    centered_hull, centroid = center_mesh(hull)
    directions = fibonacci_sphere(n_directions)
    values = support_values(centered_hull.vertices, directions)
    centers = fibonacci_sphere(n_centers)
    raw_support = np.column_stack([directions, values])
    return raw_support, centers, centroid


def surface_mesh_from_model(model: ConvexSupportModel, subdivisions: int = 3) -> trimesh.Trimesh:
    if subdivisions < 0:
        raise ValueError("subdivisions must be non-negative")

    base = trimesh.creation.icosphere(subdivisions=subdivisions, radius=1.0)
    directions = _normalize(base.vertices)
    points = np.array([model.surface_point(direction) for direction in directions], dtype=float)
    surface = trimesh.Trimesh(vertices=points, faces=base.faces, process=True)
    if surface.is_empty:
        raise ValueError("Failed to generate surface mesh")
    return surface


def save_surface_function(path: str | Path, model: ConvexSupportModel) -> None:
    path = Path(path)
    weights_str = np.array2string(model.weights, separator=", ", max_line_width=120)
    centers_str = np.array2string(model.centers, separator=", ", max_line_width=120)
    content = f'''from __future__ import annotations

import numpy as np

DEFAULT_BIAS = {model.bias!r}
DEFAULT_WEIGHTS = np.array({weights_str}, dtype=float)
CENTERS = np.array({centers_str}, dtype=float)


def _normalize(vectors):
    vectors = np.asarray(vectors, dtype=float)
    norms = np.linalg.norm(vectors, axis=-1, keepdims=True)
    norms = np.clip(norms, 1e-12, None)
    return vectors / norms


def _kernel(distances):
    scaled = np.clip(1.0 - 0.5 * np.asarray(distances, dtype=float), 0.0, None)
    return scaled**6


def support_function(nx, ny, nz, bias=DEFAULT_BIAS, weights=DEFAULT_WEIGHTS, centers=CENTERS):
    direction = _normalize(np.array([nx, ny, nz], dtype=float))
    distances = np.linalg.norm(centers - direction, axis=1)
    return float(bias + np.sum(weights * _kernel(distances)))


def surface_point(theta, phi, bias=DEFAULT_BIAS, weights=DEFAULT_WEIGHTS, centers=CENTERS):
    direction = np.array([
        np.sin(theta) * np.cos(phi),
        np.cos(theta),
        np.sin(theta) * np.sin(phi),
    ], dtype=float)
    direction = _normalize(direction)
    distances = np.linalg.norm(centers - direction, axis=1)
    h = float(bias + np.sum(weights * _kernel(distances)))
    gradient = np.zeros(3, dtype=float)
    for weight, center in zip(weights, centers):
        delta = direction - center
        distance = np.linalg.norm(delta)
        if distance < 1e-12:
            continue
        scaled = 1.0 - 0.5 * distance
        if scaled <= 0.0:
            continue
        derivative = -3.0 * scaled**5
        gradient += weight * derivative * delta / distance
    gradient -= np.dot(gradient, direction) * direction
    return h * direction + gradient
'''
    path.write_text(content, encoding="utf-8")