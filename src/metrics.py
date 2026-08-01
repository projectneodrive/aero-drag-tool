"""Geometry metrics and flow-regime helpers shared by every solver backend.

The important piece here is :func:`frontal_area`, which computes a *true*
silhouette area by rasterising the projected triangles. The cheap
``sum(area * max(n . d, 0))`` estimate used elsewhere in this repo is only
equal to the frontal area for convex bodies; for a concave hull it
over-counts every surface that hides behind another one, which biases both
``Cd`` and any drag force derived from it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh


# Reynolds band over which a bluff body transitions from sub-critical to
# fully turbulent. Cd is strongly Re-dependent inside it and comparatively
# flat outside, so a speed range that straddles the band cannot be
# extrapolated from a single CFD run.
CRITICAL_REYNOLDS = (2.0e5, 1.0e6)

# Above this Re_max / Re_min ratio we recommend a sweep even when the whole
# range sits in one regime.
MAX_SAFE_REYNOLDS_RATIO = 3.0


def normalize(vector) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("Vector must be non-zero")
    return vector / norm


def perpendicular_basis(direction) -> tuple[np.ndarray, np.ndarray]:
    """Return two unit vectors spanning the plane normal to ``direction``."""
    d = normalize(direction)
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(reference, d))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    u = normalize(np.cross(reference, d))
    v = np.cross(d, u)
    return u, v


def convex_projected_area(mesh: trimesh.Trimesh, direction) -> float:
    """Cheap projected area: exact for convex bodies, an over-estimate otherwise."""
    d = normalize(direction)
    normals = np.asarray(mesh.face_normals, dtype=float)
    areas = np.asarray(mesh.area_faces, dtype=float)
    return float(np.sum(areas * np.clip(normals @ d, 0.0, None)))


def frontal_area(mesh: trimesh.Trimesh, direction, resolution: int = 512) -> float:
    """True silhouette area of ``mesh`` seen from along ``direction``.

    Projects every relevant triangle onto the plane normal to the flow and
    rasterises the union, so overlapping surfaces are counted once. Accuracy
    is roughly one part in ``resolution``; 512 keeps the error near 0.2% for
    typical hulls while staying fast enough to call on every UI edit.
    """
    if resolution < 16:
        raise ValueError("resolution must be at least 16")

    d = normalize(direction)
    u, v = perpendicular_basis(d)

    faces = np.asarray(mesh.faces)
    if len(faces) == 0:
        return 0.0

    # For a closed surface the windward faces alone already project onto the
    # full silhouette, so we can halve the rasterisation work. A mesh with
    # holes gets every face instead, since we cannot rely on that argument.
    dots = np.asarray(mesh.face_normals, dtype=float) @ d
    if mesh.is_watertight:
        selected = dots < 0.0
        if not selected.any():
            selected = dots > 0.0
    else:
        selected = np.ones(len(faces), dtype=bool)
    if not selected.any():
        return 0.0

    triangles = np.asarray(mesh.vertices, dtype=float)[faces[selected]]
    pu = triangles @ u
    pv = triangles @ v

    u_min, u_max = float(pu.min()), float(pu.max())
    v_min, v_max = float(pv.min()), float(pv.max())
    span = max(u_max - u_min, v_max - v_min)
    if span < 1e-12:
        return 0.0

    cell = span / resolution
    n_u = int(np.ceil((u_max - u_min) / cell)) + 1
    n_v = int(np.ceil((v_max - v_min) / cell)) + 1
    mask = np.zeros((n_u, n_v), dtype=bool)

    # Pixel centres, so a pixel counts as covered when its centre is inside.
    su = (pu - u_min) / cell
    sv = (pv - v_min) / cell

    for index in range(len(su)):
        a_u, b_u, c_u = su[index]
        a_v, b_v, c_v = sv[index]

        # Signed area in pixel units; skip slivers that carry no area.
        twice_area = (b_u - a_u) * (c_v - a_v) - (c_u - a_u) * (b_v - a_v)
        if abs(twice_area) < 1e-12:
            continue

        i0 = max(int(np.floor(min(a_u, b_u, c_u) - 0.5)), 0)
        i1 = min(int(np.ceil(max(a_u, b_u, c_u) + 0.5)), n_u - 1)
        j0 = max(int(np.floor(min(a_v, b_v, c_v) - 0.5)), 0)
        j1 = min(int(np.ceil(max(a_v, b_v, c_v) + 0.5)), n_v - 1)
        if i1 < i0 or j1 < j0:
            continue

        # Always claim the pixel under the centroid so triangles finer than
        # one cell still contribute instead of silently vanishing.
        mask[
            min(max(int((a_u + b_u + c_u) / 3.0), 0), n_u - 1),
            min(max(int((a_v + b_v + c_v) / 3.0), 0), n_v - 1),
        ] = True

        grid_u = np.arange(i0, i1 + 1, dtype=float)[:, None] + 0.5
        grid_v = np.arange(j0, j1 + 1, dtype=float)[None, :] + 0.5

        w0 = ((b_u - a_u) * (grid_v - a_v) - (grid_u - a_u) * (b_v - a_v)) / twice_area
        w1 = ((c_u - b_u) * (grid_v - b_v) - (grid_u - b_u) * (c_v - b_v)) / twice_area
        w2 = ((a_u - c_u) * (grid_v - c_v) - (grid_u - c_u) * (a_v - c_v)) / twice_area
        inside = (w0 >= 0.0) & (w1 >= 0.0) & (w2 >= 0.0)
        if inside.any():
            mask[i0 : i1 + 1, j0 : j1 + 1] |= inside

    return float(mask.sum()) * cell * cell


def streamwise_length(mesh: trimesh.Trimesh, direction) -> float:
    """Body extent measured along the flow: the Reynolds reference length."""
    d = normalize(direction)
    projected = np.asarray(mesh.vertices, dtype=float) @ d
    return float(projected.max() - projected.min())


def flow_domain(
    mesh: trimesh.Trimesh,
    ground: bool,
    padding_factor: float = 4.0,
    min_padding: float = 4.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Bounding box of the fluid domain around ``mesh``.

    Both solver backends call this so they solve the same problem: a
    difference in domain size shows up directly in blockage and would make
    the cross-check meaningless. With a road the box bottom is pinned to
    z = 0 so the ground plane is a domain face.
    """
    bounds = np.asarray(mesh.bounds, dtype=float)
    extents = np.asarray(mesh.extents, dtype=float)
    padding = np.maximum(extents * padding_factor, min_padding)
    domain_min = bounds[0] - padding
    domain_max = bounds[1] + padding
    if ground:
        domain_min[2] = 0.0
        domain_max[2] = max(domain_max[2], bounds[1][2] + padding[2])
    return domain_min, domain_max


def reynolds(speed: float, length: float, density: float, viscosity: float) -> float:
    if viscosity <= 0.0:
        raise ValueError("viscosity must be positive")
    return float(density * abs(speed) * length / viscosity)


@dataclass
class GeometryMetrics:
    frontal_area: float
    wetted_area: float
    volume: float
    streamwise_length: float
    bounds_min: list[float]
    bounds_max: list[float]
    extents: list[float]
    watertight: bool
    triangle_count: int
    convex_projected_area: float

    def to_dict(self) -> dict:
        return {
            "frontal_area": self.frontal_area,
            "wetted_area": self.wetted_area,
            "volume": self.volume,
            "streamwise_length": self.streamwise_length,
            "bounds_min": self.bounds_min,
            "bounds_max": self.bounds_max,
            "extents": self.extents,
            "watertight": self.watertight,
            "triangle_count": self.triangle_count,
            "convex_projected_area": self.convex_projected_area,
        }


def geometry_metrics(mesh: trimesh.Trimesh, direction, resolution: int = 512) -> GeometryMetrics:
    bounds = np.asarray(mesh.bounds, dtype=float)
    return GeometryMetrics(
        frontal_area=frontal_area(mesh, direction, resolution=resolution),
        wetted_area=float(mesh.area),
        volume=float(mesh.volume) if mesh.is_watertight else 0.0,
        streamwise_length=streamwise_length(mesh, direction),
        bounds_min=bounds[0].tolist(),
        bounds_max=bounds[1].tolist(),
        extents=np.asarray(mesh.extents, dtype=float).tolist(),
        watertight=bool(mesh.is_watertight),
        triangle_count=int(len(mesh.faces)),
        convex_projected_area=convex_projected_area(mesh, direction),
    )


@dataclass
class ReynoldsAdvice:
    """Whether a speed range can be extrapolated from one CFD run."""

    mode: str  # "scale" or "sweep"
    reynolds_min: float
    reynolds_max: float
    ratio: float
    reference_length: float
    crosses_critical_band: bool
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "mode": self.mode,
            "reynolds_min": self.reynolds_min,
            "reynolds_max": self.reynolds_max,
            "ratio": self.ratio,
            "reference_length": self.reference_length,
            "crosses_critical_band": self.crosses_critical_band,
            "warnings": list(self.warnings),
        }


def advise_speed_mode(
    speed_min: float,
    speed_max: float,
    reference_length: float,
    density: float,
    viscosity: float,
) -> ReynoldsAdvice:
    """Recommend single-run scaling or a full sweep for a speed range.

    Drag scales as ``0.5 * rho * V^2 * A * Cd`` only while ``Cd`` itself holds
    still. ``Cd`` is a function of Reynolds number, so the question is whether
    Re moves enough across the requested speed range to matter.
    """
    speed_low = min(abs(speed_min), abs(speed_max))
    speed_high = max(abs(speed_min), abs(speed_max))

    re_min = reynolds(speed_low, reference_length, density, viscosity)
    re_max = reynolds(speed_high, reference_length, density, viscosity)
    ratio = re_max / re_min if re_min > 0.0 else float("inf")

    band_low, band_high = CRITICAL_REYNOLDS
    crosses = re_max > band_low and re_min < band_high

    warnings: list[str] = []
    mode = "scale"

    if crosses:
        mode = "sweep"
        warnings.append(
            f"Reynolds number spans {re_min:.3g} to {re_max:.3g} over "
            f"{speed_low:.4g}-{speed_high:.4g} m/s, which overlaps the transitional band "
            f"({band_low:.0e}-{band_high:.0e}) where Cd changes with speed. "
            "A single run scaled as V^2 will misreport the ends of the curve; "
            "a sweep runs the solver at each speed instead."
        )
    elif ratio > MAX_SAFE_REYNOLDS_RATIO:
        mode = "sweep"
        warnings.append(
            f"Reynolds number varies by {ratio:.1f}x across the speed range "
            f"({re_min:.3g} to {re_max:.3g}). Cd is unlikely to stay constant over "
            "that span, so single-run scaling is approximate."
        )
    else:
        warnings.append(
            f"Reynolds number stays within {ratio:.1f}x ({re_min:.3g} to {re_max:.3g}) "
            "and clear of the transitional band, so Cd can be treated as constant and "
            "the curve scaled from one run."
        )

    return ReynoldsAdvice(
        mode=mode,
        reynolds_min=re_min,
        reynolds_max=re_max,
        ratio=ratio,
        reference_length=reference_length,
        crosses_critical_band=crosses,
        warnings=warnings,
    )
