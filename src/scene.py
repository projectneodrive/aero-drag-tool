"""Scene file format for the aero drag tool.

A scene is a single self-contained JSON document holding the STL, how it is
placed relative to the road, the wind, the fluid properties and the solver
settings. The same document carries the results once it has been computed,
so an uncomputed scene and a computed one are the same kind of file and load
through the same path.

    scene = Scene.from_stl_file("hull.stl")
    scene.save("case.aero.json")           # uncomputed, ready to edit or ship
    ...
    scene.results = ResultSet(...)
    scene.save("case.aero.json")           # same file, now with data

Geometry is embedded as base64 binary STL so a scene can be copied to another
machine, computed there and copied back without dragging loose files along.
"""

from __future__ import annotations

import base64
import copy
import io
import json
import math
import platform
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import trimesh

from metrics import GeometryMetrics, ReynoldsAdvice, advise_speed_mode, geometry_metrics


FORMAT_NAME = "aero-drag-scene"
# Version 2 added the payload and fairing blocks and dropped the analytical
# backend. Older documents are rejected rather than migrated -- this is a tool
# in development, not a product with a compatibility promise.
FORMAT_VERSION = 2

DEFAULT_DENSITY = 1.225
DEFAULT_VISCOSITY = 1.8e-5

# The solver backends this build understands.
KNOWN_BACKENDS = ("openfoam", "su2")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    return float(value)


def _as_optional_int(value: Any) -> int | None:
    """A positive int, or None for "unset". Zero and junk both read as unset."""
    if value is None or value == "":
        return None
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None


def _as_optional_float(value: Any) -> float | None:
    """A float, or None for "unset". Junk reads as unset; zero is a real value."""
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass
class Orientation:
    """Hull attitude in degrees, applied about the mesh centroid as Rz*Ry*Rx."""

    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0

    def matrix(self) -> np.ndarray:
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        roll = math.radians(self.roll_deg)

        cz, sz = math.cos(yaw), math.sin(yaw)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cx, sx = math.cos(roll), math.sin(roll)

        rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
        return rz @ ry @ rx

    def to_dict(self) -> dict:
        return {"yaw_deg": self.yaw_deg, "pitch_deg": self.pitch_deg, "roll_deg": self.roll_deg}

    @classmethod
    def from_dict(cls, data: dict | None) -> "Orientation":
        data = data or {}
        return cls(
            yaw_deg=_as_float(data.get("yaw_deg"), 0.0),
            pitch_deg=_as_float(data.get("pitch_deg"), 0.0),
            roll_deg=_as_float(data.get("roll_deg"), 0.0),
        )


@dataclass
class Wind:
    """Free-stream wind in spherical form.

    ``azimuth_deg`` is measured in the ground plane from +X toward +Y and
    ``elevation_deg`` upward from that plane, so 0/0 blows along +X.
    """

    speed: float = 15.0
    azimuth_deg: float = 0.0
    elevation_deg: float = 0.0

    def vector(self) -> np.ndarray:
        azimuth = math.radians(self.azimuth_deg)
        elevation = math.radians(self.elevation_deg)
        horizontal = math.cos(elevation)
        return np.array(
            [
                self.speed * horizontal * math.cos(azimuth),
                self.speed * horizontal * math.sin(azimuth),
                self.speed * math.sin(elevation),
            ],
            dtype=float,
        )

    def direction(self) -> np.ndarray:
        vector = self.vector()
        norm = float(np.linalg.norm(vector))
        if norm < 1e-12:
            return np.array([1.0, 0.0, 0.0])
        return vector / norm

    def to_dict(self) -> dict:
        return {
            "speed": self.speed,
            "azimuth_deg": self.azimuth_deg,
            "elevation_deg": self.elevation_deg,
            "vector": self.vector().tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "Wind":
        data = data or {}
        # A raw vector wins if given, so scenes written by scripts can skip the angles.
        if data.get("vector") is not None and data.get("speed") is None:
            vector = np.asarray(data["vector"], dtype=float)
            speed = float(np.linalg.norm(vector))
            if speed < 1e-12:
                return cls(speed=0.0)
            return cls(
                speed=speed,
                azimuth_deg=math.degrees(math.atan2(vector[1], vector[0])),
                elevation_deg=math.degrees(math.asin(np.clip(vector[2] / speed, -1.0, 1.0))),
            )
        return cls(
            speed=_as_float(data.get("speed"), 15.0),
            azimuth_deg=_as_float(data.get("azimuth_deg"), 0.0),
            elevation_deg=_as_float(data.get("elevation_deg"), 0.0),
        )


@dataclass
class Road:
    """Ground plane at z = 0 with the hull ``ride_height`` above it.

    The solve sits in the vehicle's frame: the body is held still, the air
    arrives at the wind speed, and a road the vehicle is driving on has to
    slide underneath it. ``moving`` is that distinction -- a stationary road is
    a wind-tunnel floor, which grows a boundary layer no real road has.

    ``speed`` is the vehicle's speed over the ground. Left unset it tracks the
    wind, which is the still-air case: park the air, and the speed the vehicle
    feels is the speed it is doing. Pinning it is the atmospheric-wind case,
    where the two differ -- 25 m/s over the ground into a 5 m/s headwind is a
    30 m/s wind over a 25 m/s road, and only the road knows the difference.
    """

    enabled: bool = True
    ride_height: float = 0.15
    moving: bool = False  # False: stationary no-slip wall. True: road sliding underneath.
    # None means "whatever the air is doing", so a speed sweep stays a sweep of
    # the vehicle rather than of the wind it is driving into.
    speed: float | None = None

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "ride_height": self.ride_height,
            "moving": self.moving,
            "speed": self.speed,
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "Road":
        data = data or {}
        return cls(
            enabled=bool(data.get("enabled", True)),
            ride_height=_as_float(data.get("ride_height"), 0.15),
            moving=bool(data.get("moving", False)),
            speed=_as_optional_float(data.get("speed")),
        )

    def ground_speed(self, wind_vector: np.ndarray) -> float:
        """How fast the road runs, in m/s, under this free stream.

        Unset tracks the *horizontal* part of the wind: a vehicle driving on
        flat ground cannot generate a vertical relative wind, so any elevation
        in the wind is air movement the vehicle is not responsible for.
        """
        if self.speed is not None:
            return float(self.speed)
        wind = np.asarray(wind_vector, dtype=float)
        return float(np.hypot(wind[0], wind[1]))

    def velocity(self, wind_vector: np.ndarray) -> np.ndarray:
        """Velocity of the road surface as the solvers must see it.

        The road runs downstream along the wind's ground heading -- which is
        what a road aligned with the direction of travel means once the frame
        is pinned to the vehicle. Zero whenever there is nothing to move: no
        road, a stationary one, or a wind with no heading to align with.
        """
        wind = np.asarray(wind_vector, dtype=float)
        if not self.enabled or not self.moving:
            return np.zeros(3)
        heading = np.array([wind[0], wind[1], 0.0], dtype=float)
        norm = float(np.linalg.norm(heading))
        if norm < 1e-12:
            # A wind straight up or down leaves the road no direction to run in.
            return np.zeros(3)
        return heading / norm * self.ground_speed(wind)


@dataclass
class Fluid:
    density: float = DEFAULT_DENSITY
    viscosity: float = DEFAULT_VISCOSITY

    @property
    def kinematic_viscosity(self) -> float:
        return self.viscosity / self.density

    def to_dict(self) -> dict:
        return {"density": self.density, "viscosity": self.viscosity}

    @classmethod
    def from_dict(cls, data: dict | None) -> "Fluid":
        data = data or {}
        return cls(
            density=_as_float(data.get("density"), DEFAULT_DENSITY),
            viscosity=_as_float(data.get("viscosity"), DEFAULT_VISCOSITY),
        )


# Presets so a user who does not want to think about mesh resolution does not
# have to. Screening is for ranking many candidates against each other, where
# consistency matters more than absolute accuracy; accurate is for the winner.
QUALITY_PRESETS: dict[str, dict] = {
    "screening": {
        "iterations": 150,
        "mesh_resolution": 26,
        "refinement_level": 2,
        "speed_points": 3,
        "sweep_mode": "scale",
    },
    "balanced": {
        "iterations": 400,
        "mesh_resolution": 40,
        "refinement_level": 3,
        "speed_points": 5,
        "sweep_mode": "auto",
    },
    "accurate": {
        "iterations": 1000,
        "mesh_resolution": 60,
        "refinement_level": 4,
        "speed_points": 7,
        "sweep_mode": "auto",
    },
}


@dataclass
class SolverSettings:
    """Shared knobs so both backends can be told to do the same thing."""

    backends: list[str] = field(default_factory=lambda: ["openfoam"])
    reference_speed: float = 15.0
    speed_min: float = 5.0
    speed_max: float = 20.0
    speed_points: int = 7
    # "auto" lets the Reynolds analysis pick; "scale" and "sweep" force it.
    sweep_mode: str = "auto"
    turbulence: str = "kOmegaSST"  # or "laminar"
    iterations: int = 400
    mesh_resolution: int = 40  # background cells along the longest domain axis
    refinement_level: int = 3  # surface refinement for snappyHexMesh / gmsh sizing
    quality: str = "balanced"
    # MPI ranks for the solve. None means "decide from this machine", which is
    # 80% of the visible cores -- see execution.default_processes. It is left
    # unset rather than baked in so a scene stays portable between machines
    # with different core counts.
    processes: int | None = None

    def apply_preset(self, name: str) -> "SolverSettings":
        preset = QUALITY_PRESETS.get(name)
        if preset is None:
            raise ValueError(f"Unknown quality preset {name!r}")
        for key, value in preset.items():
            setattr(self, key, value)
        self.quality = name
        return self

    def speeds(self) -> list[float]:
        points = max(int(self.speed_points), 2)
        low = min(self.speed_min, self.speed_max)
        high = max(self.speed_min, self.speed_max)
        if high - low < 1e-9:
            return [low]
        return [float(value) for value in np.linspace(low, high, points)]

    def to_dict(self) -> dict:
        return {
            "backends": list(self.backends),
            "reference_speed": self.reference_speed,
            "speed_min": self.speed_min,
            "speed_max": self.speed_max,
            "speed_points": self.speed_points,
            "sweep_mode": self.sweep_mode,
            "turbulence": self.turbulence,
            "iterations": self.iterations,
            "mesh_resolution": self.mesh_resolution,
            "refinement_level": self.refinement_level,
            "quality": self.quality,
            "processes": self.processes,
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "SolverSettings":
        data = data or {}
        defaults = cls()
        backends = [str(name) for name in (data.get("backends") or defaults.backends)]
        unknown = [name for name in backends if name not in KNOWN_BACKENDS]
        if unknown:
            raise ValueError(
                f"Unknown solver backend(s) {unknown}; this build has {list(KNOWN_BACKENDS)}"
            )
        return cls(
            backends=backends,
            reference_speed=_as_float(data.get("reference_speed"), defaults.reference_speed),
            speed_min=_as_float(data.get("speed_min"), defaults.speed_min),
            speed_max=_as_float(data.get("speed_max"), defaults.speed_max),
            speed_points=int(data.get("speed_points") or defaults.speed_points),
            sweep_mode=str(data.get("sweep_mode") or defaults.sweep_mode),
            turbulence=str(data.get("turbulence") or defaults.turbulence),
            iterations=int(data.get("iterations") or defaults.iterations),
            mesh_resolution=int(data.get("mesh_resolution") or defaults.mesh_resolution),
            refinement_level=int(data.get("refinement_level") or defaults.refinement_level),
            quality=str(data.get("quality") or defaults.quality),
            # Absent and null both mean "decide from this machine"; only an
            # explicit number pins the rank count into the scene.
            processes=_as_optional_int(data.get("processes")),
        )


@dataclass
class PackagingSettings:
    """Inputs to fairing generation, so they live with the scene and persist."""

    clearance: float = 0.03
    anisotropy: float = 3.0
    resolution: int = 128
    # The taper-bounded envelope: off means the raw closing skin, which merely
    # encloses. The tail limit is what keeps the afterbody attached; the nose
    # is forgiving and mostly sets how blunt the front may be.
    streamline: bool = True
    nose_angle_deg: float = 45.0
    tail_angle_deg: float = 12.0
    # How the tapers meet the payload's own section. "faceted" is the minimal
    # envelope: flat panels, and a crease at each shoulder because the section
    # is held to the last slice and shrinking at full rate the next. Minimum
    # volume was never the objective though -- Cd.A is, and the frontal area
    # it normalises on is identical either way -- so "blended" rounds those
    # shoulders instead, buying tangent continuity with wetted area and
    # length. Both are worth flying; the loop can search between them.
    envelope_profile: str = "faceted"  # or "blended"
    # Streamwise length of each blended shoulder, as a fraction of the
    # payload's cross-flow half-width. Ignored when faceted.
    shoulder_blend: float = 0.5
    # "heuristic" builds the envelope at the angles above and stops. "cfd" then
    # puts the solver in the loop: a budget of screening solves walks the tail
    # and nose angles to whatever this payload at these conditions actually
    # wants. Slow -- each step is a real CFD solve -- and worth it exactly when
    # the rule-of-thumb angles are the thing you doubt.
    shape_solver: str = "heuristic"  # or "cfd"
    refine_solves: int = 10
    # The quality the *search* ranks candidates at. Screening is cheap and
    # consistent, which is all a ranking needs -- provided the ordering it
    # finds survives the finer mesh the answer is read on. It need not: a long
    # shallow tail is exactly what a coarse mesh resolves worst, and on the
    # sample trike the two orderings disagree. So the loop confirms its winner
    # against the heuristic at the run's own quality and keeps whichever
    # really wins; set this to that quality to search there directly instead,
    # which is slower and removes the proxy altogether.
    refine_quality: str = "screening"
    # Solve the finished shell once, at the run's own quality, as part of the
    # derive. Deriving a shape and not knowing its drag is the gap this tool
    # exists to close, and a shape panel showing geometry beside no number at
    # all invites exactly the assumption the tool is meant to replace. Costs
    # one solve; the true loop's confirmation already is that solve, so the
    # loop path reuses it rather than paying twice.
    measure_shell: bool = True

    def to_dict(self) -> dict:
        return {
            "clearance": self.clearance,
            "anisotropy": self.anisotropy,
            "resolution": self.resolution,
            "streamline": self.streamline,
            "nose_angle_deg": self.nose_angle_deg,
            "tail_angle_deg": self.tail_angle_deg,
            "envelope_profile": self.envelope_profile,
            "shoulder_blend": self.shoulder_blend,
            "shape_solver": self.shape_solver,
            "refine_solves": self.refine_solves,
            "refine_quality": self.refine_quality,
            "measure_shell": self.measure_shell,
        }

    @property
    def blend(self) -> float:
        """The blend length actually in force, zero when faceted.

        One accessor rather than a `profile == "blended"` test at every call
        site, so the two settings can never disagree about what was built.
        """
        return self.shoulder_blend if self.envelope_profile == "blended" else 0.0

    @classmethod
    def from_dict(cls, data: dict | None) -> "PackagingSettings":
        data = data or {}
        defaults = cls()
        solver = str(data.get("shape_solver") or defaults.shape_solver)
        if solver not in ("heuristic", "cfd"):
            solver = defaults.shape_solver
        profile = str(data.get("envelope_profile") or defaults.envelope_profile)
        if profile not in ("faceted", "blended"):
            profile = defaults.envelope_profile
        return cls(
            clearance=_as_float(data.get("clearance"), defaults.clearance),
            anisotropy=_as_float(data.get("anisotropy"), defaults.anisotropy),
            resolution=int(data.get("resolution") or defaults.resolution),
            streamline=bool(data.get("streamline", defaults.streamline)),
            nose_angle_deg=_as_float(data.get("nose_angle_deg"), defaults.nose_angle_deg),
            tail_angle_deg=_as_float(data.get("tail_angle_deg"), defaults.tail_angle_deg),
            envelope_profile=profile,
            shoulder_blend=_as_float(data.get("shoulder_blend"), defaults.shoulder_blend),
            shape_solver=solver,
            refine_solves=max(int(data.get("refine_solves") or defaults.refine_solves), 3),
            refine_quality=(
                str(data.get("refine_quality") or defaults.refine_quality)
                if str(data.get("refine_quality") or "") in QUALITY_PRESETS
                else defaults.refine_quality
            ),
            measure_shell=bool(data.get("measure_shell", defaults.measure_shell)),
        )


@dataclass
class FairingSpec:
    """How the current hull was generated from the payload."""

    closing_radius: float
    clearance: float
    anisotropy: float
    components: int
    plateau_width: float = 0.0
    resolution: int = 128
    smoothing: int = 12
    streamlined: bool = False
    nose_angle_deg: float | None = None
    tail_angle_deg: float | None = None
    envelope_profile: str = "faceted"
    shoulder_blend: float = 0.0

    def to_dict(self) -> dict:
        return {
            "closing_radius": self.closing_radius,
            "clearance": self.clearance,
            "anisotropy": self.anisotropy,
            "components": self.components,
            "plateau_width": self.plateau_width,
            "resolution": self.resolution,
            "smoothing": self.smoothing,
            "streamlined": self.streamlined,
            "nose_angle_deg": self.nose_angle_deg,
            "tail_angle_deg": self.tail_angle_deg,
            "envelope_profile": self.envelope_profile,
            "shoulder_blend": self.shoulder_blend,
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "FairingSpec | None":
        if not data:
            return None
        return cls(
            closing_radius=_as_float(data.get("closing_radius"), 0.0),
            clearance=_as_float(data.get("clearance"), 0.02),
            anisotropy=_as_float(data.get("anisotropy"), 3.0),
            components=int(data.get("components") or 1),
            plateau_width=_as_float(data.get("plateau_width"), 0.0),
            resolution=int(data.get("resolution") or 128),
            smoothing=int(data.get("smoothing") or 12),
            streamlined=bool(data.get("streamlined", False)),
            nose_angle_deg=None if data.get("nose_angle_deg") is None
            else float(data["nose_angle_deg"]),
            tail_angle_deg=None if data.get("tail_angle_deg") is None
            else float(data["tail_angle_deg"]),
            envelope_profile=str(data.get("envelope_profile") or "faceted"),
            shoulder_blend=_as_float(data.get("shoulder_blend"), 0.0),
        )


@dataclass
class Geometry:
    """The STL itself, embedded so the scene stays portable."""

    stl_base64: str
    source_name: str = "hull.stl"
    scale: float = 1.0

    @classmethod
    def from_bytes(cls, data: bytes, source_name: str = "hull.stl", scale: float = 1.0) -> "Geometry":
        mesh = trimesh.load(io.BytesIO(data), file_type="stl", force="mesh")
        if not isinstance(mesh, trimesh.Trimesh) or mesh.is_empty:
            raise ValueError(f"{source_name} did not contain a usable triangle mesh")
        # Re-export so the embedded payload is always compact binary STL,
        # whatever the input was.
        binary = mesh.export(file_type="stl")
        return cls(
            stl_base64=base64.b64encode(binary).decode("ascii"),
            source_name=source_name,
            scale=scale,
        )

    def raw_mesh(self) -> trimesh.Trimesh:
        data = base64.b64decode(self.stl_base64)
        mesh = trimesh.load(io.BytesIO(data), file_type="stl", force="mesh")
        if self.scale != 1.0:
            mesh.apply_scale(self.scale)
        return mesh

    def to_dict(self) -> dict:
        return {
            "source_name": self.source_name,
            "scale": self.scale,
            "encoding": "base64:stl-binary",
            "stl_base64": self.stl_base64,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Geometry":
        if not data or not data.get("stl_base64"):
            raise ValueError("Scene geometry is missing its embedded STL")
        return cls(
            stl_base64=str(data["stl_base64"]),
            source_name=str(data.get("source_name") or "hull.stl"),
            scale=_as_float(data.get("scale"), 1.0),
        )


@dataclass
class SpeedPoint:
    speed: float
    drag_force: float
    drag_coefficient: float
    frontal_area: float
    reynolds: float
    lift_force: float | None = None
    source: str = "solved"  # "solved" for a real run, "scaled" for extrapolation

    def to_dict(self) -> dict:
        return {
            "speed": self.speed,
            "drag_force": self.drag_force,
            "drag_coefficient": self.drag_coefficient,
            "frontal_area": self.frontal_area,
            "reynolds": self.reynolds,
            "lift_force": self.lift_force,
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "SpeedPoint":
        return cls(
            speed=float(data["speed"]),
            drag_force=float(data["drag_force"]),
            drag_coefficient=float(data["drag_coefficient"]),
            frontal_area=float(data["frontal_area"]),
            reynolds=float(data.get("reynolds") or 0.0),
            lift_force=None if data.get("lift_force") is None else float(data["lift_force"]),
            source=str(data.get("source") or "solved"),
        )


@dataclass
class SolverRun:
    """Everything one backend produced for one scene."""

    solver: str
    status: str  # "ok", "failed", "unavailable"
    mode: str = "scale"  # "scale" or "sweep"
    points: list[SpeedPoint] = field(default_factory=list)
    wall_time_s: float = 0.0
    message: str = ""
    log_excerpt: str = ""
    settings: dict = field(default_factory=dict)
    # Did the coefficient settle, or was it still swinging at the last
    # iteration? An unconverged run is still "ok" -- it produced a number, and
    # suppressing it would be worse than reporting it with a caveat -- but a
    # search ranking candidates against each other needs to know, because two
    # unconverged solves differ by their noise as much as by their shapes.
    converged: bool = True

    def reference_point(self) -> SpeedPoint | None:
        solved = [point for point in self.points if point.source == "solved"]
        return solved[0] if solved else (self.points[0] if self.points else None)

    def to_dict(self) -> dict:
        return {
            "solver": self.solver,
            "status": self.status,
            "mode": self.mode,
            "points": [point.to_dict() for point in self.points],
            "wall_time_s": self.wall_time_s,
            "message": self.message,
            "log_excerpt": self.log_excerpt,
            "settings": self.settings,
            "converged": self.converged,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "SolverRun":
        return cls(
            solver=str(data.get("solver") or "unknown"),
            status=str(data.get("status") or "failed"),
            mode=str(data.get("mode") or "scale"),
            points=[SpeedPoint.from_dict(item) for item in data.get("points") or []],
            wall_time_s=_as_float(data.get("wall_time_s"), 0.0),
            message=str(data.get("message") or ""),
            log_excerpt=str(data.get("log_excerpt") or ""),
            settings=data.get("settings") or {},
            converged=bool(data.get("converged", True)),
        )


@dataclass
class ResultSet:
    title: str = ""
    description: str = ""
    computed_at: str = field(default_factory=_utc_now)
    host: str = field(default_factory=platform.node)
    geometry: dict = field(default_factory=dict)
    reynolds: dict = field(default_factory=dict)
    runs: list[SolverRun] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "title": self.title,
            "description": self.description,
            "computed_at": self.computed_at,
            "host": self.host,
            "geometry": self.geometry,
            "reynolds": self.reynolds,
            "runs": [run.to_dict() for run in self.runs],
            "warnings": list(self.warnings),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ResultSet":
        return cls(
            title=str(data.get("title") or ""),
            description=str(data.get("description") or ""),
            computed_at=str(data.get("computed_at") or ""),
            host=str(data.get("host") or ""),
            geometry=data.get("geometry") or {},
            reynolds=data.get("reynolds") or {},
            runs=[SolverRun.from_dict(item) for item in data.get("runs") or []],
            warnings=[str(item) for item in data.get("warnings") or []],
        )


@dataclass
class Scene:
    geometry: Geometry
    name: str = "untitled"
    orientation: Orientation = field(default_factory=Orientation)
    wind: Wind = field(default_factory=Wind)
    road: Road = field(default_factory=Road)
    fluid: Fluid = field(default_factory=Fluid)
    solver: SolverSettings = field(default_factory=SolverSettings)
    results: ResultSet | None = None
    created_at: str = field(default_factory=_utc_now)
    notes: str = ""
    # What has to fit inside. Every import sets this alongside the hull, so any
    # STL can either be flown as-is or faired -- one import path, both uses.
    payload: Geometry | None = None
    fairing: FairingSpec | None = None
    packaging: PackagingSettings = field(default_factory=PackagingSettings)
    # Numbers each computation on this scene, so titles stay distinguishable.
    run_index: int = 0

    def next_title(self, operation: str) -> tuple[int, str]:
        """Auto-numbered default title for a computation, from the scene name."""
        index = self.run_index + 1
        return index, f"{self.name} · {operation} #{index}"

    # ---------------------------------------------------------------- geometry

    def placement_transform(self) -> np.ndarray:
        """The 4x4 taking raw hull coordinates to solver coordinates.

        The hull is rotated about its own centroid, centred on the x/y origin,
        and lifted so its lowest point sits ``ride_height`` above the road
        plane at z = 0. With the road disabled the centroid goes to the origin
        instead, since nothing then breaks the vertical symmetry.

        Returned as a matrix rather than applied directly because the payload
        has to move with the hull, not be placed by its own centroid -- and if
        the two ever used different transforms, the picture of the payload
        sitting inside the fairing would be a lie.
        """
        mesh = self.geometry.raw_mesh()
        centroid = np.asarray(mesh.centroid, dtype=float)
        rotation = self.orientation.matrix()
        rotated = (np.asarray(mesh.vertices, dtype=float) - centroid) @ rotation.T

        shift = np.zeros(3)
        if self.road.enabled:
            shift[2] = self.road.ride_height - float(rotated[:, 2].min())

        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = shift - rotation @ centroid
        return transform

    def placed_mesh(self) -> trimesh.Trimesh:
        """The hull as the solver sees it."""
        mesh = self.geometry.raw_mesh()
        mesh.apply_transform(self.placement_transform())
        return mesh

    def placed_payload(self) -> trimesh.Trimesh | None:
        """The payload moved with the hull, so it stays where it belongs."""
        if self.payload is None:
            return None
        mesh = self.payload.raw_mesh()
        mesh.apply_transform(self.placement_transform())
        return mesh

    def wind_vector(self, speed: float | None = None) -> np.ndarray:
        direction = self.wind.direction()
        magnitude = self.wind.speed if speed is None else float(speed)
        return direction * magnitude

    def ground_offset(self) -> float | None:
        """Ground offset in the convention the OpenFOAM backend expects."""
        return self.road.ride_height if self.road.enabled else None

    def road_velocity(self, speed: float | None = None) -> np.ndarray:
        """Road surface velocity at this point on the speed curve."""
        return self.road.velocity(self.wind_vector(speed))

    def metrics(self, resolution: int = 512) -> GeometryMetrics:
        return geometry_metrics(self.placed_mesh(), self.wind.direction(), resolution=resolution)

    def reynolds_advice(self, reference_length: float | None = None) -> ReynoldsAdvice:
        if reference_length is None:
            from metrics import streamwise_length

            reference_length = streamwise_length(self.placed_mesh(), self.wind.direction())
        return advise_speed_mode(
            self.solver.speed_min,
            self.solver.speed_max,
            reference_length,
            self.fluid.density,
            self.fluid.viscosity,
        )

    def resolved_sweep_mode(self, reference_length: float | None = None) -> tuple[str, ReynoldsAdvice]:
        advice = self.reynolds_advice(reference_length)
        mode = self.solver.sweep_mode
        if mode not in {"scale", "sweep"}:
            mode = advice.mode
        return mode, advice

    # -------------------------------------------------------------------- I/O

    @property
    def computed(self) -> bool:
        return self.results is not None and bool(self.results.runs)

    def copy(self) -> "Scene":
        return copy.deepcopy(self)

    def without_results(self) -> "Scene":
        return replace(self.copy(), results=None)

    def to_dict(self, include_geometry: bool = True) -> dict:
        data = {
            "format": FORMAT_NAME,
            "version": FORMAT_VERSION,
            "name": self.name,
            "created_at": self.created_at,
            "notes": self.notes,
            "orientation": self.orientation.to_dict(),
            "wind": self.wind.to_dict(),
            "road": self.road.to_dict(),
            "fluid": self.fluid.to_dict(),
            "solver": self.solver.to_dict(),
            "packaging": self.packaging.to_dict(),
            "fairing": self.fairing.to_dict() if self.fairing else None,
            "run_index": self.run_index,
            "results": self.results.to_dict() if self.results else None,
        }
        if include_geometry:
            data["geometry"] = self.geometry.to_dict()
            data["payload"] = self.payload.to_dict() if self.payload else None
        else:
            data["geometry"] = {
                "source_name": self.geometry.source_name,
                "scale": self.geometry.scale,
            }
            data["payload"] = (
                {"source_name": self.payload.source_name, "scale": self.payload.scale}
                if self.payload
                else None
            )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "Scene":
        if data.get("format") not in (None, FORMAT_NAME):
            raise ValueError(f"Not an {FORMAT_NAME} document: format={data.get('format')!r}")
        version = int(data.get("version") or FORMAT_VERSION)
        if version != FORMAT_VERSION:
            raise ValueError(
                f"Scene file is format version {version}; this build reads version "
                f"{FORMAT_VERSION} only. Rebuild it from the source STL."
            )

        results = data.get("results")
        payload = data.get("payload")
        return cls(
            geometry=Geometry.from_dict(data.get("geometry") or {}),
            payload=Geometry.from_dict(payload) if payload and payload.get("stl_base64") else None,
            fairing=FairingSpec.from_dict(data.get("fairing")),
            packaging=PackagingSettings.from_dict(data.get("packaging")),
            run_index=int(data.get("run_index") or 0),
            name=str(data.get("name") or "untitled"),
            orientation=Orientation.from_dict(data.get("orientation")),
            wind=Wind.from_dict(data.get("wind")),
            road=Road.from_dict(data.get("road")),
            fluid=Fluid.from_dict(data.get("fluid")),
            solver=SolverSettings.from_dict(data.get("solver")),
            results=ResultSet.from_dict(results) if results else None,
            created_at=str(data.get("created_at") or _utc_now()),
            notes=str(data.get("notes") or ""),
        )

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Scene":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_stl_file(cls, path: str | Path, name: str | None = None, **kwargs) -> "Scene":
        path = Path(path)
        geometry = Geometry.from_bytes(path.read_bytes(), source_name=path.name)
        return cls(geometry=geometry, name=name or path.stem, **kwargs)
