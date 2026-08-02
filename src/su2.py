"""SU2 drag backend for an arbitrary STL.

The original ``drag.py`` could only mesh a hard-coded 1 m cube. This module
generalises that track: it wraps any triangle mesh in a box, meshes the fluid
volume with the gmsh Python API, writes an incompressible RANS config and runs
``SU2_CFD``.

Two deliberate choices make the SU2 numbers directly comparable with the
OpenFOAM ones:

* the flow domain is sized by the same rule as the OpenFOAM case, and
* ``REF_AREA`` is the same true frontal area, so both solvers report Cd on
  the same reference.

The free stream is given to SU2 as an explicit velocity vector with AOA and
sideslip left at zero, and drag is recovered by projecting the reported force
coefficients ``(CFx, CFy, CFz)`` onto the wind direction. That sidesteps any
ambiguity about how SU2 orients its own lift/drag axes for an arbitrary wind.

Mesh generation runs locally through the gmsh Python module on any platform.
The solver itself runs wherever :mod:`execution` finds it: natively when
``SU2_CFD`` is on PATH, in a pinned Docker image, or through WSL -- including
the ``~/su2-install/bin`` location that ``setup.sh`` produces.
"""

from __future__ import annotations

import csv
import math
import os
import shlex
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

import execution as execution_env
from execution import Runner
from metrics import flow_domain, frontal_area, normalize


WSL_DISTRO = "Ubuntu-22.04"
# Where setup.sh installs SU2. Its PATH export lands at the end of ~/.bashrc,
# past the non-interactive early return, so a non-interactive shell will not
# see it and we have to look here explicitly.
WSL_SU2_BIN = "$HOME/su2-install/bin"
# The container puts SU2 on PATH through its own ENV, so no preamble is needed.
CONTAINER_SU2_BIN = "/opt/su2/bin"


def _wsl_bash(script: str, check: bool = True, capture_output: bool = True, timeout: int | None = None):
    return subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-e", "bash", "-lc", script],
        check=check,
        capture_output=capture_output,
        text=True,
        timeout=timeout,
    )


def _wsl_path(path: str | Path) -> str:
    completed = _wsl_bash(f"wslpath -a {shlex.quote(str(Path(path)))}")
    return completed.stdout.strip()


def _native_runner(processes: int) -> Runner | None:
    native = shutil.which("SU2_CFD")
    if not native:
        return None
    return Runner(mode="native", processes=processes, executable=native, label="SU2_CFD")


def _docker_runner(processes: int) -> Runner | None:
    image = execution_env.SU2_IMAGE
    if not execution_env.docker_available() or not execution_env.image_present(image):
        return None
    return Runner(
        mode="docker",
        processes=processes,
        image=image,
        executable="SU2_CFD",
        preamble=f"export PATH={CONTAINER_SU2_BIN}:$PATH",
        label="SU2_CFD",
    )


def _wsl_runner(processes: int) -> Runner | None:
    try:
        probe = _wsl_bash(
            f"command -v SU2_CFD || (test -x {WSL_SU2_BIN}/SU2_CFD && echo {WSL_SU2_BIN}/SU2_CFD)",
            check=False,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    found = probe.stdout.strip().splitlines()
    if probe.returncode != 0 or not found:
        return None
    return Runner(
        mode="wsl",
        processes=processes,
        distro=WSL_DISTRO,
        executable=found[-1].strip(),
        preamble=f"export PATH={WSL_SU2_BIN}:$PATH",
        label="SU2_CFD",
    )


_PROBES = {"native": _native_runner, "docker": _docker_runner, "wsl": _wsl_runner}


def detect_su2(processes: int | None = None) -> Runner | None:
    """Find a usable SU2_CFD. Returns None if there is none.

    Order is native, Docker, WSL -- fastest first, then the reproducible one,
    then the legacy install. ``AERO_EXECUTION`` pins a single mode instead.
    """
    count = execution_env.resolve_processes(processes)
    forced = execution_env.forced_mode()
    order = [forced] if forced else list(execution_env.MODES)
    for mode in order:
        runner = _PROBES[mode](count)
        if runner is not None:
            return runner
    return None


def su2_available() -> bool:
    return detect_su2() is not None


def gmsh_available() -> bool:
    try:
        import gmsh  # noqa: F401
    except ImportError:
        return False
    return True


# --------------------------------------------------------------------------
# Mesh generation
# --------------------------------------------------------------------------


def _smallest_body_gap(mesh: trimesh.Trimesh) -> float | None:
    """Closest approach between separate bodies in the mesh, or None."""
    try:
        from fairing import smallest_gap

        bodies = mesh.split(only_watertight=False)
        return smallest_gap(list(bodies))
    except Exception:
        return None


def _surface_shells(gmsh, surface_tags: list[int]) -> list[list[int]]:
    """Group surface patches into closed shells by shared bounding curves.

    After classifySurfaces a single STL body is split into many patches. Two
    patches belong to the same shell exactly when they share a curve, so a
    union-find over the curve-to-surface map recovers one group per body.
    """
    parent = {tag: tag for tag in surface_tags}

    def find(tag: int) -> int:
        while parent[tag] != tag:
            parent[tag] = parent[parent[tag]]
            tag = parent[tag]
        return tag

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    by_curve: dict[int, list[int]] = {}
    for tag in surface_tags:
        for _, curve in gmsh.model.getBoundary([(2, tag)], oriented=False, recursive=False):
            by_curve.setdefault(abs(int(curve)), []).append(tag)

    for shared in by_curve.values():
        for other in shared[1:]:
            union(shared[0], other)

    groups: dict[int, list[int]] = {}
    for tag in surface_tags:
        groups.setdefault(find(tag), []).append(tag)
    return [sorted(group) for group in groups.values()]


def build_volume_mesh(
    mesh: trimesh.Trimesh,
    output_path: str | Path,
    ground: bool,
    refinement_level: int = 3,
    surface_cells: int = 25,
    farfield_cells: int = 8,
    classify_angle_deg: float = 40.0,
    verbose: bool = False,
) -> dict:
    """Mesh the fluid volume between ``mesh`` and its surrounding box.

    Returns a summary dict with the domain, marker names and element counts.

    The STL is merged as a discrete surface, re-classified into parametrisable
    patches and given a geometry so it can bound a volume. The box is built
    face by face rather than as a primitive, because each face needs its own
    physical group: the bottom becomes a viscous road when ``ground`` is set,
    and part of the far field otherwise.
    """
    try:
        import gmsh
    except ImportError as error:  # pragma: no cover - depends on install
        raise RuntimeError(
            "The gmsh Python module is required to build SU2 meshes (pip install gmsh)"
        ) from error

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    domain_min, domain_max = flow_domain(mesh, ground=ground)
    extents = np.asarray(mesh.extents, dtype=float)
    body_scale = float(np.max(extents))
    surface_size = body_scale / max(surface_cells, 1)
    far_size = body_scale / max(farfield_cells, 1) * 4.0

    # Separate lumps need surface elements smaller than the gap between them,
    # or the re-meshed patches interpenetrate and the 3D mesher fails on a
    # self-intersecting boundary. Floor it so a hairline gap cannot demand an
    # unbuildable mesh -- that case is caught and reported below instead.
    gap = _smallest_body_gap(mesh)
    if gap is not None and gap > 0:
        surface_size = min(surface_size, max(gap / 3.0, body_scale / 400.0))
        if surface_size < body_scale / 300.0:
            raise RuntimeError(
                f"Separate bodies come within {gap * 1000:.1f} mm on a {body_scale:.2f} m hull. "
                "Resolving a channel that narrow would need an unreasonable mesh; merge those "
                "bodies with a larger closing radius, or open the gap up."
            )

    stl_path = output_path.parent / "body.stl"
    mesh.export(stl_path)

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1 if verbose else 0)
        gmsh.model.add("aero")

        gmsh.merge(str(stl_path))

        # Rebuild a usable geometry on top of the raw triangles so the STL can
        # act as an internal boundary of the fluid volume. This has to happen
        # exactly once for the whole model: classifySurfaces reclassifies every
        # discrete entity present, so calling it per body invalidates the tags
        # of the bodies already processed.
        angle = math.radians(classify_angle_deg)
        gmsh.model.mesh.classifySurfaces(angle, True, True, math.radians(180.0))
        gmsh.model.mesh.createGeometry()

        body_tags = [tag for dim, tag in gmsh.model.getEntities(2)]
        if not body_tags:
            raise RuntimeError("gmsh found no surfaces in the STL")

        # A fairing is often several separate lumps -- a main body plus wheel
        # pods. Each closed shell needs its own surface loop; bundling them all
        # into one describes a single impossible surface and the 3D mesher
        # fails. Recover the shells by grouping patches that share a curve.
        body_loops = [
            gmsh.model.geo.addSurfaceLoop(shell) for shell in _surface_shells(gmsh, body_tags)
        ]

        x0, y0, z0 = domain_min
        x1, y1, z1 = domain_max
        corners = [
            (x0, y0, z0),
            (x1, y0, z0),
            (x1, y1, z0),
            (x0, y1, z0),
            (x0, y0, z1),
            (x1, y0, z1),
            (x1, y1, z1),
            (x0, y1, z1),
        ]
        points = [gmsh.model.geo.addPoint(x, y, z, far_size) for x, y, z in corners]
        p1, p2, p3, p4, p5, p6, p7, p8 = points

        bottom = [
            gmsh.model.geo.addLine(p1, p2),
            gmsh.model.geo.addLine(p2, p3),
            gmsh.model.geo.addLine(p3, p4),
            gmsh.model.geo.addLine(p4, p1),
        ]
        top = [
            gmsh.model.geo.addLine(p5, p6),
            gmsh.model.geo.addLine(p6, p7),
            gmsh.model.geo.addLine(p7, p8),
            gmsh.model.geo.addLine(p8, p5),
        ]
        risers = [
            gmsh.model.geo.addLine(p1, p5),
            gmsh.model.geo.addLine(p2, p6),
            gmsh.model.geo.addLine(p3, p7),
            gmsh.model.geo.addLine(p4, p8),
        ]

        def face(loop_lines: list[int]) -> int:
            loop = gmsh.model.geo.addCurveLoop(loop_lines)
            return gmsh.model.geo.addPlaneSurface([loop])

        zmin_face = face(bottom)
        zmax_face = face(top)
        ymin_face = face([bottom[0], risers[1], -top[0], -risers[0]])
        xmax_face = face([bottom[1], risers[2], -top[1], -risers[1]])
        ymax_face = face([bottom[2], risers[3], -top[2], -risers[2]])
        xmin_face = face([bottom[3], risers[0], -top[3], -risers[3]])

        box_faces = [zmin_face, zmax_face, ymin_face, xmax_face, ymax_face, xmin_face]
        box_loop = gmsh.model.geo.addSurfaceLoop(box_faces)

        # The fluid is the box with every body carved out of it.
        gmsh.model.geo.addVolume([box_loop, *body_loops])
        gmsh.model.geo.synchronize()

        gmsh.model.addPhysicalGroup(2, body_tags, name="body")
        if ground:
            gmsh.model.addPhysicalGroup(2, [zmin_face], name="ground")
            farfield_faces = [zmax_face, ymin_face, xmax_face, ymax_face, xmin_face]
        else:
            farfield_faces = box_faces
        gmsh.model.addPhysicalGroup(2, farfield_faces, name="farfield")
        gmsh.model.addPhysicalGroup(3, [1], name="fluid")

        # Fine cells on the body, coarsening outward.
        distance = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(distance, "SurfacesList", body_tags)
        gmsh.model.mesh.field.setNumber(distance, "Sampling", 200)

        threshold = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(threshold, "InField", distance)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMin", surface_size)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMax", far_size)
        # Hold the fine size only in a thin band around the surface. Scaling
        # DistMin off the body length instead filled a half-body-deep halo with
        # surface-sized cells, which is what made a modest case reach millions.
        gmsh.model.mesh.field.setNumber(threshold, "DistMin", surface_size * 2.0)
        gmsh.model.mesh.field.setNumber(threshold, "DistMax", body_scale * 1.5)
        gmsh.model.mesh.field.setAsBackgroundMesh(threshold)

        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.Algorithm3D", 10)  # HXT: fast and robust
        gmsh.option.setNumber("Mesh.Optimize", 1)

        try:
            gmsh.model.mesh.generate(3)
        except Exception as error:
            detail = ""
            if gap is not None and gap > 0 and gap < body_scale * 0.06:
                detail = (
                    f" The closest approach between separate bodies is {gap * 1000:.1f} mm on a "
                    f"{body_scale:.2f} m body. A gap that narrow chokes the flow and needs cells "
                    "finer than the gap; merge those bodies (a larger closing radius) or open the "
                    "gap up."
                )
            raise RuntimeError(f"gmsh could not mesh the fluid volume: {error}.{detail}") from error

        node_tags, _, _ = gmsh.model.mesh.getNodes()
        element_types, element_tags, _ = gmsh.model.mesh.getElements(3)
        cell_count = int(sum(len(tags) for tags in element_tags))

        gmsh.write(str(output_path))

        return {
            "mesh_file": str(output_path),
            "domain_min": np.asarray(domain_min).tolist(),
            "domain_max": np.asarray(domain_max).tolist(),
            "node_count": int(len(node_tags)),
            "cell_count": cell_count,
            "surface_size": surface_size,
            "farfield_size": far_size,
            "markers": ["body", "ground", "farfield"] if ground else ["body", "farfield"],
        }
    finally:
        gmsh.finalize()


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------


def write_su2_config(
    path: str | Path,
    mesh_filename: str,
    wind_vector: np.ndarray,
    density: float,
    viscosity: float,
    reference_area: float,
    reference_length: float,
    ground: bool,
    turbulent: bool = True,
    iterations: int = 400,
    cfl: float = 5.0,
    road_velocity: np.ndarray | None = None,
) -> Path:
    """Write an incompressible SU2 config for this case."""
    path = Path(path)
    wind = np.asarray(wind_vector, dtype=float)
    speed = float(np.linalg.norm(wind))
    reynolds = density * speed * reference_length / viscosity

    wall_marker = "MARKER_HEATFLUX= ( body, 0.0" + (", ground, 0.0" if ground else "") + " )"

    # A translating road, written the way SU2's own moving-wall cases are: the
    # ground stays a viscous wall and additionally gets a surface velocity.
    # Omitted entirely when nothing moves, so a static case is byte-for-byte
    # what it was before.
    road = np.zeros(3) if road_velocity is None else np.asarray(road_velocity, dtype=float)
    road_block = ""
    if ground and float(np.linalg.norm(road)) > 1e-12:
        road_block = f"""
% ---------------- Moving road ----------------
% The frame is the vehicle's, so the ground runs downstream at the vehicle's
% speed rather than standing still and growing a boundary layer no road has.
SURFACE_MOVEMENT= MOVING_WALL
MARKER_MOVING= ( ground )
SURFACE_TRANSLATION_RATE= {road[0]:.10g} {road[1]:.10g} {road[2]:.10g}
"""

    solver = "INC_RANS" if turbulent else "INC_NAVIER_STOKES"
    turb_block = "KIND_TURB_MODEL= SST\n" if turbulent else ""
    turb_numerics = (
        "CONV_NUM_METHOD_TURB= SCALAR_UPWIND\n"
        "MUSCL_TURB= NO\n"
        "TIME_DISCRE_TURB= EULER_IMPLICIT\n"
        if turbulent
        else ""
    )
    turb_freestream = (
        "FREESTREAM_TURBULENCEINTENSITY= 0.05\n" "FREESTREAM_TURB2LAMVISCRATIO= 10.0\n" if turbulent else ""
    )

    content = f"""%
% SU2 configuration generated by the aero drag tool.
%
% Drag is recovered from the force coefficients CFx/CFy/CFz projected onto the
% wind direction, so AOA and SIDESLIP_ANGLE stay at zero and the free stream is
% given directly as a velocity vector.
%

SOLVER= {solver}
{turb_block}MATH_PROBLEM= DIRECT
RESTART_SOL= NO

% ---------------- Incompressible free stream ----------------
INC_DENSITY_MODEL= CONSTANT
INC_ENERGY_EQUATION= NO
INC_DENSITY_INIT= {density:.10g}
INC_VELOCITY_INIT= ( {wind[0]:.10g}, {wind[1]:.10g}, {wind[2]:.10g} )
INC_NONDIM= DIMENSIONAL

VISCOSITY_MODEL= CONSTANT_VISCOSITY
MU_CONSTANT= {viscosity:.10g}
{turb_freestream}
REYNOLDS_NUMBER= {reynolds:.10g}
REYNOLDS_LENGTH= {reference_length:.10g}

AOA= 0.0
SIDESLIP_ANGLE= 0.0

% ---------------- Reference quantities ----------------
% REF_AREA is the true frontal silhouette area, matching the OpenFOAM case so
% both solvers report Cd on the same reference.
REF_ORIGIN_MOMENT_X= 0.0
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0
REF_AREA= {reference_area:.10g}
REF_LENGTH= {reference_length:.10g}
REF_DIMENSIONALIZATION= DIMENSIONAL

% ---------------- Markers ----------------
{wall_marker}
MARKER_FAR= ( farfield )
MARKER_MONITORING= ( body )
MARKER_PLOTTING= ( body )
MARKER_ANALYZE= ( body )
{road_block}
% ---------------- Numerics ----------------
NUM_METHOD_GRAD= WEIGHTED_LEAST_SQUARES
CONV_NUM_METHOD_FLOW= FDS
MUSCL_FLOW= YES
SLOPE_LIMITER_FLOW= VENKATAKRISHNAN
VENKAT_LIMITER_COEFF= 0.05
TIME_DISCRE_FLOW= EULER_IMPLICIT
{turb_numerics}
LINEAR_SOLVER= FGMRES
LINEAR_SOLVER_PREC= ILU
LINEAR_SOLVER_ERROR= 1E-6
LINEAR_SOLVER_ITER= 10

CFL_NUMBER= {cfl:.10g}
CFL_ADAPT= YES
CFL_ADAPT_PARAM= ( 0.5, 1.5, 1.0, 50.0 )

ITER= {int(iterations)}
CONV_RESIDUAL_MINVAL= -10
CONV_STARTITER= 10

% ---------------- I/O ----------------
MESH_FILENAME= {mesh_filename}
MESH_FORMAT= SU2

SCREEN_OUTPUT= ( INNER_ITER, RMS_PRESSURE, RMS_VELOCITY-X, DRAG, LIFT )
HISTORY_OUTPUT= ( ITER, RMS_RES, AERO_COEFF )
OUTPUT_FILES= ( RESTART, PARAVIEW, SURFACE_CSV )

CONV_FILENAME= history
RESTART_FILENAME= restart
VOLUME_FILENAME= flow
SURFACE_FILENAME= surface
"""
    path.write_text(content, encoding="utf-8")
    return path


# --------------------------------------------------------------------------
# History parsing
# --------------------------------------------------------------------------


def parse_history(path: str | Path) -> dict[str, np.ndarray]:
    """Read an SU2 history.csv into named columns."""
    path = Path(path)
    with path.open(newline="", encoding="utf-8", errors="ignore") as handle:
        rows = list(csv.reader(handle))
    if not rows:
        raise RuntimeError(f"{path.name} is empty")

    header = [cell.strip().strip('"').strip() for cell in rows[0]]
    values: list[list[float]] = []
    for row in rows[1:]:
        try:
            values.append([float(cell) for cell in row[: len(header)]])
        except ValueError:
            continue
    if not values:
        raise RuntimeError(f"No numeric rows in {path.name}")

    width = min(len(header), min(len(row) for row in values))
    table = np.array([row[:width] for row in values], dtype=float)
    return {header[index]: table[:, index] for index in range(width)}


def _tail_mean(series: np.ndarray | None, fraction: float = 0.2) -> float | None:
    if series is None or len(series) == 0:
        return None
    count = max(int(len(series) * fraction), 1)
    tail = series[-count:]
    finite = tail[np.isfinite(tail)]
    if len(finite) == 0:
        return None
    return float(np.mean(finite))


def drag_coefficient_from_history(history: dict[str, np.ndarray], wind_direction: np.ndarray) -> tuple[float, float | None, float]:
    """Return (Cd along the wind, Cl if available, peak-to-peak Cd spread).

    Prefers projecting the force-coefficient vector onto the wind, which is
    correct for any wind direction. Falls back to SU2's own CD column when the
    force components were not written.
    """
    direction = normalize(wind_direction)
    components = [history.get("CFx"), history.get("CFy"), history.get("CFz")]

    if all(item is not None for item in components):
        length = min(len(item) for item in components)
        stacked = np.column_stack([item[:length] for item in components])
        series = stacked @ direction
    else:
        series = history.get("CD")
        if series is None:
            raise RuntimeError(
                f"SU2 history has neither CFx/CFy/CFz nor CD (columns: {sorted(history)})"
            )

    value = _tail_mean(series)
    if value is None:
        raise RuntimeError("SU2 history contained no usable drag values")

    count = max(int(len(series) * 0.2), 1)
    tail = series[-count:]
    finite = tail[np.isfinite(tail)]
    spread = float(np.max(finite) - np.min(finite)) if len(finite) > 1 else 0.0

    lift = _tail_mean(history.get("CL"))
    return float(value), lift, spread


# --------------------------------------------------------------------------
# Case preparation and running
# --------------------------------------------------------------------------


@dataclass
class SU2Result:
    drag_force: float
    drag_coefficient: float
    case_dir: Path
    lift_coefficient: float | None = None
    lift_force: float | None = None
    reference_area: float = 0.0
    reference_length: float = 0.0
    cd_spread: float | None = None
    converged: bool = False
    log_excerpt: str = ""
    settings: dict = field(default_factory=dict)


def prepare_su2_case(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    case_dir: str | Path,
    density: float = 1.225,
    viscosity: float = 1.8e-5,
    ground_offset: float | None = None,
    turbulent: bool = True,
    iterations: int = 400,
    surface_cells: int = 25,
    refinement_level: int = 3,
    reference_area: float | None = None,
    road_velocity: np.ndarray | None = None,
) -> dict:
    """Write a complete, self-contained SU2 case (mesh + config) to disk.

    Useful on its own: the directory can be zipped up and run by hand on a
    machine that has SU2 but not this tool.
    """
    case_dir = Path(case_dir)
    case_dir.mkdir(parents=True, exist_ok=True)

    case_mesh = mesh.copy()
    if ground_offset is not None:
        shift = float(ground_offset) - float(np.min(case_mesh.vertices[:, 2]))
        case_mesh.apply_translation([0.0, 0.0, shift])

    ground = ground_offset is not None
    if reference_area is None:
        reference_area = frontal_area(case_mesh, wind_vector)
    reference_length = float(np.max(np.asarray(case_mesh.extents, dtype=float)))

    mesh_info = build_volume_mesh(
        case_mesh,
        case_dir / "mesh.su2",
        ground=ground,
        refinement_level=refinement_level,
        surface_cells=surface_cells,
    )

    write_su2_config(
        case_dir / "case.cfg",
        mesh_filename="mesh.su2",
        wind_vector=wind_vector,
        density=density,
        viscosity=viscosity,
        reference_area=float(reference_area),
        reference_length=reference_length,
        ground=ground,
        turbulent=turbulent,
        iterations=iterations,
        road_velocity=road_velocity,
    )

    return {
        **mesh_info,
        "case_dir": str(case_dir),
        "config": str(case_dir / "case.cfg"),
        "reference_area": float(reference_area),
        "reference_length": reference_length,
        "ground": ground,
        "road_speed": float(np.linalg.norm(road_velocity)) if road_velocity is not None else 0.0,
    }


def _run_su2_command(case_dir: Path, runner: Runner) -> Path:
    log_path = case_dir / "log.su2"

    if runner.mode == "native":
        # Kept off the bash path: a native install need not have a shell that
        # understands -lc, and this already worked.
        if runner.processes > 1 and shutil.which("mpirun"):
            command = ["mpirun", "-np", str(runner.processes), runner.executable, "case.cfg"]
        else:
            command = [runner.executable, "case.cfg"]
        with log_path.open("w", encoding="utf-8") as handle:
            execution_env.run_process(
                command, cwd=case_dir, stdout=handle, stderr=subprocess.STDOUT
            )
        return log_path

    binary = shlex.quote(runner.executable)
    if runner.processes > 1:
        # --allow-run-as-root is harmless outside a container and required
        # inside one, where the build user is root.
        inner = f"mpirun -np {runner.processes} --allow-run-as-root {binary} case.cfg"
    else:
        inner = f"{binary} case.cfg"
    runner.bash(f"{inner} > log.su2 2>&1", case_dir=case_dir, check=False, capture_output=True)
    return log_path


def run_su2_drag(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float = 1.225,
    viscosity: float = 1.8e-5,
    ground_offset: float | None = None,
    work_dir: str | Path | None = None,
    keep_case: bool = False,
    turbulent: bool = True,
    iterations: int = 400,
    surface_cells: int = 25,
    refinement_level: int = 3,
    processes: int | None = None,
    reference_area: float | None = None,
    road_velocity: np.ndarray | None = None,
) -> SU2Result:
    runner = detect_su2(processes=processes)
    if runner is None:
        raise RuntimeError(
            "SU2_CFD was not found on PATH, in Docker or in WSL. Build the container "
            "image (docker/build.sh), install SU2 with setup.sh, or save the scene and "
            "compute it on a machine that has SU2 (see runner.py)."
        )

    wind_vector = np.asarray(wind_vector, dtype=float)
    speed = float(np.linalg.norm(wind_vector))
    if speed < 1e-12:
        raise ValueError("Wind vector must be non-zero")

    if work_dir is None:
        case_dir = Path(tempfile.mkdtemp(prefix="su2_case_"))
    else:
        case_dir = Path(work_dir)
        case_dir.mkdir(parents=True, exist_ok=True)

    setup = prepare_su2_case(
        mesh,
        wind_vector,
        case_dir,
        density=density,
        viscosity=viscosity,
        ground_offset=ground_offset,
        turbulent=turbulent,
        iterations=iterations,
        surface_cells=surface_cells,
        refinement_level=refinement_level,
        reference_area=reference_area,
        road_velocity=road_velocity,
    )

    log_path = _run_su2_command(case_dir, runner)
    log_text = log_path.read_text(encoding="utf-8", errors="ignore") if log_path.exists() else ""
    excerpt = "\n".join(log_text.splitlines()[-25:])

    history_path = case_dir / "history.csv"
    if not history_path.exists():
        candidates = sorted(case_dir.glob("history*.csv")) + sorted(case_dir.glob("history*.dat"))
        history_path = candidates[0] if candidates else None
    if history_path is None:
        raise RuntimeError("SU2 produced no history file. Tail of the log:\n" + excerpt)

    history = parse_history(history_path)
    drag_coefficient, lift_coefficient, spread = drag_coefficient_from_history(
        history, wind_vector / speed
    )

    area = float(setup["reference_area"])
    dynamic_pressure = 0.5 * density * speed * speed
    drag_force = dynamic_pressure * area * drag_coefficient
    lift_force = None if lift_coefficient is None else dynamic_pressure * area * lift_coefficient
    converged = bool(abs(drag_coefficient) > 1e-9 and spread / abs(drag_coefficient) < 0.05)

    result = SU2Result(
        drag_force=float(drag_force),
        drag_coefficient=float(drag_coefficient),
        case_dir=case_dir,
        lift_coefficient=None if lift_coefficient is None else float(lift_coefficient),
        lift_force=None if lift_force is None else float(lift_force),
        reference_area=area,
        reference_length=float(setup["reference_length"]),
        cd_spread=spread,
        converged=converged,
        log_excerpt=excerpt,
        settings={
            "turbulence": "SST" if turbulent else "laminar",
            "iterations": iterations,
            "moving_ground": bool(setup["road_speed"] > 1e-12),
            "road_speed": float(setup["road_speed"]),
            "surface_cells": surface_cells,
            "cells": setup.get("cell_count"),
            "nodes": setup.get("node_count"),
            "execution": runner.describe(),
        },
    )

    if not keep_case:
        shutil.rmtree(case_dir, ignore_errors=True)
    return result
