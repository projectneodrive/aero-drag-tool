"""OpenFOAM 13 drag backend, driven through WSL.

Generates a complete case from a triangle mesh (blockMesh background grid,
snappyHexMesh body fit, incompressibleFluid steady solve with a forceCoeffs
function object) and reads the drag coefficient back out.

Notes on OpenFOAM 13 specifics:

* ``simpleFoam`` only survives as a shim that re-executes
  ``foamRun -solver incompressibleFluid``; we call the latter directly.
* Coefficients are written by the function object *during* the solve, into
  ``postProcessing/forceCoeffsIncompressible/<t>/forceCoeffs.dat``. Running
  ``postProcess -func ...`` afterwards fails with "Could not find U, p", so
  we read the file the solver already wrote.
* That file's columns are ``Time Cm Cd Cl Cl(f) Cl(r)`` -- Cd is the *third*
  column, and it must be located by header name rather than by position.
"""

from __future__ import annotations

import re
import shlex
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

from metrics import flow_domain, frontal_area


WSL_DISTRO = "Ubuntu-22.04"
OPENFOAM_BASHRC = "/opt/openfoam13/etc/bashrc"

# Standard k-omega closure constant, used to seed omega from k.
C_MU = 0.09
# Free-stream turbulence intensity and the fraction of the body length used
# as the inlet turbulent length scale.
TURBULENCE_INTENSITY = 0.05
TURBULENCE_LENGTH_FRACTION = 0.07


def _normalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("Vector must be non-zero")
    return vector / norm


def openfoam_available() -> bool:
    try:
        result = subprocess.run(
            ["wsl", "-d", WSL_DISTRO, "-u", "root", "-e", "bash", "-lc", f"test -f {shlex.quote(OPENFOAM_BASHRC)}"],
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


def windows_to_wsl_path(path: str | Path) -> str:
    path_text = str(Path(path))
    completed = subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-u", "root", "-e", "bash", "-lc", f"wslpath -a {shlex.quote(path_text)}"],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def run_wsl_bash(
    script: str,
    cwd: str | Path | None = None,
    capture_output: bool = False,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    command = f"source {shlex.quote(OPENFOAM_BASHRC)}"
    if cwd is not None:
        command += f" && cd {shlex.quote(windows_to_wsl_path(cwd))}"
    command += f" && {script}"
    return subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-u", "root", "-e", "bash", "-lc", command],
        check=check,
        capture_output=capture_output,
        text=True,
    )


def orthonormal_basis(direction: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    drag_dir = _normalize(direction)
    reference = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(reference, drag_dir))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0], dtype=float)
    lift_dir = _normalize(np.cross(drag_dir, reference))
    pitch_axis = _normalize(np.cross(drag_dir, lift_dir))
    return drag_dir, lift_dir, pitch_axis


def _vector_to_foam(vector: np.ndarray) -> str:
    vector = np.asarray(vector, dtype=float)
    return f"({vector[0]:.10g} {vector[1]:.10g} {vector[2]:.10g})"


def _foam_scalar(value: float) -> str:
    return f"{float(value):.10g}"


_HEADER = '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/'''


def _foam_file(class_name: str, object_name: str, location: str | None = None) -> str:
    location_line = f'\n    location    "{location}";' if location else ""
    return f"""{_HEADER}
FoamFile
{{
    format      ascii;
    class       {class_name};{location_line}
    object      {object_name};
}}
"""


def _block_cell_counts(domain_min: np.ndarray, domain_max: np.ndarray, resolution: int) -> tuple[int, int, int]:
    """Cell counts giving roughly cubic background cells."""
    extents = np.asarray(domain_max, dtype=float) - np.asarray(domain_min, dtype=float)
    longest = float(np.max(extents))
    if longest <= 0.0:
        raise ValueError("Domain has zero extent")
    counts = np.maximum(np.round(resolution * extents / longest), 8).astype(int)
    return int(counts[0]), int(counts[1]), int(counts[2])


def _block_mesh_dict(
    domain_min: np.ndarray,
    domain_max: np.ndarray,
    project_area: float,
    resolution: int,
) -> str:
    x0, y0, z0 = domain_min
    x1, y1, z1 = domain_max
    nx, ny, nz = _block_cell_counts(domain_min, domain_max, resolution)
    return f'''{_foam_file("dictionary", "blockMeshDict")}
convertToMeters 1;

projArea        {_foam_scalar(project_area)};

vertices
(
    ({x0:.10g} {y0:.10g} {z0:.10g})
    ({x1:.10g} {y0:.10g} {z0:.10g})
    ({x1:.10g} {y1:.10g} {z0:.10g})
    ({x0:.10g} {y1:.10g} {z0:.10g})
    ({x0:.10g} {y0:.10g} {z1:.10g})
    ({x1:.10g} {y0:.10g} {z1:.10g})
    ({x1:.10g} {y1:.10g} {z1:.10g})
    ({x0:.10g} {y1:.10g} {z1:.10g})
);

blocks
(
    hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1)
);

edges
(
);

boundary
(
    xmin
    {{
        type patch;
        faces ((0 4 7 3));
    }}
    xmax
    {{
        type patch;
        faces ((1 2 6 5));
    }}
    ymin
    {{
        type patch;
        faces ((0 1 5 4));
    }}
    ymax
    {{
        type patch;
        faces ((3 7 6 2));
    }}
    zmin
    {{
        type {{ZMIN_TYPE}};
        faces ((0 3 2 1));
    }}
    zmax
    {{
        type patch;
        faces ((4 5 6 7));
    }}
);

mergePatchPairs
(
);
'''


def _block_mesh_dict_for(
    domain_min: np.ndarray,
    domain_max: np.ndarray,
    project_area: float,
    resolution: int,
    ground: bool,
) -> str:
    text = _block_mesh_dict(domain_min, domain_max, project_area, resolution)
    return text.replace("{ZMIN_TYPE}", "wall" if ground else "patch")


def _snappy_hex_mesh_dict(location_in_mesh: np.ndarray, refinement_level: int) -> str:
    x, y, z = location_in_mesh
    coarse = max(int(refinement_level) - 1, 0)
    fine = max(int(refinement_level), coarse)
    return f'''{_foam_file("dictionary", "snappyHexMeshDict")}
#includeEtc "caseDicts/mesh/generation/snappyHexMeshDict.cfg"

castellatedMesh on;
snap            on;
addLayers       off;

geometry
{{
    body
    {{
        type triSurface;
        file "body.stl";
    }}
}};

castellatedMeshControls
{{
    maxLocalCells 400000;
    maxGlobalCells 4000000;
    minRefinementCells 0;
    maxLoadUnbalance 0.10;
    nCellsBetweenLevels 2;
    resolveFeatureAngle 30;
    allowFreeStandingZoneFaces true;
    extendedRefinementSpan true;

    features
    (
    );

    refinementSurfaces
    {{
        body
        {{
            level ({coarse} {fine});
            patchInfo
            {{
                type wall;
            }}
        }}
    }}

    refinementRegions {{ }};
    insidePoint ({x:.10g} {y:.10g} {z:.10g});
}}

snapControls
{{
    nSmoothPatch 3;
    tolerance 2.0;
    nSolveIter 100;
    nRelaxIter 5;

    nFeatureSnapIter 10;
    explicitFeatureSnap false;
    multiRegionFeatureSnap false;
    implicitFeatureSnap true;
}}

addLayersControls
{{
}}

meshQualityControls
{{
    maxNonOrtho 65;
    maxBoundarySkewness 20;
    maxInternalSkewness 4;
    maxConcave 80;
    minVol 1e-13;
    minTetQuality 1e-30;
    minTwist 0.02;
    minDeterminant 0.001;
    minFaceWeight 0.02;
    minVolRatio 0.01;
    minTriangleTwist -1;
    nSmoothScale 4;
    errorReduction 0.75;
}}

mergeTolerance 1e-6;
'''


def _control_dict(iterations: int) -> str:
    write_interval = max(int(iterations) // 4, 1)
    return f'''{_foam_file("dictionary", "controlDict")}
application     foamRun;

solver          incompressibleFluid;

startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         {int(iterations)};
deltaT          1;
writeControl    timeStep;
writeInterval   {write_interval};
purgeWrite      2;
writeFormat     ascii;
writePrecision  10;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable yes;

functions
{{
    #include "functions"
}}
'''


def _decompose_par_dict(n_processors: int) -> str:
    return f'''{_foam_file("dictionary", "decomposeParDict")}
numberOfSubdomains {int(n_processors)};

method          scotch;
'''


def _fv_schemes(turbulent: bool) -> str:
    turbulence_divs = ""
    if turbulent:
        turbulence_divs = """    div(phi,k)      bounded Gauss limitedLinear 1;
    div(phi,omega)  bounded Gauss limitedLinear 1;
"""
    return f'''{_foam_file("dictionary", "fvSchemes")}
ddtSchemes
{{
    default steadyState;
}}

gradSchemes
{{
    default Gauss linear;
}}

divSchemes
{{
    // A permissive default keeps the viscous stress term resolvable; the
    // convective terms that actually matter are pinned explicitly below.
    default         Gauss linear;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
{turbulence_divs}}}

laplacianSchemes
{{
    default Gauss linear corrected;
}}

interpolationSchemes
{{
    default linear;
}}

snGradSchemes
{{
    default corrected;
}}

wallDist
{{
    method meshWave;
}}

// ************************************************************************* //
'''


def _fv_solution(turbulent: bool) -> str:
    turbulence_solvers = ""
    turbulence_residuals = ""
    turbulence_relaxation = ""
    if turbulent:
        turbulence_solvers = """
    "(k|omega)"
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-8;
        relTol          0.1;
    }
"""
        turbulence_residuals = """        "(k|omega)"     1e-5;
"""
        turbulence_relaxation = """        "(k|omega)"     0.7;
"""
    return f'''{_foam_file("dictionary", "fvSolution")}
solvers
{{
    p
    {{
        solver          GAMG;
        tolerance       1e-7;
        relTol          0.01;
        smoother        DICGaussSeidel;
    }}

    U
    {{
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-8;
        relTol          0.1;
    }}
{turbulence_solvers}}}

SIMPLE
{{
    nNonOrthogonalCorrectors 0;
    residualControl
    {{
        p               1e-5;
        U               1e-6;
{turbulence_residuals}    }}
}}

relaxationFactors
{{
    fields
    {{
        p               0.3;
    }}
    equations
    {{
        U               0.7;
{turbulence_relaxation}    }}
}}

// ************************************************************************* //
'''


def _momentum_transport(turbulent: bool) -> str:
    if not turbulent:
        return f'''{_foam_file("dictionary", "momentumTransport")}
simulationType  laminar;

// ************************************************************************* //
'''
    return f'''{_foam_file("dictionary", "momentumTransport")}
simulationType  RAS;

RAS
{{
    model               kOmegaSST;
    turbulence          on;
    printCoeffs         on;
}}

// ************************************************************************* //
'''


def _physical_properties(density: float, kinematic_viscosity: float) -> str:
    return f'''{_foam_file("dictionary", "physicalProperties", location="constant")}
viscosityModel  constant;

rho             [1 -3 0 0 0 0 0] {_foam_scalar(density)};

nu              [0 2 -1 0 0 0 0] {_foam_scalar(kinematic_viscosity)};

// ************************************************************************* //
'''


def _farfield_patches() -> tuple[str, ...]:
    return ("xmin", "xmax", "ymin", "ymax", "zmax")


def _field_u(wind_vector: np.ndarray, ground: bool, moving_ground: bool) -> str:
    ux, uy, uz = np.asarray(wind_vector, dtype=float)
    free = f"({ux:.10g} {uy:.10g} {uz:.10g})"

    blocks = []
    for patch in _farfield_patches():
        blocks.append(
            f"""    {patch}
    {{
        type            freestreamVelocity;
        freestreamValue uniform {free};
        value           uniform {free};
    }}"""
        )

    if ground and moving_ground:
        # A road sliding underneath the body at free-stream speed: the usual
        # way to avoid growing a spurious floor boundary layer.
        zmin = f"""    zmin
    {{
        type            fixedValue;
        value           uniform {free};
    }}"""
    elif ground:
        zmin = """    zmin
    {
        type            noSlip;
    }"""
    else:
        zmin = f"""    zmin
    {{
        type            freestreamVelocity;
        freestreamValue uniform {free};
        value           uniform {free};
    }}"""
    blocks.append(zmin)

    blocks.append("""    body
    {
        type            noSlip;
    }""")

    body = "\n".join(blocks)
    return f'''{_foam_file("volVectorField", "U")}
internalField   uniform {free};

dimensions      [0 1 -1 0 0 0 0];

boundaryField
{{
{body}
    #includeEtc "caseDicts/setConstraintTypes"
}}

// ************************************************************************* //
'''


def _field_p(ground: bool) -> str:
    blocks = []
    for patch in _farfield_patches():
        blocks.append(
            f"""    {patch}
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}"""
        )
    if ground:
        blocks.append("""    zmin
    {
        type            zeroGradient;
    }""")
    else:
        blocks.append("""    zmin
    {
        type            freestreamPressure;
        freestreamValue uniform 0;
    }""")
    blocks.append("""    body
    {
        type            zeroGradient;
    }""")
    body = "\n".join(blocks)
    return f'''{_foam_file("volScalarField", "p")}
internalField   uniform 0;

dimensions      [0 2 -2 0 0 0 0];

boundaryField
{{
{body}
    #includeEtc "caseDicts/setConstraintTypes"
}}

// ************************************************************************* //
'''


def turbulence_seed(speed: float, reference_length: float) -> tuple[float, float, float]:
    """Free-stream k, omega and nut from intensity and a length scale."""
    speed = max(abs(float(speed)), 1e-6)
    length = max(abs(float(reference_length)), 1e-6)
    k = 1.5 * (speed * TURBULENCE_INTENSITY) ** 2
    length_scale = TURBULENCE_LENGTH_FRACTION * length
    omega = (k**0.5) / ((C_MU**0.25) * length_scale)
    nut = k / omega
    return float(k), float(omega), float(nut)


def _turbulence_field(
    name: str,
    class_name: str,
    dimensions: str,
    value: float,
    wall_type: str,
    ground: bool,
    ground_is_wall: bool,
) -> str:
    literal = _foam_scalar(value)
    blocks = []
    for patch in _farfield_patches():
        blocks.append(
            f"""    {patch}
    {{
        type            freestream;
        freestreamValue uniform {literal};
        value           uniform {literal};
    }}"""
        )

    if ground and ground_is_wall:
        blocks.append(
            f"""    zmin
    {{
        type            {wall_type};
        value           uniform {literal};
    }}"""
        )
    else:
        blocks.append(
            f"""    zmin
    {{
        type            freestream;
        freestreamValue uniform {literal};
        value           uniform {literal};
    }}"""
        )

    blocks.append(
        f"""    body
    {{
        type            {wall_type};
        value           uniform {literal};
    }}"""
    )

    body = "\n".join(blocks)
    return f'''{_foam_file(class_name, name)}
internalField   uniform {literal};

dimensions      {dimensions};

boundaryField
{{
{body}
    #includeEtc "caseDicts/setConstraintTypes"
}}

// ************************************************************************* //
'''


def _force_coeffs_incompressible(
    direction: np.ndarray,
    lift_dir: np.ndarray,
    pitch_axis: np.ndarray,
    mag_u_inf: float,
    rho_inf: float,
    l_ref: float,
    a_ref: float,
) -> str:
    return f'''{_foam_file("dictionary", "forceCoeffsIncompressible")}
patches         (body);

magUInf         {_foam_scalar(mag_u_inf)};
Aref            {_foam_scalar(a_ref)};
dragDir         {_vector_to_foam(direction)};
liftDir         {_vector_to_foam(lift_dir)};

lRef            {_foam_scalar(l_ref)};
CofR            (0 0 0);
pitchAxis       {_vector_to_foam(pitch_axis)};

#includeEtc "caseDicts/functions/forces/forceCoeffsIncompressible.cfg"

rho             rhoInf;
rhoInf          {_foam_scalar(rho_inf)};

// ************************************************************************* //
'''


@dataclass
class CoefficientHistory:
    """Coefficient traces read from a forceCoeffs.dat file."""

    times: np.ndarray
    columns: dict[str, np.ndarray]

    def averaged(self, name: str, fraction: float = 0.2) -> float | None:
        """Mean of the last ``fraction`` of the trace.

        Steady SIMPLE on a bluff body rarely settles to a fixed point -- the
        coefficient keeps oscillating by a few percent. Averaging the tail is
        far more representative than picking the final iteration.
        """
        series = self.columns.get(name)
        if series is None or len(series) == 0:
            return None
        count = max(int(len(series) * fraction), 1)
        tail = series[-count:]
        finite = tail[np.isfinite(tail)]
        if len(finite) == 0:
            return None
        return float(np.mean(finite))

    def spread(self, name: str, fraction: float = 0.2) -> float | None:
        """Peak-to-peak variation over the averaging window: a convergence hint."""
        series = self.columns.get(name)
        if series is None or len(series) == 0:
            return None
        count = max(int(len(series) * fraction), 1)
        tail = series[-count:]
        finite = tail[np.isfinite(tail)]
        if len(finite) < 2:
            return 0.0
        return float(np.max(finite) - np.min(finite))


def parse_force_coeffs_file(path: Path) -> CoefficientHistory:
    """Read a forceCoeffs.dat, mapping columns by their header names.

    The header looks like ``# Time  Cm  Cd  Cl  Cl(f)  Cl(r)``. Reading Cd by
    position is how this file got misread as Cm before, so everything here is
    keyed off the names.
    """
    text = path.read_text(encoding="utf-8", errors="ignore")
    names: list[str] = []
    rows: list[list[float]] = []

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            candidate = [token for token in re.split(r"\s{1,}|\t+", stripped.lstrip("# ").strip()) if token]
            # The column header is the last comment line that starts with Time.
            if candidate and candidate[0] == "Time":
                names = candidate
            continue
        tokens = stripped.split()
        try:
            rows.append([float(token) for token in tokens])
        except ValueError:
            continue

    if not rows:
        raise RuntimeError(f"No coefficient data in {path}")
    if not names:
        raise RuntimeError(f"No column header found in {path}; refusing to guess column order")

    width = min(len(names), min(len(row) for row in rows))
    table = np.array([row[:width] for row in rows], dtype=float)
    columns = {names[index]: table[:, index] for index in range(width)}
    return CoefficientHistory(times=columns.get("Time", table[:, 0]), columns=columns)


def find_force_coeffs_file(case_dir: Path) -> Path | None:
    root = case_dir / "postProcessing" / "forceCoeffsIncompressible"
    if not root.exists():
        root = case_dir / "postProcessing"
        if not root.exists():
            return None
    candidates = sorted(root.rglob("forceCoeffs*.dat"))
    if not candidates:
        candidates = sorted(root.rglob("coefficient*.dat"))
    return candidates[-1] if candidates else None


@dataclass
class OpenFOAMResult:
    drag_force: float
    drag_coefficient: float
    log_path: Path
    case_dir: Path
    lift_coefficient: float | None = None
    lift_force: float | None = None
    reference_area: float = 0.0
    reference_length: float = 0.0
    cd_spread: float | None = None
    converged: bool = False
    log_excerpt: str = ""
    settings: dict = field(default_factory=dict)


def prepare_openfoam_case(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float,
    viscosity: float,
    ground_offset: float | None,
    case_dir: Path,
    turbulent: bool = False,
    iterations: int = 400,
    mesh_resolution: int = 40,
    refinement_level: int = 3,
    moving_ground: bool = False,
    n_processors: int = 1,
    reference_area: float | None = None,
) -> dict:
    """Write a complete OpenFOAM case and return the reference quantities used."""
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "0").mkdir(exist_ok=True)
    (case_dir / "constant").mkdir(exist_ok=True)
    (case_dir / "system").mkdir(exist_ok=True)
    tri_surface_dir = case_dir / "constant" / "triSurface"
    tri_surface_dir.mkdir(parents=True, exist_ok=True)

    case_mesh = mesh.copy()
    if ground_offset is not None:
        translation = np.array(
            [0.0, 0.0, float(ground_offset) - float(np.min(case_mesh.vertices[:, 2]))], dtype=float
        )
        case_mesh.apply_translation(translation)

    extents = np.asarray(case_mesh.extents, dtype=float)
    ground = ground_offset is not None
    # Shared with the SU2 backend so both solvers see the same domain.
    domain_min, domain_max = flow_domain(case_mesh, ground=ground)

    location_in_mesh = domain_min + 0.85 * (domain_max - domain_min)
    drag_dir, lift_dir, pitch_axis = orthonormal_basis(wind_vector)
    speed = float(np.linalg.norm(wind_vector))
    if speed < 1e-12:
        raise ValueError("Wind vector must be non-zero")

    if reference_area is None:
        reference_area = frontal_area(case_mesh, wind_vector)
    reference_length = float(np.max(extents))
    kinematic_viscosity = viscosity / density

    case_mesh.export(tri_surface_dir / "body.stl")
    (case_dir / "system" / "blockMeshDict").write_text(
        _block_mesh_dict_for(domain_min, domain_max, reference_area, mesh_resolution, ground), encoding="utf-8"
    )
    (case_dir / "system" / "snappyHexMeshDict").write_text(
        _snappy_hex_mesh_dict(location_in_mesh, refinement_level), encoding="utf-8"
    )
    (case_dir / "system" / "controlDict").write_text(_control_dict(iterations), encoding="utf-8")
    (case_dir / "system" / "fvSchemes").write_text(_fv_schemes(turbulent), encoding="utf-8")
    (case_dir / "system" / "fvSolution").write_text(_fv_solution(turbulent), encoding="utf-8")
    (case_dir / "system" / "meshQualityDict").write_text(
        f'{_foam_file("dictionary", "meshQualityDict")}\n#includeEtc "caseDicts/mesh/generation/meshQualityDict.cfg"\n',
        encoding="utf-8",
    )
    if n_processors > 1:
        (case_dir / "system" / "decomposeParDict").write_text(_decompose_par_dict(n_processors), encoding="utf-8")

    (case_dir / "constant" / "momentumTransport").write_text(_momentum_transport(turbulent), encoding="utf-8")
    (case_dir / "constant" / "physicalProperties").write_text(
        _physical_properties(density, kinematic_viscosity), encoding="utf-8"
    )
    (case_dir / "0" / "U").write_text(_field_u(wind_vector, ground, moving_ground), encoding="utf-8")
    (case_dir / "0" / "p").write_text(_field_p(ground), encoding="utf-8")

    if turbulent:
        k_value, omega_value, nut_value = turbulence_seed(speed, reference_length)
        ground_is_wall = ground and not moving_ground
        (case_dir / "0" / "k").write_text(
            _turbulence_field("k", "volScalarField", "[0 2 -2 0 0 0 0]", k_value, "kqRWallFunction", ground, ground_is_wall),
            encoding="utf-8",
        )
        (case_dir / "0" / "omega").write_text(
            _turbulence_field("omega", "volScalarField", "[0 0 -1 0 0 0 0]", omega_value, "omegaWallFunction", ground, ground_is_wall),
            encoding="utf-8",
        )
        (case_dir / "0" / "nut").write_text(
            _turbulence_field("nut", "volScalarField", "[0 2 -1 0 0 0 0]", nut_value, "nutkWallFunction", ground, ground_is_wall),
            encoding="utf-8",
        )

    (case_dir / "system" / "functions").write_text("#includeFunc forceCoeffsIncompressible\n", encoding="utf-8")
    (case_dir / "system" / "forceCoeffsIncompressible").write_text(
        _force_coeffs_incompressible(
            drag_dir, lift_dir, pitch_axis, speed, density, reference_length, reference_area
        ),
        encoding="utf-8",
    )

    return {
        "drag_dir": drag_dir,
        "lift_dir": lift_dir,
        "pitch_axis": pitch_axis,
        "reference_area": float(reference_area),
        "reference_length": reference_length,
        "domain_min": domain_min.tolist(),
        "domain_max": domain_max.tolist(),
        "cells": list(_block_cell_counts(domain_min, domain_max, mesh_resolution)),
        "speed": speed,
    }


def _run_openfoam_commands(case_dir: Path, n_processors: int = 1) -> Path:
    """Mesh and solve. Meshing stays serial; only the solve is decomposed."""
    if n_processors > 1:
        solve = (
            "decomposePar -force > log.decomposePar 2>&1 ; "
            f"mpirun -np {int(n_processors)} --allow-run-as-root foamRun -solver incompressibleFluid -parallel > log.foamRun 2>&1 ; "
            "reconstructPar -latestTime > log.reconstructPar 2>&1"
        )
    else:
        solve = "foamRun -solver incompressibleFluid > log.foamRun 2>&1"

    commands = " ; ".join(
        [
            "blockMesh > log.blockMesh 2>&1",
            "snappyHexMesh -overwrite > log.snappyHexMesh 2>&1",
            solve,
        ]
    )
    run_wsl_bash(commands, cwd=case_dir, capture_output=False, check=False)

    log_path = case_dir / "log.foamRun"
    if not log_path.exists():
        for name in ("log.snappyHexMesh", "log.blockMesh"):
            candidate = case_dir / name
            if candidate.exists():
                raise RuntimeError(
                    f"OpenFOAM stopped before the solve. Tail of {name}:\n"
                    + "\n".join(candidate.read_text(encoding="utf-8", errors="ignore").splitlines()[-25:])
                )
        raise RuntimeError("OpenFOAM produced no logs at all; is the WSL distro running?")
    return log_path


def _log_excerpt(path: Path, lines: int = 25) -> str:
    if not path.exists():
        return ""
    return "\n".join(path.read_text(encoding="utf-8", errors="ignore").splitlines()[-lines:])


def run_openfoam_drag(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float = 1.225,
    viscosity: float = 1.8e-5,
    ground_offset: float | None = None,
    work_dir: str | Path | None = None,
    keep_case: bool = False,
    turbulent: bool = False,
    iterations: int = 400,
    mesh_resolution: int = 40,
    refinement_level: int = 3,
    moving_ground: bool = False,
    n_processors: int = 1,
    reference_area: float | None = None,
) -> OpenFOAMResult:
    if not openfoam_available():
        raise RuntimeError(
            f"OpenFOAM 13 is not reachable ({OPENFOAM_BASHRC} in WSL {WSL_DISTRO})"
        )

    wind_vector = np.asarray(wind_vector, dtype=float)
    if work_dir is None:
        case_dir = Path(tempfile.mkdtemp(prefix="openfoam_case_"))
    else:
        case_dir = Path(work_dir)
        case_dir.mkdir(parents=True, exist_ok=True)

    setup = prepare_openfoam_case(
        mesh,
        wind_vector,
        density,
        viscosity,
        ground_offset,
        case_dir,
        turbulent=turbulent,
        iterations=iterations,
        mesh_resolution=mesh_resolution,
        refinement_level=refinement_level,
        moving_ground=moving_ground,
        n_processors=n_processors,
        reference_area=reference_area,
    )
    log_path = _run_openfoam_commands(case_dir, n_processors=n_processors)

    coefficients_file = find_force_coeffs_file(case_dir)
    if coefficients_file is None:
        raise RuntimeError(
            "OpenFOAM wrote no force coefficients. Tail of the solver log:\n" + _log_excerpt(log_path)
        )

    history = parse_force_coeffs_file(coefficients_file)
    drag_coefficient = history.averaged("Cd")
    if drag_coefficient is None:
        raise RuntimeError(
            f"No Cd column in {coefficients_file.name} (columns: {sorted(history.columns)})"
        )
    lift_coefficient = history.averaged("Cl")
    cd_spread = history.spread("Cd")

    speed = float(setup["speed"])
    area = float(setup["reference_area"])
    dynamic_pressure = 0.5 * density * speed * speed
    drag_force = dynamic_pressure * area * drag_coefficient
    lift_force = None if lift_coefficient is None else dynamic_pressure * area * lift_coefficient

    # A tail spread under a few percent of the mean is as converged as a
    # steady solve on a bluff body usually gets.
    converged = bool(
        cd_spread is not None and abs(drag_coefficient) > 1e-9 and cd_spread / abs(drag_coefficient) < 0.05
    )

    result = OpenFOAMResult(
        drag_force=float(drag_force),
        drag_coefficient=float(drag_coefficient),
        log_path=log_path,
        case_dir=case_dir,
        lift_coefficient=None if lift_coefficient is None else float(lift_coefficient),
        lift_force=None if lift_force is None else float(lift_force),
        reference_area=area,
        reference_length=float(setup["reference_length"]),
        cd_spread=cd_spread,
        converged=converged,
        log_excerpt=_log_excerpt(log_path),
        settings={
            "turbulence": "kOmegaSST" if turbulent else "laminar",
            "iterations": iterations,
            "mesh_resolution": mesh_resolution,
            "refinement_level": refinement_level,
            "moving_ground": moving_ground,
            "n_processors": n_processors,
            "background_cells": setup["cells"],
        },
    )

    if not keep_case:
        shutil.rmtree(case_dir, ignore_errors=True)

    return result
