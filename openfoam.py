from __future__ import annotations

import math
import os
import re
import shlex
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh


WSL_DISTRO = "Ubuntu-22.04"
OPENFOAM_BASHRC = "/opt/openfoam13/etc/bashrc"


def _normalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("Vector must be non-zero")
    return vector / norm


def openfoam_available() -> bool:
    result = subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-u", "root", "-e", "bash", "-lc", f"test -f {shlex.quote(OPENFOAM_BASHRC)}"],
        capture_output=True,
        text=True,
    )
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


def run_wsl_bash(script: str, cwd: str | Path | None = None, capture_output: bool = False) -> subprocess.CompletedProcess[str]:
    command = f"source {shlex.quote(OPENFOAM_BASHRC)}"
    if cwd is not None:
        command += f" && cd {shlex.quote(windows_to_wsl_path(cwd))}"
    command += f" && {script}"
    return subprocess.run(
        ["wsl", "-d", WSL_DISTRO, "-u", "root", "-e", "bash", "-lc", command],
        check=True,
        capture_output=capture_output,
        text=True,
    )


def projected_area(mesh: trimesh.Trimesh, wind_vector: np.ndarray) -> float:
    direction = _normalize(wind_vector)
    normals = np.asarray(mesh.face_normals, dtype=float)
    areas = np.asarray(mesh.area_faces, dtype=float)
    facing = np.clip(normals @ direction, 0.0, None)
    return float(np.sum(areas * facing))


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


def _block_mesh_dict(domain_min: np.ndarray, domain_max: np.ndarray, project_area: float) -> str:
    x0, y0, z0 = domain_min
    x1, y1, z1 = domain_max
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}}

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
    hex (0 1 2 3 4 5 6 7) (40 40 40) simpleGrading (1 1 1)
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
        type patch;
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


def _snappy_hex_mesh_dict(location_in_mesh: np.ndarray) -> str:
    x, y, z = location_in_mesh
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       dictionary;
    object      snappyHexMeshDict;
}}

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
    maxLocalCells 200000;
    maxGlobalCells 2000000;
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
            level (2 3);
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


def _control_dict() -> str:
    return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      controlDict;
}

application     simpleFoam;

startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         200;
deltaT          1;
writeControl    timeStep;
writeInterval   50;
purgeWrite      0;
writeFormat     ascii;
writePrecision  10;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable yes;

functions
{
    #include "functions"
}
'''


def _fv_schemes() -> str:
    return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      fvSchemes;
}

ddtSchemes
{
    default steadyState;
}

gradSchemes
{
    default Gauss linear;
}

divSchemes
{
    default none;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
}

laplacianSchemes
{
    default Gauss linear corrected;
}

interpolationSchemes
{
    default linear;
}

snGradSchemes
{
    default corrected;
}

// ************************************************************************* //
'''


def _fv_solution() -> str:
    return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      fvSolution;
}

solvers
{
    p
    {
        solver          GAMG;
        tolerance       1e-7;
        relTol          0.01;
        smoother        DICGaussSeidel;
    }

    U
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-8;
        relTol          0.1;
    }
}

SIMPLE
{
    nNonOrthogonalCorrectors 0;
    residualControl
    {
        p               1e-5;
        U               1e-6;
    }
}

relaxationFactors
{
    fields
    {
        p               0.3;
    }
    equations
    {
        U               0.7;
    }
}

// ************************************************************************* //
'''


def _momentum_transport(laminar: bool = True) -> str:
    if laminar:
        return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      momentumTransport;
}

simulationType  laminar;

// ************************************************************************* //
'''
    return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O n;?           | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      momentumTransport;
}

simulationType  RAS;

RAS
{
    model               kOmegaSST;
    turbulence          on;
}

// ************************************************************************* //
'''


def _physical_properties(density: float, kinematic_viscosity: float) -> str:
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       dictionary;
    location    "constant";
    object      physicalProperties;
}}

viscosityModel  constant;

rho             [1 -3 0 0 0 0 0] {_foam_scalar(density)};

nu              [0 2 -1 0 0 0 0] {_foam_scalar(kinematic_viscosity)};

// ************************************************************************* //
'''


def _field_u(wind_vector: np.ndarray, ground: bool) -> str:
    ux, uy, uz = np.asarray(wind_vector, dtype=float)
    wall_block = '''
    ground
    {
        type            noSlip;
    }
''' if ground else ''
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       volVectorField;
    object      U;
}}

internalField   uniform ({ux:.10g} {uy:.10g} {uz:.10g});

dimensions      [0 1 -1 0 0 0 0];

boundaryField
{{
    xmin
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10g} {uy:.10g} {uz:.10g});
        value           uniform ({ux:.10g} {uy:.10g} {uz:.10g});
    }}
    xmax
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10g} {uy:.10g} {uz:.10g});
        value           uniform ({ux:.10g} {uy:.10g} {uz:.10g});
    }}
    ymin
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10g} {uy:.10g} {uz:.10g});
        value           uniform ({ux:.10g} {uy:.10g} {uz:.10g});
    }}
    ymax
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10g} {uy:.10g} {uz:.10g});
        value           uniform ({ux:.10g} {uy:.10g} {uz:.10g});
    }}
    zmin
    {{
        type            {'noSlip' if ground else 'freestreamVelocity'};
{('        freestreamValue uniform (' + f'{ux:.10g} {uy:.10g} {uz:.10g}' + ');\n        value           uniform (' + f'{ux:.10g} {uy:.10g} {uz:.10g}' + ');') if not ground else ''}
    }}
    zmax
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10g} {uy:.10g} {uz:.10g});
        value           uniform ({ux:.10g} {uy:.10g} {uz:.10g});
    }}
    body
    {{
        type            noSlip;
    }}
    #includeEtc "caseDicts/setConstraintTypes"
}}

// ************************************************************************* //
'''


def _field_p(ground: bool) -> str:
    ground_block = '''
    ground
    {
        type            zeroGradient;
    }
''' if ground else ''
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       volScalarField;
    object      p;
}}

internalField   uniform 0;

dimensions      [0 2 -2 0 0 0 0];

boundaryField
{{
    xmin
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}
    xmax
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}
    ymin
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}
    ymax
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}
    zmin
    {{
        type            {'zeroGradient' if ground else 'freestreamPressure'};
{('        freestreamValue uniform 0;') if not ground else ''}
    }}
    zmax
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}
    body
    {{
        type            zeroGradient;
    }}
    #includeEtc "caseDicts/setConstraintTypes"
}}

// ************************************************************************* //
'''


def _mesh_quality_dict() -> str:
    return '''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{
    format      ascii;
    class       dictionary;
    object      meshQualityDict;
}

#includeEtc "caseDicts/mesh/generation/meshQualityDict.cfg"

// ************************************************************************* //
'''


def _force_coeffs_incompressible(direction: np.ndarray, lift_dir: np.ndarray, pitch_axis: np.ndarray, mag_u_inf: float, rho_inf: float, l_ref: float, a_ref: float) -> str:
    return f'''/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Website:  https://openfoam.org
    \\  /    A nd           | Version:  13
     \\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       dictionary;
    object      forceCoeffsIncompressible;
}}

patches         (body);

magUInf         {_foam_scalar(mag_u_inf)};
Aref            {_foam_scalar(a_ref)};
dragDir         {_vector_to_foam(direction)};
liftDir         {_vector_to_foam(lift_dir)};

// Moment calculation parameters
lRef            {_foam_scalar(l_ref)};
CofR            (0 0 0);
pitchAxis       {_vector_to_foam(pitch_axis)};

#includeEtc "caseDicts/functions/forces/forceCoeffsIncompressible.cfg"

rho             rhoInf;
rhoInf          {_foam_scalar(rho_inf)};

// ************************************************************************* //
'''


def _detect_drag_coefficient(log_text: str) -> float:
    matches = re.findall(r"\bCd:\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)", log_text)
    if not matches:
        raise RuntimeError("Could not find Cd in OpenFOAM log output")
    return float(matches[-1])


def _parse_force_coeffs_file(path: Path) -> float | None:
    if not path.exists():
        return None
    lines = [line.strip() for line in path.read_text(encoding="utf-8", errors="ignore").splitlines() if line.strip() and not line.startswith("#")]
    if not lines:
        return None
    tokens = re.findall(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", lines[-1])
    if len(tokens) < 2:
        return None
    return float(tokens[1])


@dataclass
class OpenFOAMResult:
    drag_force: float
    drag_coefficient: float
    log_path: Path
    case_dir: Path


def prepare_openfoam_case(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float,
    viscosity: float,
    ground_offset: float | None,
    case_dir: Path,
) -> tuple[Path, np.ndarray, np.ndarray, np.ndarray, float, float]:
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "0").mkdir(exist_ok=True)
    (case_dir / "constant").mkdir(exist_ok=True)
    (case_dir / "system").mkdir(exist_ok=True)
    tri_surface_dir = case_dir / "constant" / "triSurface"
    tri_surface_dir.mkdir(parents=True, exist_ok=True)

    case_mesh = mesh.copy()
    if ground_offset is not None:
        translation = np.array([0.0, 0.0, float(ground_offset) - float(np.min(case_mesh.vertices[:, 2]))], dtype=float)
        case_mesh.apply_translation(translation)

    bounds = np.asarray(case_mesh.bounds, dtype=float)
    extents = np.asarray(case_mesh.extents, dtype=float)
    padding = np.maximum(extents * 4.0, np.array([4.0, 4.0, 4.0], dtype=float))
    domain_min = bounds[0] - padding
    domain_max = bounds[1] + padding
    if ground_offset is not None:
        domain_min[2] = 0.0
        domain_max[2] = max(domain_max[2], float(np.max(case_mesh.vertices[:, 2])) + padding[2])

    location_in_mesh = domain_min + 0.85 * (domain_max - domain_min)
    drag_dir, lift_dir, pitch_axis = orthonormal_basis(wind_vector)
    speed = float(np.linalg.norm(wind_vector))
    if speed < 1e-12:
        raise ValueError("Wind vector must be non-zero")

    reference_area = projected_area(case_mesh, wind_vector)
    reference_length = float(np.max(extents))
    kinematic_viscosity = viscosity / density

    case_mesh.export(tri_surface_dir / "body.stl")
    (case_dir / "system" / "blockMeshDict").write_text(_block_mesh_dict(domain_min, domain_max, reference_area), encoding="utf-8")
    (case_dir / "system" / "snappyHexMeshDict").write_text(_snappy_hex_mesh_dict(location_in_mesh), encoding="utf-8")
    (case_dir / "system" / "controlDict").write_text(_control_dict(), encoding="utf-8")
    (case_dir / "system" / "fvSchemes").write_text(_fv_schemes(), encoding="utf-8")
    (case_dir / "system" / "fvSolution").write_text(_fv_solution(), encoding="utf-8")
    (case_dir / "system" / "meshQualityDict").write_text(_mesh_quality_dict(), encoding="utf-8")
    (case_dir / "constant" / "momentumTransport").write_text(_momentum_transport(laminar=True), encoding="utf-8")
    (case_dir / "constant" / "physicalProperties").write_text(_physical_properties(density, kinematic_viscosity), encoding="utf-8")
    (case_dir / "0" / "U").write_text(_field_u(wind_vector, ground_offset is not None), encoding="utf-8")
    (case_dir / "0" / "p").write_text(_field_p(ground_offset is not None), encoding="utf-8")
    (case_dir / "system" / "functions").write_text("#includeFunc forceCoeffsIncompressible\n", encoding="utf-8")
    (case_dir / "system" / "forceCoeffsIncompressible").write_text(
        _force_coeffs_incompressible(drag_dir, lift_dir, pitch_axis, speed, density, reference_length, reference_area),
        encoding="utf-8",
    )
    return case_dir, drag_dir, lift_dir, pitch_axis, reference_area, reference_length


def _run_openfoam_commands(case_dir: Path) -> Path:
    log_path = case_dir / "log.simpleFoam"
    commands = " ; ".join(
        [
            "blockMesh > log.blockMesh 2>&1",
            "snappyHexMesh -overwrite > log.snappyHexMesh 2>&1",
            "simpleFoam > log.simpleFoam 2>&1",
            "postProcess -func forceCoeffsIncompressible -latestTime > log.postProcess 2>&1",
        ]
    )
    run_wsl_bash(commands, cwd=case_dir, capture_output=False)
    if not log_path.exists():
        raise RuntimeError("OpenFOAM did not produce a simpleFoam log")
    return log_path


def run_openfoam_drag(
    mesh: trimesh.Trimesh,
    wind_vector: np.ndarray,
    density: float = 1.225,
    viscosity: float = 1.8e-5,
    ground_offset: float | None = None,
    work_dir: str | Path | None = None,
    keep_case: bool = False,
) -> OpenFOAMResult:
    if not openfoam_available():
        raise RuntimeError("OpenFOAM v13 is not available in WSL Ubuntu-22.04")

    wind_vector = np.asarray(wind_vector, dtype=float)
    if work_dir is None:
        case_dir = Path(tempfile.mkdtemp(prefix="openfoam_case_"))
    else:
        case_dir = Path(work_dir)
        case_dir.mkdir(parents=True, exist_ok=True)

    prepare_openfoam_case(mesh, wind_vector, density, viscosity, ground_offset, case_dir)
    log_path = _run_openfoam_commands(case_dir)
    log_text = log_path.read_text(encoding="utf-8", errors="ignore")
    drag_coefficient = None
    try:
        drag_coefficient = _detect_drag_coefficient(log_text)
    except RuntimeError:
        coeff_file = next((case_dir / "postProcessing").rglob("forceCoeffs*.dat"), None) if (case_dir / "postProcessing").exists() else None
        if coeff_file is not None:
            drag_coefficient = _parse_force_coeffs_file(coeff_file)
    if drag_coefficient is None:
        raise RuntimeError("Unable to parse drag coefficient from OpenFOAM output")

    speed = float(np.linalg.norm(wind_vector))
    reference_area = projected_area(mesh if ground_offset is None else mesh.copy(), wind_vector)
    drag_force = 0.5 * density * speed * speed * reference_area * drag_coefficient
    result = OpenFOAMResult(drag_force=float(drag_force), drag_coefficient=float(drag_coefficient), log_path=log_path, case_dir=case_dir)

    if not keep_case:
        # Keep the log file path for debugging if the caller wants it, but remove the heavy case tree by default.
        shutil.rmtree(case_dir, ignore_errors=True)

    return result