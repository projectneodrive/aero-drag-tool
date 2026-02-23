import os
import subprocess
import numpy as np
import pyvista as pv
import numpy as np

resolution = 0.3
V = 20.0
rho = 1.225
L = 1.0

mesh_file = "cube.su2"
cfg_file = "cube.cfg"

# =========================
# 1. Generate mesh with Gmsh only if missing
# =========================
if not os.path.exists(mesh_file):
    geo = f"""
    SetFactory("OpenCASCADE");
    lc = {resolution};

    Box(1) = {{-5,-5,0, 15,10,6}};
    Box(2) = {{-0.5,-0.5,1, 1,1,1}};
    BooleanDifference{{ Volume{{1}}; Delete; }}{{ Volume{{2}}; }}

    Physical Volume("fluid") = {{1}};
    Physical Surface("cube") = {{7,8,9,10,11,12}};
    Physical Surface("ground") = {{1}};
    Physical Surface("farfield") = {{2,3,4,5,6}};

    Mesh.CharacteristicLengthMin = lc;
    Mesh.CharacteristicLengthMax = lc;
    """
    with open("cube.geo","w") as f:
        f.write(geo)
    subprocess.run(["gmsh","cube.geo","-3","-format","su2","-o",mesh_file],check=True)
else:
    print(f"Mesh file '{mesh_file}' already exists, skipping mesh generation.")

# =========================
# 2. Create SU2 config
# =========================
cfg = f"""
% ---------------------------- Solver ----------------------------
SOLVER= RANS
KIND_TURB_MODEL= SST
MATH_PROBLEM= DIRECT
RESTART_SOL= NO

% ---------------------- Freestream ------------------------------
MACH_NUMBER= 0.058
AOA= 0.0
SIDESLIP_ANGLE= 0.0

FREESTREAM_TEMPERATURE= 288.0
FREESTREAM_PRESSURE= 101325.0
FREESTREAM_DENSITY= {rho}
FREESTREAM_VELOCITY= ({V}, 0.0, 0.0)

% ---------------------- Fluid model -----------------------------
FLUID_MODEL= STANDARD_AIR

VISCOSITY_MODEL= CONSTANT_VISCOSITY
MU_CONSTANT= 1.8E-5

REYNOLDS_NUMBER= 1.36E6
REYNOLDS_LENGTH= 1.0

% ---------------------- Reference values ------------------------
REF_ORIGIN_MOMENT_X= 0.0
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0

REF_AREA= 1.0
REF_LENGTH= 1.0

% ---------------------- Mesh ------------------------
MESH_FILENAME= {mesh_file}
MESH_FORMAT= SU2

% ---------------------- Boundary conditions ---------------------
MARKER_FAR= (farfield)
MARKER_HEATFLUX= (cube, 0.0), (ground, 0.0)

MARKER_MONITORING= (cube)
MARKER_PLOTTING= (cube)

% ---------------------- Numerics ------------------------
NUM_METHOD_GRAD= GREEN_GAUSS

CONV_NUM_METHOD_FLOW= ROE
SLOPE_LIMITER_FLOW= VENKATAKRISHNAN

CONV_NUM_METHOD_TURB= SCALAR_UPWIND
SLOPE_LIMITER_TURB= VENKATAKRISHNAN

TIME_DISCRE_FLOW= EULER_IMPLICIT
TIME_DISCRE_TURB= EULER_IMPLICIT

LINEAR_SOLVER= FGMRES
LINEAR_SOLVER_PREC= ILU
LINEAR_SOLVER_ERROR= 1E-6
LINEAR_SOLVER_ITER= 10

% ---------------------- CFL ------------------------
CFL_NUMBER= 5.0
CFL_ADAPT= YES
CFL_ADAPT_PARAM= (0.1, 1.2, 10.0, 100.0)

% ---------------------- Convergence ------------------------
ITER= 2

CONV_RESIDUAL_MINVAL= -12
CONV_STARTITER= 10

% ---------------------- Output ------------------------
OUTPUT_FILES= (RESTART, PARAVIEW_ASCII, SURFACE_PARAVIEW, SURFACE_CSV)


RESTART_FILENAME= restart
VOLUME_FILENAME= flow
SURFACE_FILENAME= surface
CONV_FILENAME= history

HISTORY_OUTPUT= (ITER, RMS_RES, CD, CL, CMZ)

"""
with open(cfg_file,"w") as f:
    f.write(cfg)

# =========================
# 3. Run solver
# =========================
# Use single process without mpirun first to isolate MPI errors
subprocess.run(["SU2_CFD", cfg_file], check=True)

# =========================
# 4. Extract drag
# =========================

# Load surface VTU
surf = pv.read("surface.vtu")  # UnstructuredGrid

# Pressure at points
p = surf.point_data["Pressure"]

# Node coordinates
points = surf.points

# Cell connectivity
cells = surf.cells
celltypes = surf.celltypes

# Initialize force
F = np.zeros(3)

# VTK_TRIANGLE = 5
for i, ct in enumerate(celltypes):
    if ct != 5:  # skip non-triangle cells
        continue

    # Extract indices of triangle vertices
    # In VTK unstructured grid, surf.offset[i] gives start index in surf.cells
    # Each triangle has 3 vertices; surf.cells contains [n, i0, i1, i2,...]
    offset = surf.offset[i]
    tri = cells[offset+1:offset+4]  # skip the first element (number of points)

    verts = points[tri]
    tri_p = p[tri]
    p_avg = np.mean(tri_p)

    # Compute area vector (normal * area)
    v0, v1, v2 = verts
    nA = 0.5 * np.cross(v1 - v0, v2 - v0)

    # Add contribution (pressure * area)
    F += -p_avg * nA

print("Force vector [Nx, Ny, Nz] =", F)
Drag = F[0]  # assuming flow in X
Lift = F[2]  # assuming Z-up
print("Drag:", Drag, "N")
print("Lift:", Lift, "N")