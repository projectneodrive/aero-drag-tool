import os
import subprocess
import numpy as np
import pyvista as pv

# ============================================================
# USER INPUT
# ============================================================

L = 1.0          # cube side length [m]
V = 20.0         # velocity [m/s]
rho = 1.225
mu = 1.8e-5

mesh_file = "cube.su2"
cfg_file = "cube.cfg"
geo_file = "cube.geo"

# ============================================================
# AUTOMATIC PHYSICS-BASED PARAMETER COMPUTATION
# ============================================================

Re = rho * V * L / mu

# Skin friction coefficient estimate
Cf = 0.026 / (Re ** (1/7))

# friction velocity
u_tau = np.sqrt(Cf / 2) * V

# first boundary layer height (y+ = 1)
y_plus = 1.0
y1 = y_plus * mu / (rho * u_tau)

# boundary layer thickness estimate
delta = 0.37 * L / (Re ** 0.2)

# mesh sizes
surface_size = L / 25
farfield_size = L / 8

# domain size
upstream = 5 * L
downstream = 15 * L
side = 5 * L

# safe CFL for implicit solver
CFL = min(5.0, farfield_size / L * 10)

print("\n--- Automatic parameters ---")
print("Re =", Re)
print("y1 =", y1)
print("delta =", delta)
print("surface_size =", surface_size)
print("farfield_size =", farfield_size)
print("CFL =", CFL)

# ============================================================
# GENERATE MESH WITH GMSH (same structure as your original)
# ============================================================

if not os.path.exists(mesh_file):

    geo = f"""
    SetFactory("OpenCASCADE");

    Box(1) = {{{-upstream},{-side},{-side}, {upstream+downstream},{2*side},{2*side}}};
    Box(2) = {{-0.5,-0.5,-0.5, 1,1,1}};

    BooleanDifference{{ Volume{{1}}; Delete; }}{{ Volume{{2}}; }}

    Physical Volume("fluid") = {{1}};
    Physical Surface("cube") = {{7,8,9,10,11,12}};
    Physical Surface("farfield") = {{1,2,3,4,5,6}};

    Field[1] = Distance;
    Field[1].SurfacesList = {{7,8,9,10,11,12}};

    Field[2] = Threshold;
    Field[2].InField = 1;
    Field[2].SizeMin = {surface_size};
    Field[2].SizeMax = {farfield_size};
    Field[2].DistMin = {L};
    Field[2].DistMax = {5*L};

    BoundaryLayer Field = 3;
    Field[3].SurfacesList = {{7,8,9,10,11,12}};
    Field[3].Size = {y1};
    Field[3].Ratio = 1.2;
    Field[3].Thickness = {delta};

    Background Field = 2;
    """

    with open(geo_file, "w") as f:
        f.write(geo)

    subprocess.run([
        "gmsh",
        geo_file,
        "-3",
        "-format", "su2",
        "-o", mesh_file
    ], check=True)

else:
    print("Mesh exists, skipping generation.")

# ============================================================
# CREATE SU2 CONFIG (same as your original, only dynamic values)
# ============================================================

cfg = f"""
SOLVER= INC_RANS

KIND_TURB_MODEL= SST
MATH_PROBLEM= DIRECT
RESTART_SOL= NO

MACH_NUMBER= 0.0
AOA= 0.0
SIDESLIP_ANGLE= 0.0

FREESTREAM_TEMPERATURE= 288.0
FREESTREAM_PRESSURE= 101325.0
FREESTREAM_DENSITY= {rho}
FREESTREAM_VELOCITY= ({V}, 0.0, 0.0)

FLUID_MODEL= STANDARD_AIR

VISCOSITY_MODEL= CONSTANT_VISCOSITY
MU_CONSTANT= {mu}

REYNOLDS_NUMBER= {Re}
REYNOLDS_LENGTH= {L}

REF_ORIGIN_MOMENT_X= 0.0
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0

REF_AREA= {L*L}
REF_LENGTH= {L}

MESH_FILENAME= {mesh_file}
MESH_FORMAT= SU2

MARKER_FAR= (farfield)
MARKER_HEATFLUX= (cube, 0.0)

MARKER_MONITORING= (cube)
MARKER_PLOTTING= (cube)
MARKER_ANALYZE= (cube)

NUM_METHOD_GRAD= GREEN_GAUSS

CONV_NUM_METHOD_FLOW= FDS
SLOPE_LIMITER_FLOW= VENKATAKRISHNAN

CONV_NUM_METHOD_TURB= FDS

TIME_DISCRE_FLOW= EULER_IMPLICIT
TIME_DISCRE_TURB= EULER_IMPLICIT

LINEAR_SOLVER= FGMRES
LINEAR_SOLVER_PREC= ILU
LINEAR_SOLVER_ERROR= 1E-6
LINEAR_SOLVER_ITER= 10

CFL_NUMBER= {CFL}
CFL_ADAPT= YES
CFL_ADAPT_PARAM= (0.5, 1.5, 1.0, 50.0)

ITER= 10

CONV_RESIDUAL_MINVAL= -10
CONV_STARTITER= 10

OUTPUT_FILES= (RESTART, PARAVIEW_ASCII, SURFACE_PARAVIEW, SURFACE_CSV)

RESTART_FILENAME= restart
VOLUME_FILENAME= flow
SURFACE_FILENAME= surface
CONV_FILENAME= history

HISTORY_OUTPUT= (ITER, RESIDUALS, AERO_COEFF, DRAG, LIFT, RMS_RES, CD, CL, CMZ, )
"""

with open(cfg_file, "w") as f:
    f.write(cfg)

# ============================================================
# RUN SOLVER (UNCHANGED FROM YOUR WORKING VERSION)
# ============================================================

subprocess.run(["mpirun", "-np", "16", "SU2_CFD", cfg_file], check=True)

#subprocess.run(["SU2_CFD", cfg_file], check=True)
s