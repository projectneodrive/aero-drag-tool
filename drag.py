import os
import subprocess
import numpy as np

# =========================
# USER PARAMETERS
# =========================

resolution = 0.1   # mesh resolution (m)
V = 20.0           # air speed m/s
rho = 1.225
L = 1.0

mesh_file = "cube.su2"
cfg_file = "cube.cfg"

# =========================
# 1. Generate mesh with Gmsh
# =========================

geo = f"""
SetFactory("OpenCASCADE");

lc = {resolution};

// domain
Box(1) = {{-5,-5,0, 15,10,6}};

// cube
Box(2) = {{-0.5,-0.5,1, 1,1,1}};

// subtract cube from domain
BooleanDifference{{ Volume{{1}}; Delete; }}{{ Volume{{2}}; }}

// physical groups
Physical Volume("fluid") = {{1}};
Physical Surface("cube") = {{7,8,9,10,11,12}};
Physical Surface("ground") = {{1}};
Physical Surface("farfield") = {{2,3,4,5,6}};

// mesh resolution
Mesh.CharacteristicLengthMin = lc;
Mesh.CharacteristicLengthMax = lc;
"""

with open("cube.geo","w") as f:
    f.write(geo)

# generate mesh
subprocess.run(["gmsh","cube.geo","-3","-format","su2","-o",mesh_file],check=True)

# =========================
# 2. Create SU2 config
# =========================

cfg = f"""
SOLVER= NAVIER_STOKES
KIND_TURB_MODEL= SST

MATH_PROBLEM= DIRECT
RESTART_SOL= NO

MACH_NUMBER= 0.058
FREESTREAM_TEMPERATURE= 288
FREESTREAM_PRESSURE= 101325
FREESTREAM_DENSITY= {rho}
FREESTREAM_VELOCITY= ({V},0,0)

REF_ORIGIN_MOMENT_X= 0
REF_ORIGIN_MOMENT_Y= 0
REF_ORIGIN_MOMENT_Z= 0

REF_AREA= 1.0
REF_LENGTH= 1.0

MARKER_HEATFLUX= (cube, 0.0)
MARKER_HEATFLUX= (ground, 0.0)

MARKER_FAR= (farfield)

MARKER_MONITORING= (cube)
MARKER_PLOTTING= (cube)

NUM_METHOD_GRAD= GREEN_GAUSS

CFL_NUMBER= 2
CFL_ADAPT= YES

ITER= 1000

MESH_FILENAME= {mesh_file}

OUTPUT_FORMAT= CSV
CONV_FILENAME= history
"""

with open(cfg_file,"w") as f:
    f.write(cfg)

# =========================
# 3. Run solver
# =========================

subprocess.run(["SU2_CFD",cfg_file],check=True)

# =========================
# 4. Extract drag
# =========================

data = np.genfromtxt("history.csv",delimiter=",",names=True)

Cd = data["CD"][-1]

A = 1.0

Drag = 0.5*rho*V**2*A*Cd

print("========== RESULTS ==========")
print("Cd =",Cd)
print("Drag force =",Drag,"N")
print("Expected ≈ 257 N")