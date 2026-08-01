# aero-drag-tool

This repo contains a python tool to compute the drag of a given shape and optimise the aero.

Two solver tracks live side by side:

- **SU2** (`drag.py`) — standalone script that builds a gmsh mesh and runs `SU2_CFD`.
- **OpenFOAM** (`drag_openfoam.py` / `optimise_hull.py`) — convex-hull approximation plus a drag-minimisation loop, running through WSL Ubuntu 22.04.

# SU2 track

## Install
Tested on: Ubuntu 22.04 / Pop!_OS 22.04
First you need to install SU2 then you can run the python script.

### Clone this repo
```bash
git clone git@github.com:projectneodrive/aero-drag-tool.git
cd aero-drag-tool
```

### install su2 and dependencies
```bash
chmod +x setup.sh
./setup.sh
```

### run the script

```bash
python3 drag.py
```

# OpenFOAM track

Python tooling for turning an arbitrary STL into a smooth convex hull approximation, exporting a surface-function representation, and running a drag-minimisation loop with either a fast analytical proxy or a real OpenFOAM backend running through WSL Ubuntu 22.04.

## What is implemented

- STL import and convex-hull extraction with `trimesh`.
- Smooth convex support-function fitting on the unit sphere.
- Mesh export from the fitted support function.
- Drag estimation from projected area, with a finite-difference optimisation loop, plus an OpenFOAM drag backend.
- Outputs for `final_hull.stl`, `surface_function.py`, `raw_support.npy`, `weights.npy`, and a history plot.

## Install

```bash
python -m pip install -r requirements.txt
```

## Run

```bash
python optimise_hull.py --input shape.stl --wind 10,0,0 --ground 0.2 --output_dir results
```

Optional flags:

- `--max_iter` sets the optimisation budget.
- `--n_directions` and `--n_centers` control the smooth hull fit.
- `--visualize` writes a comparison plot.

## Notes

- The default solver path is still the analytical proxy for speed.
- Use `--solver openfoam` to run the actual OpenFOAM backend through WSL.
- The OpenFOAM case is generated per evaluation and uses `simpleFoam` plus a `forceCoeffs` function object.
- `drag_openfoam.py` is a compatibility entry point that forwards to the CLI.
