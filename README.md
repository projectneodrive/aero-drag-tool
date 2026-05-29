# aero-drag-tool

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
- `drag.py` is kept as a compatibility entry point and now forwards to the CLI.
