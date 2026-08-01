# aero-drag-tool

Compute the aerodynamic drag of a shape with two independent CFD solvers and
compare them, from a browser GUI or from the command line.

The point of running two solvers is the cross-check: OpenFOAM and SU2 are given
the same placed mesh, the same flow domain and the same reference area, so the
Cd they report can be compared directly. If they disagree, the tool says so.

```
python -m pip install -r requirements.txt
python src/server.py           # opens http://127.0.0.1:8000
```

The application lives in `src/`; the repository root holds only the README,
dependencies, the SU2 install script and the two sample STLs.

![The GUI: packaging inputs and fairing candidates on the left, the scene in the
middle, actions and results on the right](docs/screenshot.png)

Above: the bundled **Sample trike** (`sample2.stl`, orange) inside the merged
single-shell fairing the tool proposed for it, with the drag force and drag
coefficient curves from a four-speed OpenFOAM sweep. Two samples ship with the
tool: `sample.stl` is a unit cube, useful because its Cd is known and easy to
check; `sample2.stl` is a mock tadpole trike -- a reclined rider envelope plus
three wheels -- whose four separate bodies are what the packaging sweep is for.

## From a payload STL to a hull, without needing to know CFD

There is one import path. **Import STL** (or **Sample cube**) loads a shape as
both the hull and the payload, and what happens next is the button you press:
**Compute drag** flies it as-is, **Analyse packaging** treats it as the thing a
fairing has to enclose.

The shortest path to a hull:

1. **Import STL** — whatever has to fit inside: rider, wheels, battery.
2. **Analyse packaging** — sweeps a closing radius and proposes one fairing per
   stable topology, each verified to enclose the payload.
3. **Compare designs** — runs every candidate through the solvers at the
   screening preset and ranks them by drag area.
4. The winner is selected for you, with its own drag curve. Switch to
   **Accurate** and **Compute drag** to confirm it properly, then
   **Download hull STL**.

Every computation is titled automatically from the scene name and the operation
(`payload · drag #2`), and both the title and its description are editable.
While a solver runs, every parameter is locked, so the values on screen are
always the ones the running job was given.

### How it decides one lump or several

"Should the wheels get their own pods or disappear into one shell?" is not a
discrete choice — it is a **length scale**. Two lumps are one body when the gap
between them is small compared to the skin you would wrap around them.

So the tool sweeps that scale. A morphological closing (dilate by r, erode by r)
hugs each lump at small r and bridges the gaps at large r; the topology changes
on its own. Plot the number of separate bodies against r and you get a
staircase — and **the flat runs are the candidate designs**. A wide plateau is a
robust topology; a narrow one is two parts happening to nearly touch.

Two things make it aerodynamic rather than merely geometric:

- The closing is **anisotropic**, stretched along the flow. Two lumps in line
  merge far more readily than two side by side, because bridging in-line lumps
  costs almost no frontal area while bridging spanwise ones fills the whole span.
- Candidates whose bodies come within ~6% of the hull length are flagged
  **choked**: a narrow channel is high velocity with a boundary layer off both
  walls, often worse than the solid bridge you were avoiding, and it needs cells
  finer than the gap to mesh at all.

The skin itself is the clearance level set of a signed distance field, so the
payload gap is exact by construction and the surface is smooth to sub-voxel
precision. Every candidate is then re-checked with a real containment test.

The clearance counts in the sweep too. Offsetting the skin outward is itself a
dilation, so it merges any bodies closer than twice the clearance -- which means
the topology worth counting is the skin's, not the closed set's. Count the
closed set and the tool proposes candidates the builder cannot actually make.

**Ranking is by Cd·A, not Cd.** On a mock rider-and-wheels payload the merged
shell posts the *lowest* Cd (0.603 against 0.623) and still loses, because its
frontal area is 8% larger. Ranking on the coefficient alone picks the wrong
design.

## What the GUI does

**Scene view.** The hull as loaded from STL, the wind as an arrow arriving from
upstream, and the road as a plane at z = 0 with a grid at hull-length spacing.
Orbit with the left mouse button, zoom with the wheel, pan with the right.

**Editing.** Wind speed, azimuth and elevation; hull yaw, pitch and roll; ride
height above the road; and the road itself, which can be switched off entirely
or made to slide at free-stream speed. Air density and viscosity are editable
too. The frontal area updates as you drag, because it depends on the wind angle.

**Results.** Two charts, and Cd gets its own: it is *not* constant, since it
varies with Reynolds number, so drag force and drag coefficient are plotted on
separate axes rather than sharing one. (Overlaying them would be a dual-axis
chart, where the alignment of the two scales is arbitrary and invents a
relationship that is not in the data.) The table lists Cd per speed beside the
force. Points the solver actually computed are drawn filled; points
extrapolated from one solve are hollow.

**Scene files.** One JSON document holds the STL (embedded), the flow setup and
the results. A scene that has not been computed and one that has are the same
kind of file and open the same way, so you can build a case here, compute it on
another machine, and open the result back in the GUI.

## Frontal area, and why the old number was wrong

Drag coefficients only mean something against a stated reference area, and this
tool uses the true silhouette: the projected triangles are rasterised and the
union is measured, so surfaces hidden behind other surfaces are counted once.

The `sum(area * max(n . d, 0))` estimate this repo used before is exact only for
convex bodies. On a torus it over-reports by 62% (2.4 m² against a true
1.4794 m², analytically 1.4827 m²); on any hull with a cavity or a wheel well it
is similarly optimistic, and the error goes straight into Cd.

## Speed curves: one run scaled, or a sweep

Drag follows `0.5 * rho * V^2 * A * Cd` only while Cd itself holds still, and Cd
is a function of Reynolds number. So the tool computes the Re range across your
speed range and picks:

- **scale** — one solve at the reference speed, the curve follows V².
- **sweep** — a solve at every speed on the curve.

It chooses **sweep** when the Re range overlaps the transitional band
(2×10⁵ – 10⁶), where Cd genuinely moves with speed, or when Re varies by more
than 3× across the range. Either way it says what it chose and why, in the GUI
and on the command line. You can override it.

For a vehicle running 5–20 m/s this virtually always lands on sweep: the range
spans 4× in Re and straddles the transitional band. That is real physics, not
conservatism — scaling a 20 m/s solve down to 5 m/s there can be well off.

## Command line

```bash
python src/runner.py info                                  # which solvers are usable here
python src/runner.py new --stl hull.stl --ground 0.15 -o case.aero.json
python src/runner.py run case.aero.json --solver openfoam --solver su2
python src/runner.py show case.aero.json
python src/runner.py export case.aero.json --dir cases/    # ready-to-run solver cases

# The whole design loop, headless:
python src/runner.py fair --payload rider_and_wheels.stl --output_dir candidates/
python src/runner.py compare candidates/*.aero.json --quality screening
```

`src/drag.py` is the SU2-flavoured entry point: the same CLI with SU2 selected
by default.

### Computing offline

The GUI's **Save** writes into `scenes/`. Those are plain scene files:

```bash
# on the machine that has the solvers
python src/runner.py run scenes/case.aero.json -o scenes/case.solved.json
```

Open the solved file back in the GUI with **Import scene** and the results
appear. `runner.py export` is the escape hatch when you would rather drive the
solvers by hand: it writes a complete OpenFOAM case and a complete SU2 case,
each with an `Allrun` script.

## Solvers

| Backend | What it runs | Where |
|---|---|---|
| `openfoam` | blockMesh → snappyHexMesh → `foamRun -solver incompressibleFluid`, drag from a `forceCoeffs` function object | OpenFOAM 13 in WSL `Ubuntu-22.04` |
| `su2` | gmsh volume mesh → `SU2_CFD` incompressible RANS, drag from the force coefficients projected onto the wind | `SU2_CFD` on PATH, or in WSL (including `~/su2-install/bin`) |

There is deliberately no analytical backend. `0.5 ρ V² A Cd` needs you to supply
the Cd, which is the number the tool exists to find — it can only ever echo an
assumption back. The half of it that *is* knowable without a solver, the frontal
area, is shown in the Geometry panel already.

### Quality presets and run time

| Preset | Iterations | Mesh | Speed curve | For |
|---|---|---|---|---|
| Screening | 150 | 26 | one solve, scaled | ranking candidates against each other |
| Balanced | 400 | 40 | automatic | ordinary work |
| Accurate | 1000 | 60 | automatic | the design you are keeping |

Every run is estimated before it starts, and the estimate **calibrates itself**:
each completed solve is recorded to `runtime_history.json` and later predictions
are fitted to those measurements rather than to numbers shipped with the tool.
During a run the remaining time is rescaled by how the run is actually going, so
a machine slower than the model converges on a correct ETA after the first solve
instead of insisting on its original guess.

Both CFD backends average the coefficient over the last 20% of iterations
rather than taking the final value, because a steady solve on a bluff body
keeps oscillating by a few percent. If the remaining spread is over 5% of the
mean, the result is flagged as unconverged.

### OpenFOAM notes

- OpenFOAM 13 replaced `simpleFoam` with `foamRun -solver incompressibleFluid`;
  the tool calls the latter directly.
- Coefficients are read from the file the solver writes during the run, at
  `postProcessing/forceCoeffsIncompressible/<t>/forceCoeffs.dat`. Running
  `postProcess -func ...` afterwards fails with "Could not find U, p".
- That file's columns are `Time Cm Cd Cl Cl(f) Cl(r)`. Cd is the **third**
  column; the parser locates it by header name. Reading it positionally is how
  this repo previously reported Cm as the drag coefficient.

### SU2 notes

- The old cube-only `.geo` is gone. Volume meshing now runs through the gmsh
  Python API: the STL is merged as a discrete surface, re-classified into
  parametrisable patches, and used as the inner boundary of a box, with
  `body` / `ground` / `farfield` markers.
- The free stream is passed as an explicit velocity vector with AOA and
  sideslip at zero, and drag is recovered by projecting `(CFx, CFy, CFz)` onto
  the wind direction. That is correct for any wind angle and avoids depending
  on how SU2 orients its own lift/drag axes.
- Mesh generation runs anywhere gmsh installs, including Windows. Only the
  solve needs SU2.

## Installing the solvers

`setup.sh` builds SU2 v8.4.0 (Ubuntu/Pop!_OS tested). OpenFOAM 13 is expected at
`/opt/openfoam13` inside WSL `Ubuntu-22.04`; adjust `WSL_DISTRO` and
`OPENFOAM_BASHRC` at the top of `src/openfoam.py` if yours lives elsewhere.

Note that `setup.sh` appends its `PATH` export to the end of `~/.bashrc`, past
the early return for non-interactive shells, so a non-interactive `bash -lc`
will not see it. The SU2 backend therefore also probes `~/su2-install/bin`
directly.

## Layout

| File | Role |
|---|---|
| `src/server.py` | FastAPI backend for the GUI |
| `src/web/` | Browser front end (three.js vendored locally, no CDN) |
| `src/scene.py` | Scene file format: payload, hull, flow, solver settings, results |
| `src/fairing.py` | Closing sweep, topology plateaus, candidate fairing generation |
| `src/estimates.py` | Runtime prediction, self-calibrating from measured solves |
| `src/metrics.py` | Frontal area, flow domain, Reynolds analysis |
| `src/solvers.py` | Backend registry and the scale-vs-sweep orchestration |
| `src/openfoam.py` | OpenFOAM case generation and run |
| `src/su2.py` | SU2 case generation and run |
| `src/runner.py` | Command line: info / new / run / show / fair / compare / export |
| `src/drag.py` | SU2-flavoured entry point into the same CLI |

Scene files are format version 2 and older ones are rejected rather than
migrated; rebuild them from the source STL. There is no compatibility layer,
deliberately -- this is a tool under development, not a shipped product.
