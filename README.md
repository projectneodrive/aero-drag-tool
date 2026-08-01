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
2. **Analyse packaging** — sweeps a closing radius, proposes one fairing per
   stable topology, and streamlines each into the smallest taper-bounded body
   that encloses the payload (verified with a real containment test).
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

### From packaging skin to a body you would fly

The closing decides *topology*, never *profile*: it is bounded by the convex
hull, so on a convex payload it does nothing at all, and the skin would come
out as the payload plus clearance — a rounded cube for a cube. Nothing in
"wrap this as tightly as possible" ever grows a tail, because a tail is volume
the payload does not need.

So a second stage grows one deliberately. Each candidate skin is the **minimal
taper-bounded envelope** of its closed set: the smallest body whose
cross-sections never grow steeper than the nose angle (45° by default) and
never shrink steeper than the tail angle (12°) along the flow. Those two
limits are the whole of classical streamlining — a tail shallow enough to keep
the boundary layer attached, a nose that may be blunt because accelerating
flow forgives almost anything — and the minimal body satisfying them is
computed exactly, per voxel slice, from running distance fields. A cube in, a
teardrop out: fineness ratio ~3.7, frontal area unchanged, with the tail
length set by the taper limit rather than by taste.

The envelope also fairs in-line bodies into each other automatically — the
leading body's tail cone reaches the trailing body's nose — which is why the
component count is recounted from the built mesh rather than trusted from the
plateau. What the taper limit *costs* (wetted area, hence friction) versus
what it saves (pressure drag) is not decided here: that is exactly the
question the Cd·A comparison answers. Tweak the angles or disable the stage
entirely (`--no-streamline`, or the packaging checkbox in the GUI) to see the
difference in numbers.

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
| `openfoam` | blockMesh → snappyHexMesh → `foamRun -solver incompressibleFluid`, drag from a `forceCoeffs` function object | OpenFOAM 13, in Docker or in WSL `Ubuntu-22.04` |
| `su2` | gmsh volume mesh → `SU2_CFD` incompressible RANS, drag from the force coefficients projected onto the wind | `SU2_CFD` in Docker, on PATH, or in WSL (including `~/su2-install/bin`) |

There is deliberately no analytical backend. `0.5 ρ V² A Cd` needs you to supply
the Cd, which is the number the tool exists to find — it can only ever echo an
assumption back. The half of it that *is* knowable without a solver, the frontal
area, is shown in the Geometry panel already.

### Where the solvers run

Both backends already ran their solver as an external process in a Linux
environment reached through a shim; WSL was simply the only shim. `src/execution.py`
makes it a choice, probed in this order:

| Mode | When it wins |
|---|---|
| `native` | the solver is on `PATH` — no virtualisation layer, so fastest |
| `docker` | a pinned image is built — reproducible, and the case mounts at `/case` |
| `wsl` | an existing WSL install — the original path, still supported |

`AERO_EXECUTION=docker` pins one mode instead of probing. `python src/runner.py info`
prints which one each backend resolved to.

### Containers

```bash
./docker/build.sh          # both images; .\docker\build.ps1 on Windows
python src/runner.py info  # confirm they were picked up
```

Two images, because the two solvers share no useful base:

- **OpenFOAM** installs v13 from the Foundation's apt repository. There is no
  official v13 image to pull — [hub.docker.com/u/openfoam](https://hub.docker.com/u/openfoam)
  stops at `openfoam11` plus a rolling `-dev` — and installing from apt pins the
  version explicitly instead. The **Foundation** build (openfoam.org) is required:
  the ESI build (openfoam.com, `opencfd/*`) has no `foamRun` at all and spells
  `momentumTransport` / `physicalProperties` differently.
- **SU2** compiles 8.4.0 from source with MPI. The precompiled binaries from the
  download page have parallel support explicitly disabled, and this tool
  dispatches through `mpirun`, so downloading them would silently cost every
  core. `-Denable-cuda` and `-Denable-pywrapper` from `setup.sh` are dropped:
  nothing imports `pysu2`, and see below on GPUs. It is a multi-stage build, so
  the toolchain does not ship in the final image.

On Windows, keep case directories off `C:\` bind mounts — they cross the 9p
filesystem boundary, which is slow at exactly this workload (snappyHexMesh
writes hundreds of thousands of small files). A named volume or a path inside
the WSL2 filesystem is much faster.

### Why the containers are CPU-only

Not an oversight. OpenFOAM's GPU paths (PETSc4FOAM, AmgX) live on the ESI fork,
not the Foundation build this tool drives; adopting them means porting the whole
backend and re-validating every Cd. They accelerate only the linear solve, so
Amdahl's law caps the win at roughly 1.5–2× — and only past ~1M cells, where the
GPU has enough work to beat per-iteration transfer and launch latency. The
largest case here is 0.27M. snappyHexMesh and gmsh have no GPU path at all, and
the measurements in `runtime_history.json` say meshing and fixed overhead *are*
the runtime: a 266k-cell/400-iteration solve came in at 9.7 s against 9–15 s for
42k cells at 150 iterations.

Cores are the lever that actually moves, which is what the next section is for.

### Parallel ranks

The solve runs on **80% of the cores visible to the tool** by default — not all
of them, because the mesher, the GUI and the OS still need one, and an
oversubscribed MPI run is slower than a correctly sized one.

```bash
python src/runner.py run case.aero.json                 # auto: 80% of cores
python src/runner.py run case.aero.json --processes 12   # pin it
python src/runner.py info                                # what it resolved to
```

In the GUI it is the **Parallel ranks** field under Run. Leave it blank for
auto; type a number to pin it. `AERO_PROCESSES` overrides the default for a
whole session.

"Visible to the tool" is not "on the machine": CPU affinity and cgroup quotas
both narrow it, and both are normal when the tool itself runs in a container.
`execution.available_cores` reads `sched_getaffinity` and the cgroup v1/v2 CPU
quota rather than trusting `os.cpu_count`.

A scene stores the rank count only when you pin one, so a saved scene stays
portable to a machine with a different core count. An explicit request is
clamped to the cores that exist.

Meshing is still serial — only the solve is decomposed. Running snappyHexMesh in
parallel means `decomposePar` before it and either `reconstructParMesh` after or
carrying the decomposed mesh into the solve, which changes the pipeline enough
to need validating against known cases first.

**Ranks do not pay off at screening size.** Measured on the sample cube, 42k
cells, 150 iterations, OpenFOAM through WSL on an 8-core laptop:

| Ranks | Wall time |
|---|---|
| 1 | 28.9 s |
| 2 | 29.7 s |
| 6 (the 80% default) | 37.8 s |

More ranks is *slower*, because `decomposePar` and `reconstructPar` are serial
and MPI startup is a fixed cost, while the solve itself is a few seconds of a
run dominated by meshing and process startup. The crossover is somewhere above
these mesh sizes — the accurate preset (resolution 60, refinement 4) generates
far more cells, and that is where the ranks earn their keep. Pin `--processes 1`
for screening sweeps until the crossover is measured on real fairings.

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

**Containers are the recommended route** — `./docker/build.sh` builds both, the
versions are pinned rather than inherited from whatever the machine happens to
have, and nothing lands on `PATH`. Requires Docker (Docker Desktop + WSL2 on
Windows). See [Containers](#containers) above.

The container also makes the offline split practical: design on a laptop, save
the scene, and run `runner.py run` on a machine that has the images, without a
second full install.

### Installing natively instead

`setup.sh` builds SU2 v8.4.0 (Ubuntu/Pop!_OS tested). OpenFOAM 13 is expected at
`/opt/openfoam13` inside WSL `Ubuntu-22.04`; adjust `WSL_DISTRO` and
`OPENFOAM_BASHRC` at the top of `src/openfoam.py` if yours lives elsewhere.

Note that `setup.sh` appends its `PATH` export to the end of `~/.bashrc`, past
the early return for non-interactive shells, so a non-interactive `bash -lc`
will not see it. The SU2 backend therefore also probes `~/su2-install/bin`
directly. The containers do not have this problem: they set `PATH` through
`ENV`, which a non-interactive shell does see.

`setup.sh` also passes `-Denable-cuda`; the containers deliberately do not. See
[Why the containers are CPU-only](#why-the-containers-are-cpu-only).

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
| `src/execution.py` | Where the solvers run (native/Docker/WSL) and how many ranks they get |
| `docker/` | Solver images and their build scripts |
| `src/runner.py` | Command line: info / new / run / show / fair / compare / export |
| `src/drag.py` | SU2-flavoured entry point into the same CLI |

Scene files are format version 2 and older ones are rejected rather than
migrated; rebuild them from the source STL. There is no compatibility layer,
deliberately -- this is a tool under development, not a shipped product.
