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

![The GUI: one tab per run, this run's parameters on the left, the scene in the
middle, its results on the right](docs/screenshot.png)

Two samples ship with the tool: `sample.stl` is a unit cube, useful because its
Cd is known and easy to check; `sample2.stl` is a mock tadpole trike -- a
reclined rider envelope plus three wheels -- whose four separate bodies are what
the shape search exists for.

## Runs

The unit of work is a **run**: one shape, the parameters it was given, and the
results that came back. Every tab is one. A run stops changing once it has been
solved, so the history *is* the record — every number on screen can be traced to
the exact inputs that produced it, and a run you solved an hour ago still says
what it said then.

That single rule settles the questions a mutable "current scene" leaves open:

- **Editing a solved run does not rewrite it.** Change the wind speed
  afterwards and the knob is flagged with the value the run was actually solved
  at, next to a way back. The curve stays on screen — it is still a real
  measurement — but never without saying what it belongs to.
- **Computing forks.** In a fresh draft, **Compute drag** solves into that run.
  In a solved one it opens a *new* run carrying your edits, auto-titled and
  auto-described (`Re-run of trike · drag #1 with a different wind speed`).
- **The freeze is per run.** Only the working run locks its parameters. Every
  other tab stays fully readable and editable while the solver grinds, and the
  solving tab keeps a progress bar under its label so you can watch it from
  anywhere.
- **Every run says where its shape came from.** The lineage line under the
  title names the parent — an imported file, or the shell of a shape run — so
  the chain from payload to final hull is always readable.

There are two verbs, in a bar that spans the run they act on:

| | |
|---|---|
| **Compute drag** | solve this run's shape at these conditions |
| **Derive a lower-drag shape** | wrap this run's shape in a single-body fairing, as a new run |

So the loop is: import an STL, compute its drag for a baseline, derive a shape
around it, open that shell as its own run, compute *its* drag — and because the
new run knows its parent, the headline number is shown as a change
(`Cd·A 0.288 m², −16.1% vs trike · drag #1`). That is the number the loop
exists to move. Repeat as needed; nothing you did earlier is overwritten.

## The fairing is always a single closed shell

Separate pods around separate lumps are not offered, and there is no choice of
body count. The gap between two pods is a channel with a boundary layer off
both walls — usually worse than the solid bridge it was avoiding, and it needs
cells finer than the gap to mesh at all. So the only question left is *how much
closing it takes* to reach one body, which is a length scale rather than a
decision.

A morphological closing (dilate by r, erode by r) hugs each lump at small r and
bridges the gaps at large r. The component count is monotone non-increasing in
r, so there is exactly one threshold radius where the payload first becomes a
single body, and every radius above it also works. The tool **bisects for that
threshold** and builds just above it: enough closing to merge, and no more,
because every millimetre past it is frontal area bought for nothing. On the
sample trike that lands at 67 mm, where the old pick-a-plateau approach used
100 mm and paid 2% more frontal area for it.

Two things make the merge aerodynamic rather than merely geometric:

- The closing is **anisotropic**, stretched along the flow. Two lumps in line
  merge far more readily than two side by side, because bridging in-line lumps
  costs almost no frontal area while bridging spanwise ones fills the whole span.
- The clearance counts in the sweep too. Offsetting the skin outward is itself a
  dilation, so it merges any bodies closer than twice the clearance — which
  means the topology worth counting is the skin's, not the closed set's.

The skin itself is the clearance level set of a signed distance field, so the
payload gap is exact by construction and the surface is smooth to sub-voxel
precision. The field is Gaussian-filtered before the surface is extracted:
distance fields computed from binary voxels carry staircase ripples at the
pitch scale, and filtering the *field* removes them at the source — a Gaussian
preserves linear fields exactly, so flat and gently curved regions do not move
at all — where smoothing the extracted *mesh* could only polish triangle by
triangle.

**The built shell is then verified, not assumed.** The sweep runs on a coarse
grid because counting components tolerates a blurry payload, while the skin is
built on a fine one that resolves narrow gaps the sweep blurred shut — so the
threshold it found can still come out in pieces. The tool splits the built mesh,
counts the bodies, and opens the radius until it really is one, reporting how
many attempts it took. A two-piece "single shell" would mesh into a choked
channel and report a drag coefficient for a shape nobody chose. Containment is
re-checked against the payload the same way.

### From packaging skin to a body you would fly

The closing decides *whether it is one body*, never the *profile*: it is bounded
by the convex hull, so on a convex payload it does nothing at all, and the skin
would come out as the payload plus clearance — a rounded cube for a cube.
Nothing in "wrap this as tightly as possible" ever grows a tail, because a tail
is volume the payload does not need.

So a second stage grows one deliberately. The skin is the **minimal
taper-bounded envelope** of the closed set: the smallest body whose
cross-sections never grow steeper than the nose angle (45° by default) and
never shrink steeper than the tail angle (12°) along the flow. Those two
limits are the whole of classical streamlining — a tail shallow enough to keep
the boundary layer attached, a nose that may be blunt because accelerating
flow forgives almost anything — and the minimal body satisfying them is
computed exactly, per voxel slice, from running distance fields. A cube in, a
teardrop out: fineness ratio ~3.7, frontal area unchanged, with the tail
length set by the taper limit rather than by taste.

The envelope also fairs in-line bodies into each other automatically — the
leading body's tail cone reaches the trailing body's nose — which helps the
merge rather than hindering it, and is another reason the body count is taken
from the built mesh rather than trusted from the sweep. What the taper limit
*costs* (wetted area, hence friction) versus what it saves (pressure drag) is
not decided here: solve the shell and compare its Cd·A against the payload's.
Tweak the angles or disable the stage entirely (`--no-streamline`, or the
checkbox in the GUI's shape search) to see the difference in numbers.

**Compare by Cd·A, not Cd.** A shape can post a flattering coefficient purely by
being bigger, since Cd is normalised by the frontal area it is quoted on. On the
mock trike the shell's Cd is comfortably below the bare payload's and it still
has to earn that against 60% more frontal area — which is exactly why the delta
tile reports drag area rather than the coefficient.

## What the GUI does

**Tabs.** One per run, with a glyph that carries two things at once: hue is the
kind (blue solves drag, orange derives shapes) and fill is the state, so a
glance at the bar answers what is open and what it is doing. A solving run keeps
a progress bar under its label from every tab. Close one with the ×; open a new
one with **+**, **Import STL** or the **Library**.

**Scene view.** The shape as loaded from STL, the wind as an arrow arriving from
upstream, and the road as a plane at z = 0 with a grid at hull-length spacing.
Orbit with the left mouse button, zoom with the wheel, pan with the right. The
legend names the shape, so "which one am I looking at" never needs the panel.

**Editing.** Wind speed, azimuth and elevation; yaw, pitch and roll; ride
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

**As run.** Under the results, an immutable record of the shape, flow and
solver settings the numbers came from — separate from the editable title above
it, and the thing that makes editing a solved run safe.

**Run files.** One JSON document holds the STL (embedded), the flow setup and
the results. A run that has not been solved and one that has are the same kind
of file and open the same way, so you can build a case here, compute it on
another machine, and open the result back as a run.

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
python src/runner.py new --stl rider_and_wheels.stl -o baseline.aero.json
python src/runner.py run baseline.aero.json                       # the payload's own drag
python src/runner.py fair --payload rider_and_wheels.stl -o shell.aero.json
python src/runner.py run shell.aero.json                          # the shell's
python src/runner.py compare baseline.aero.json shell.aero.json   # ranked by Cd·A
```

`src/drag.py` is the SU2-flavoured entry point: the same CLI with SU2 selected
by default.

### Computing offline

The GUI's **Library → Save this run** writes into `scenes/`. Those are plain
scene files:

```bash
# on the machine that has the solvers
python src/runner.py run scenes/case.aero.json -o scenes/case.solved.json
```

Open the solved file back in the GUI with **Open run file** and it appears as a
run with its results in place. `runner.py export` is the escape hatch when you
would rather drive the solvers by hand: it writes a complete OpenFOAM case and a
complete SU2 case, each with an `Allrun` script.

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

Or run the whole thing as a compose stack, GUI included:

```bash
docker compose --profile solvers pull   # published images from GHCR, no compile
docker compose up web                   # then open http://127.0.0.1:8000
```

`docker-compose.yml` pulls the images that `.github/workflows/containers.yml`
publishes on every release tag — which is the point: the SU2 image compiles
from source, and pulling it skips that entirely. Its twin
`docker-compose-local.yml` is the same stack built from the checkout, for
unreleased changes or machines without registry access. Either way the GUI
container launches the solver containers as *siblings* through the host's
Docker socket, and case directories live on the identity-mounted
`/tmp/aero-cases` — inside the Docker Desktop VM on Windows, which is the fast
side of the filesystem boundary described below.

Two images for the solvers, because they share no useful base:

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
| Screening | 150 | 26 | one solve, scaled | ranking designs against each other |
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
| `src/server.py` | FastAPI backend for the GUI: the open runs and their jobs |
| `src/runs.py` | The run record — shape, parameters, results — and the as-run diff |
| `src/web/` | Browser front end (three.js vendored locally, no CDN) |
| `src/scene.py` | Scene file format: payload, hull, flow, solver settings, results |
| `src/fairing.py` | Closing sweep, single-body threshold, shell generation |
| `src/estimates.py` | Runtime prediction, self-calibrating from measured solves |
| `src/metrics.py` | Frontal area, flow domain, Reynolds analysis |
| `src/solvers.py` | Backend registry and the scale-vs-sweep orchestration |
| `src/openfoam.py` | OpenFOAM case generation and run |
| `src/su2.py` | SU2 case generation and run |
| `src/execution.py` | Where the solvers run (native/Docker/WSL) and how many ranks they get |
| `docker/` | Solver and GUI images, and the solver build scripts |
| `docker-compose.yml` | The stack on published GHCR images; `-local.yml` builds instead |
| `src/runner.py` | Command line: info / new / run / show / fair / compare / export |
| `src/drag.py` | SU2-flavoured entry point into the same CLI |

Scene files are format version 2 and older ones are rejected rather than
migrated; rebuild them from the source STL. There is no compatibility layer,
deliberately -- this is a tool under development, not a shipped product.
