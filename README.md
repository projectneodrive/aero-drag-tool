# aero-drag-tool

Compute the aerodynamic drag of a shape with two independent CFD solvers and
compare them, then derive a lower-drag shape around it — from a browser GUI or
from the command line.

The point of running two solvers is the cross-check: OpenFOAM and SU2 are given
the same placed mesh, the same flow domain and the same reference area, so the
Cd they report can be compared directly. If they disagree, the tool says so.

![A shape run: the imported payload in orange, the single-body shell the tool
derived around it in blue, with the closing sweep and the shell's measurements
on the right](docs/images/04-shell.png)

**New to any of this?** [`docs/how-it-works.md`](docs/how-it-works.md) explains
the whole pipeline — what a CFD solver actually does, how the shape maker
wraps a payload, how the optimiser searches — in plain language, with the
reasoning behind every technical choice. No prior aerodynamics assumed.

---

## Install

Two routes. **Docker is recommended**: it pins OpenFOAM 13 and SU2 8.4.0
rather than inheriting whatever the machine happens to have, nothing lands on
`PATH`, and the SU2 image is published pre-compiled so you skip a long build.

### Docker (recommended)

Needs Docker — Docker Desktop with the WSL2 backend on Windows, Docker Engine
on Linux.

```bash
git clone https://github.com/projectneodrive/aero-drag-tool.git
cd aero-drag-tool

# The GUI bind-mounts these two, and neither is in git. Create them first, or
# Docker invents a *directory* where the history file should be and run-time
# calibration quietly stops working.
mkdir -p scenes
[ -f runtime_history.json ] || echo '{"samples": []}' > runtime_history.json

docker compose --profile solvers pull   # web + both solver images, from GHCR
docker compose up web                   # then open http://127.0.0.1:8000
```

On Windows PowerShell those two prep lines are:

```powershell
New-Item -ItemType Directory -Force scenes | Out-Null
if (-not (Test-Path runtime_history.json)) { '{"samples": []}' | Out-File -Encoding utf8 runtime_history.json }
```

That is the whole install. The `web` container serves the GUI and launches the
solver images as *sibling* containers through the host's Docker socket, so
there is no second thing to start.

To build the images from the checkout instead — unreleased changes, or no
registry access — use the twin compose file:

```bash
docker compose -f docker-compose-local.yml --profile solvers build
docker compose -f docker-compose-local.yml up web
```

Expect that to take a while: the SU2 image compiles from source with MPI,
which is exactly what pulling the published image avoids. See
[Containers](#containers) for why each image is built the way it is.

### Local Python

Gets you the GUI, all the geometry, and the whole shape maker. Computing drag
additionally needs at least one solver — see below.

```bash
python -m pip install -r requirements.txt
python src/server.py             # opens http://127.0.0.1:8000
```

Python 3.10+. The front end is static files with three.js vendored in the
repo, so there is no node build step.

For the solvers, easiest is still to build the two images and let the tool
find them:

```bash
./docker/build.sh                # .\docker\build.ps1 on Windows
```

Or install them natively: `setup.sh` builds SU2 v8.4.0 (Ubuntu/Pop!_OS
tested), and OpenFOAM 13 is expected at `/opt/openfoam13` inside WSL
`Ubuntu-22.04` — adjust `WSL_DISTRO` and `OPENFOAM_BASHRC` at the top of
[`src/openfoam.py`](src/openfoam.py) if yours lives elsewhere. See
[Installing the solvers](#installing-the-solvers) for the details and the
gotchas.

### Check it worked

```bash
python src/runner.py info
```

```
Solver availability:

  [yes] OpenFOAM     foamRun via WSL Ubuntu-22.04, 6 processes
  [no ] SU2          SU2_CFD not found natively, in Docker or in WSL. Build the image with docker/build.sh.

  cores visible : 8
  default ranks : 6 (80% of them; override with --processes)
```

The GUI shows the same thing, live, in the Solve panel: `ready` next to a
backend it can use, `not here` next to one it cannot, with the reason. You can
load a shape, inspect its geometry and derive shells with no solver at all —
only **Compute drag** needs one.

---

## Quickstart: from an STL to a refined shell

Two samples ship with the tool: `sample.stl` is a unit cube, useful because
its Cd is known and easy to check; `sample2.stl` is a mock tadpole trike — a
reclined rider envelope plus three wheels — whose separate bodies are what the
shape search exists for. This walkthrough uses the trike, and every number in
it is a real measurement from the run pictured.

### 1. Load a shape

**Import STL**, or **Sample trike** to follow along. A new tab opens: that is
a *run*, and it holds this shape, the conditions you set, and whatever the
solvers eventually say about it.

![A freshly imported run: the payload on the road with the wind arrow, its
parameters on the left, its measured geometry on the right](docs/images/01-payload.png)

The right-hand panel is already useful before anything is solved. The frontal
area is the true silhouette at this wind angle — projected triangles
rasterised and unioned, so surfaces hiding behind other surfaces are counted
once — and it updates as you drag the wind or attitude sliders. The yellow
note is the Reynolds analysis telling you, in advance, whether one solve can
be scaled across your speed range or whether every speed needs its own.

Set the wind speed, ride height and attitude on the left. Pick a quality
preset under **Solve**: *screening* to rank designs against each other,
*balanced* for ordinary work, *accurate* for the design you are keeping.

### 2. Compute its drag — the baseline

Press **Compute drag**. The run's parameters freeze (so the numbers can always
be traced to the inputs that produced them), a progress bar appears under its
tab, and every *other* tab stays fully live while the solver grinds.

![The solved baseline: drag force and drag coefficient on separate charts,
Cd·A in the headline tile](docs/images/03-baseline.png)

```
   Cd = 0.7793    A = 0.3012 m²    →   Cd·A = 0.2347 m²     (screening)
```

Drag force and Cd get **separate charts on purpose**: Cd is not constant — it
moves with Reynolds number — so plotting both on one dual axis would invent a
relationship that is not in the data. Filled markers are speeds the solver
actually computed; hollow ones were scaled from a single solve.

### 3. Derive a shape around it

Under **Shape search**, press **Derive a lower-drag shape**. This opens a new
run that wraps *this* shape in a single closed shell.

That is the screenshot at the top of this README. What it shows:

- The payload (orange) inside the shell (blue), which **encloses** it with the
  clearance you asked for — checked by ray-casting, not assumed.
- **Bodies vs closing radius**: the payload starts as 3 separate lumps and
  becomes one at a closing radius of 67 mm. The tool bisects for that
  threshold and builds at 71 mm — the smallest radius that holds, because
  every millimetre past it is frontal area bought for nothing.
- `BODIES 1`, verified by splitting the built mesh and counting, not by
  trusting the sweep.

The shape solver dropdown picks how the profile is chosen: **Heuristic** shapes
it by taper rules in seconds; **True loop** puts the CFD solver inside the
derivation and measures the nose and tail angles this payload actually wants
(see [The true loop](#the-true-loop-measuring-the-angles-instead-of-assuming-them)).

The derive ends by **solving the shell once** at the run's quality, so the
shape panel shows the drag it achieved without you running anything else —
one backend, one speed, a headline number. The true loop's confirmation is
already that solve, so the loop path reuses it rather than paying twice.
Because the derive now depends on the quality preset, that selector is
mirrored into the Shape search panel next to the button (it is the same
setting as Solve → Quality, not a second one). Turn the measurement off with
`measure_shell` if you only want geometry.

### 4. Compute the shell's drag, and compare

Press **Compute drag on the shell**. That opens the shell as its own run and
solves it, so the shape run stays as it was.

![The shell solved: the headline tile reports drag area with the change
against the run it came from](docs/images/05-shell-drag.png)

```
   Cd = 0.6629    A = 0.4747 m²    →   Cd·A = 0.3147 m²     (screening)
                                            +34.1% vs trike · drag #1
```

Because the new run knows its parent, the headline is a **change**, and it is
quoted in **drag area (Cd·A) rather than Cd** — a shape can post a flattering
coefficient purely by being bigger.

Read that result carefully, because it is the tool working, not failing. The
shell is a *better shape*: its Cd is 15% below the bare payload's. It is still
the *worse vehicle*, because merging three separate lumps into one closed body
means bridging the gaps between them, and every bridge is silhouette that used
to be open air — **+58% frontal area**, which 15% of Cd does not pay back.

That is the loop's whole purpose: it measured instead of assuming. From here
you would attack the area (tighter clearance, higher streamwise bias, move the
payload), or fly the true loop to find out what tail angle this body actually
wants. Nothing you did earlier is overwritten — each attempt is its own tab.

### The same loop, headless

```bash
python src/runner.py new  --stl sample2.stl     --ground 0.15 --speed 15 --mode scale -o baseline.aero.json
python src/runner.py fair --payload sample2.stl --ground 0.15 --speed 15 --mode scale -o shell.aero.json
python src/runner.py compare baseline.aero.json shell.aero.json   # solves both, ranks them
```

```
Ranking by drag area (Cd x A, lower is better):

  scene                              solver          Cd     A m^2   Cd.A m^2
  baseline.aero.json                 openfoam    0.4071    0.3012    0.12262
  shell.aero.json                    openfoam    0.4606    0.4747    0.21864
```

Those are *balanced* quality (the CLI default) against the GUI walkthrough's
*screening*, which is why every number differs — and why on this pair even the
Cd ordering flips. Screening is for ranking candidates against each other under
identical settings, not for absolute numbers; see
[Quality presets and run time](#quality-presets-and-run-time).

---

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
(`Cd·A 0.3147 m², +34.1% vs trike · drag #1`). That is the number the loop
exists to move, in either direction: the sign is a measurement, not a promise.
Repeat as needed; nothing you did earlier is overwritten.

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

### Faceted or blended shoulders

Being *minimal* has a consequence worth knowing about: the envelope holds full
section right up to the payload's last slice, and one slice later it is
already shrinking at the full taper. The surface tangent jumps from 0° to the
taper angle in a single step, so **each shoulder is a crease, not a fillet**.
On the sample cube the profile is a 45° square pyramid, a 1.06 m flat prism,
and a 12° square pyramid, with a knife edge at each join.

That is not a smoothing failure — the field Gaussian is chosen to preserve
linear fields exactly, and a crease is precisely where two linear pieces meet.
It follows from minimality, and minimality is in direct conflict with tangent
continuity: a smooth shoulder must carry section the payload does not force.

But **minimum volume was never the objective**. The tool ranks on Cd·A, and the
frontal area is set by the payload's widest section either way. So the shoulder
is a choice rather than a property:

| Profile | What it is |
|---|---|
| **Faceted** | the minimal envelope — flat panels, creased shoulders, shortest body |
| **Blended** | shoulders rounded over a blend length, tangent-continuous into the flat and into the taper |

Blended costs wetted area and length. It does **not** cost frontal area — the
construction is bounded by the payload's own widest section, so the silhouette
is identical and only the wetted area moves. On the sample trike, frontal area
holds to within 0.1% across the whole range while volume climbs smoothly:

| Shoulder fill | Frontal area | Wetted | Volume |
|---|---|---|---|
| 0.00 (faceted) | 0.4810 m² | 6.26 m² | 0.648 m³ |
| 0.05 | 0.4812 m² | 6.71 m² | 0.692 m³ |
| 0.10 | 0.4814 m² | 7.07 m² | 0.730 m³ |
| 0.20 | 0.4814 m² | 7.70 m² | 0.792 m³ |

The setting is the **radial fill** — how much extra section the shoulder
carries, as a fraction of the body's cross-flow half-width. Not a streamwise
length: turning 45° over a short distance needs a large fillet radius, so a
length-based setting swelled the nose 6.5× harder than the tail and behaved as
a two-position switch rather than a control.

Which one is faster is a genuine question, so it is put to the solver rather
than assumed: the true loop searches the blend as a third parameter, and its
range reaches down to zero, so a blended search contains the faceted shape and
returns it if the fillet does not pay.

```bash
python src/runner.py fair --payload sample2.stl --profile blended --shoulder-fill 0.10
```

In the GUI it is the **Shoulders** dropdown and the **Shoulder fill** slider
under Shape search. It is quoted as a fraction of the payload's cross-flow
half-width, so the same number means the same proportions at any scale, and
useful values are small — 0.05 to 0.20.

### The true loop: measuring the angles instead of assuming them

The heuristic's 12°/45° are right *in general*; they are not right for this
payload at this Reynolds number. The **shape solver** menu in the shape search
offers the honest alternative: **True loop** puts the CFD solver inside the
derivation. The heuristic shell is built first and flown as the baseline, then
the loop walks the tail angle and the nose angle by golden-section search —
tail first, because it dominates the trade, then the nose, then the tail again
around the measured nose if the budget allows — building a real shell at each
step, solving it at screening quality with one backend, and reading Cd·A.

**Each candidate is rebuilt from the original payload**, never from the
heuristic shell. The angles are inputs to the envelope generator, not edits to
a mesh, so containment is re-established from scratch every step rather than
inherited from a shape being changed underneath it — and a candidate that
somehow fails the containment check is ranked behind every real number instead
of being allowed to win on a flattering Cd·A.

**The search brackets straddle the angles it started from**, so the loop is
free to lengthen the shell and to shorten it. On the defaults that is a tail
of 6–22° around the heuristic's 12° and a nose of 25–72° around its 45°; set a
25° tail by hand and the bracket moves with it (12–45°) rather than leaving
the starting shape outside its own search range, where every candidate flown
would be on one side of it. If the winner lands on a bracket edge the run says
so — that is the search hitting a bound, not finding a minimum.

What makes this affordable is *what* it optimises. Free-form shape
optimisation needs adjoint gradients and hundreds of solves; this walks the
envelope generator's own two parameters, where every candidate contains the
payload by construction, no solve is wasted on an infeasible shape, and a
budget of ~10 solves (settable as `refine_solves`) lands within a degree or
two of the optimum. The budget is split rather than pooled — the tail searches
first and widest, but the nose keeps a reserved share, so a tight budget
cannot spend the lot on the tail and hand back an unexamined nose angle as if
it had been measured. Expect minutes to an hour, watchable line by line in the
progress log with a measured ETA from the first solve onward.

The result is the argmin over everything flown — baseline included, so the
loop can never hand back something worse than the heuristic. The measured
angles are written into the run's own knobs (so deriving again starts from
them), recorded in the shell panel with the gain over the heuristic, and the
whole evaluation history is kept in the run file. It needs a live CFD
backend, which is checked when you press the button rather than discovered
half an hour into the queue.

### The search mesh is not the judging mesh

That "never worse than the heuristic" guarantee needs one more step to be
worth anything, because **the loop ranks at screening quality and you read the
answer at the run's own**. Those are different meshes, and a ranking taken on
a coarse one only transfers to a fine one if the two agree on the order.

On the sample trike they do not. Here are four shells differing only in tail
angle, each solved at both qualities. Frontal area is constant to four decimal
places *by construction* — the envelope cannot exceed the payload's widest
section — so every difference below is pure Cd:

| Tail | Frontal area | Cd screening | Cd·A screening | Cd balanced | Cd·A balanced |
|---|---|---|---|---|---|
| 9° | 0.4748 m² | 0.4850 | 0.2303 | 0.4581 | 0.2175 |
| 12° | 0.4747 m² | **0.6629** | 0.3147 | 0.4606 | 0.2186 |
| 16° | 0.4747 m² | **0.3860** | 0.1832 | 0.4470 | 0.2122 |
| 20° | 0.4746 m² | 0.4097 | 0.1944 | 0.4199 | 0.1993 |

Read the two Cd columns. Balanced spans **9.7%** across the whole bracket and
moves smoothly — that is the aerodynamics, and it is a modest effect.
Screening spans **72%** and is not even monotone: at 12° it reports 0.6629
against balanced's 0.4606, a **44% error on a single point**.

The real signal is ~10%; the measurement error is ~44%. **The search is
fitting noise**, and its answer is close to random within the bracket — which
is why it sometimes lands better than the heuristic and sometimes worse.
Screening picks 16° here; balanced says 20°.

The dangerous part: that 0.6629 solve **converged**. It was not oscillating,
it did not warn, it settled confidently on a wrong number because a 26-cell
mesh cannot resolve a long shallow afterbody. No convergence check catches
this — only a finer mesh does.

So the loop closes with a **confirmation**: it solves its own winner and the
heuristic shell at the run's quality, keeps whichever actually wins, and says
so. Two extra solves, outside the search budget. If the winner loses, the run
reports `True loop — reverted`, keeps the heuristic shell, and quotes the
margin by which the proxy was wrong.

**On this payload, run the search at balanced.** The confirmation stops the
loop handing back something worse, but it cannot manufacture a good answer
from a noisy search — it can only refuse a bad one. Set **Search quality** to
balanced and the loop ranks on a mesh that can actually see a 10% effect.
There is a real gain waiting there: at balanced, 20° beats the 12° heuristic
by 8.9% on Cd·A.

Three further guards, two of which the trike trips:

- **Split candidates are never flown.** At a 21.5° tail the trike's shell
  comes out in two pieces — the steeper tail cone no longer reaches between
  the lumps. A split shell meshes into a choked channel and returns a
  flattering coefficient for a shape nobody chose. Rejected unsolved.
- **Candidates that stop enclosing the payload are never flown.** The same
  21.5° shell also fails this.
- **Unconverged solves are flagged.** If a third or more of the search was
  still oscillating, the run says the ranking is mostly solver noise. Note
  this does *not* catch the case above — a converged solve on too coarse a
  mesh is confidently wrong, and only the confirmation sees it.

**Search quality** (in the shape search, or `refine_quality` in the scene)
sets the mesh the search itself ranks on. Leave it at screening for a fast
loop with a confirmed answer; raise it to balanced or accurate to remove the
proxy altogether, which is much slower but is what you want when the
confirmation keeps reverting.

With the blended profile selected the loop searches a **third** parameter, the
shoulder blend, after the two angles — see below.

**Compare by Cd·A, not Cd.** A shape can post a flattering coefficient purely by
being bigger, since Cd is normalised by the frontal area it is quoted on. On the
mock trike at screening quality the shell's Cd lands 15% below the bare
payload's — and it still loses, because bridging three lumps into one body cost
it 58% more frontal area. That is exactly why the delta tile reports drag area
rather than the coefficient, and why the loop is worth running rather than
assuming a fairing pays.

Two further cautions the same measurement makes visible. Between screening and
balanced quality the *ordering of Cd itself* flips on this pair — a long
shallow tail is what a coarse mesh resolves worst, so screening flatters it,
which is why screening is for ranking candidates under identical settings and
never for a number you quote elsewhere. And the frontal area is the expensive
axis here: clearance, streamwise bias and how tightly the payload is packed all
move it, and all of them are cheaper to change than the taper angles.

## What the GUI does

**Tabs.** One per run, with a glyph that carries two things at once: hue is the
kind (blue solves drag, orange derives shapes) and fill is the state, so a
glance at the bar answers what is open and what it is doing. A solving run keeps
a progress bar under its label from every tab. Close one with the ×; open a new
one with **+**, **Import STL** or the **Library**.

![A run on the solver: its parameters locked, a measured ETA, the solver log
live in the Progress panel, and every other tab still editable](docs/images/02-solving.png)

**The queue.** The solvers want the whole machine, so runs line up rather than
fight: one solve at a time, with the line itself on a pinned **Queue** tab.
Anything still waiting can be stopped, paused or reordered; whatever is already
on the solver has to finish or be killed.

**Scene view.** The shape as loaded from STL, the wind as an arrow arriving from
upstream, and the road as a plane at z = 0 with a grid at hull-length spacing.
Orbit with the left mouse button, zoom with the wheel, pan with the right. The
legend names the shape, so "which one am I looking at" never needs the panel.

**Editing.** Wind speed, azimuth and elevation; yaw, pitch and roll; ride
height above the road; and the road itself, which can be switched off entirely,
left standing, or driven along (see below). Air density and viscosity are
editable too. The frontal area updates as you drag, because it depends on the
wind angle.

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

## The road, and how fast it is going

A solve happens in the vehicle's frame: the body is held still and the air is
sent at it. That leaves the road with a choice to make, and it is not cosmetic.
A road standing still in that frame is a wind-tunnel floor — it grows its own
boundary layer, thickening all the way to the body, and the flow underneath the
hull is wrong before it ever arrives. A real road does not do that, because the
vehicle is the thing that moves.

So a road that is switched on can also be **driven along**, and its speed is the
vehicle's speed over the ground, applied along the wind's ground heading:

- **Blank (the default)** — the road tracks the wind. This is still air: park the
  atmosphere, and the speed the body feels is the speed it is doing. It keeps
  pace across the whole speed curve, so a sweep sweeps the *vehicle*.
- **A number** — the ground speed, pinned. This is the atmospheric-wind case:
  25 m/s over the ground into a 5 m/s headwind is a 30 m/s wind over a 25 m/s
  road, and only the road speed tells the two apart.

Pinning it does mean Cd stops being one number for the whole curve — the road's
share of the flow changes at every other speed — so the tool says so and points
you at solving each speed instead.

Both backends implement it: OpenFOAM gets a translating `fixedValue` floor with
wall functions on it, SU2 a `MARKER_MOVING` ground with `SURFACE_TRANSLATION_RATE`.

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

[Install](#docker-recommended) has the commands. This section is why the images
are built the way they are.

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

**Containers are the recommended route** — see [Install](#install) for the
commands. The versions are pinned rather than inherited from whatever the
machine happens to have, and nothing lands on `PATH`.

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
| `docs/how-it-works.md` | The whole pipeline explained from first principles |

The repository root holds only the README, dependencies, the compose files, the
SU2 install script and the two sample STLs.

Scene files are format version 2 and older ones are rejected rather than
migrated; rebuild them from the source STL. There is no compatibility layer,
deliberately -- this is a tool under development, not a shipped product.
