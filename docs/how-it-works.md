# How it works

*A long, plain-language walk through everything this tool does and why it does
it that way. No prior aerodynamics needed. If you can picture a swimming pool
and a Lego brick, you can follow all of it.*

The promise of this document: **nothing here is magic.** Every number the tool
shows you comes from a chain of small, boring, checkable steps. By the end you
should be able to point at any figure on the screen and say roughly which
arithmetic produced it.

---

## Contents

- [0. The whole thing in one page](#0-the-whole-thing-in-one-page)
- [1. What drag actually is](#1-what-drag-actually-is)
- [2. How the solver measures drag](#2-how-the-solver-measures-drag)
- [3. How the shape maker builds a fairing](#3-how-the-shape-maker-builds-a-fairing)
- [4. The shape optimiser: putting the solver inside the loop](#4-the-shape-optimiser-putting-the-solver-inside-the-loop)
- [5. The app around it](#5-the-app-around-it)
- [6. A worked example, end to end](#6-a-worked-example-end-to-end)
- [7. Glossary](#7-glossary)
- [8. Not black magic: the cheat sheet](#8-not-black-magic-the-cheat-sheet)

---

## 0. The whole thing in one page

You have a shape — a rider on a bike, a cargo pod, a robot, anything you can
export as an STL file. You want to know how hard the air pushes back when it
moves, and then you want a better shape.

The tool does three things, and they are completely separate machines that
happen to sit in the same box:

```
   your STL                                              a number
  ┌─────────┐    ┌───────────────┐    ┌──────────────┐   ┌────────┐
  │ payload │ →  │ SHAPE MAKER   │ →  │   SOLVER     │ → │  Cd·A  │
  │ 3 lumps │    │ wrap it in    │    │ simulate air │   │ 0.21 m²│
  └─────────┘    │ one smooth    │    │ flowing over │   └────────┘
                 │ shell         │    │ it           │
                 └───────────────┘    └──────────────┘
                        ↑                     │
                        └─────────────────────┘
                    OPTIMISER: try a shape, measure it,
                    adjust, try again  ("the true loop")
```

1. **The solver** takes a shape and tells you its drag. It does this by
   chopping the air around the shape into millions of little boxes and doing
   arithmetic on each box until the numbers stop changing.
2. **The shape maker** takes a lumpy payload and wraps it in one smooth
   streamlined shell that still has room for everything inside.
3. **The optimiser** runs the other two in a loop: build a shell, measure it,
   nudge the shape, measure again — keeping the best one.

Everything else in the app — tabs, queues, progress bars, time estimates — is
bookkeeping around those three.

---

## 1. What drag actually is

### 1.1 Air is stuff

Stick your hand out of a car window at 100 km/h. That push you feel is drag.
Air feels like nothing when you stand still, but it has mass — a cubic metre
of it weighs about 1.2 kg, roughly a large bag of sugar — and when you move
through it you have to shove it out of the way.

Drag comes from two things:

- **Pressure drag** — you push air out of the way at the front, and behind
  you there's a messy swirly low-pressure hole (the *wake*) that sucks you
  backwards. This is most of the drag on a chunky shape.
- **Friction drag** — air sticks to your surface and rubs. The more surface
  you have, the more rubbing. This is most of the drag on a long thin shape.

**These two fight each other,** and that fight is the single most important
idea in this whole tool. Make the tail longer and smoother: less swirly hole
(good), more surface to rub (bad). There's a sweet spot, it depends on the
shape and the speed, and *nobody can tell you where it is by looking* — you
have to measure. That's why the tool exists.

### 1.2 The drag equation

```
       drag force  =  ½ · ρ · V² · A · Cd
                      ─   ─    ─   ─   ──
                      │   │    │   │   └── the "shape score" (no units)
                      │   │    │   └────── frontal area, m²  (your silhouette)
                      │   │    └────────── speed, m/s
                      │   └─────────────── air density, kg/m³ (~1.2)
                      └─────────────────── just the number one-half
```

Read it as a sentence: *the push is half the air's density, times your speed
squared, times how big a hole you punch in the air, times how badly shaped you
are.*

Three of those four you can measure with a ruler and a thermometer. The
fourth, **Cd**, is the whole problem. It is a single number that summarises
"how bad is this shape at moving through air" — about 1.05 for a cube, about
0.04 for a well-designed glider wing section, about 0.3 for a modern car.

You cannot compute Cd from a formula. You have to either build the thing and
put it in a wind tunnel, or simulate the air. This tool simulates the air.

> **Why there is deliberately no "just estimate it" mode**
> A tool that lets you type in a Cd and multiplies it out is not measuring
> anything — it's echoing your assumption back at you with more decimal
> places. Cd is the unknown. The half of the equation that *is* knowable
> without a simulation is the frontal area A, and the app shows you that in
> the Geometry panel from the moment you load a file.

### 1.3 Cd·A, and why Cd alone will lie to you

Here's a trap that catches everyone.

Cd is *divided by* the frontal area. So you can improve your Cd just by
getting bigger — the shape "scores" better per square metre while pushing more
square metres. Here is the sample trike, really measured, both ways:

| | Cd | Frontal area | Cd·A |
|---|---|---|---|
| Bare payload — screening quality | 0.779 | 0.301 m² | 0.235 m² |
| Streamlined shell — screening quality | **0.663** ↓ | 0.475 m² | **0.315 m²** ↑ |
| Bare payload — balanced quality | 0.407 | 0.301 m² | 0.123 m² |
| Streamlined shell — balanced quality | 0.461 | 0.475 m² | **0.219 m²** ↑ |

(Real numbers from `sample2.stl`, OpenFOAM, 15 m/s, road at 0.15 m.)

Look at the screening pair. The shell is a **better shape** — its Cd is 15%
lower, exactly as a streamlined body should be. And it is still the **worse
vehicle**, because Cd·A went *up* by a third.

**Cd·A** — "drag area" — is the honest number. It's what actually appears in
the force equation once you multiply Cd and A together, and it's the number
the app puts in the big tile at the top of the results panel, with the change
against the run it came from.

Where did the extra area come from? The payload is a rider capsule and three
wheels — four parts, which the voxel grid resolves as **three separate lumps**
because the rear wheel already touches the hull. To become one closed body the
shell has to **bridge** the gaps between them, and a bridge across a gap is new
silhouette that wasn't there before. That's the bill: **+58% frontal area**,
paid up front, and a 15% better shape score doesn't cover it.

That is not a bug, and it isn't a verdict on fairings. It's the actual trade.
On a payload with a genuinely bluff front and exposed messy bits — a seated
rider with no envelope, an open frame, luggage strapped on — the same wrapping
usually wins, because there is real pressure drag to recover. **The tool's job
is to tell you which case you're in, not to assume.**

> **A second lesson hiding in that table:** between screening and balanced, the
> *ordering of Cd itself flips.* At 150 iterations on a coarse mesh the shell
> looks 15% better; at 400 iterations on a mesh 1.5× finer it looks slightly
> worse. A long shallow tail is exactly the feature a coarse mesh resolves
> badly, so screening flatters it. Screening is for **ranking candidates
> against each other** under identical settings — never for an absolute number
> you are going to quote elsewhere. Cd·A pointed the same way at both
> qualities, which is part of why that is the number in the big tile.

---

## 2. How the solver measures drag

This part is called **CFD**: Computational Fluid Dynamics. It sounds like a
priesthood. It is actually one simple idea repeated a very large number of
times.

### 2.1 The idea: chop the air into boxes

You cannot track every air molecule — there are about 25 million billion of
them in a cubic centimetre. So instead you chop the space around your shape
into a few hundred thousand little boxes called **cells**, and for each cell
you keep just two things:

- how fast the air in this cell is moving, and in what direction (3 numbers)
- what the pressure is (1 number)

That's it. Four numbers per box. The pile of boxes is called a **mesh**.

```
        the mesh, seen from the side (cartoon; real ones are 3D)

    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤   big cells far away,
    ├──┼──┼──┼┬─┬┼┬─┬┼──┼──┼──┼──┼──┼──┤   where nothing interesting
    ├──┼──┼──┼┼█┼┼┼█┼┼──┼──┼──┼──┼──┼──┤   is happening
    ├──┼──┼──┼┴─┴┼┴─┴┼──┼──┼──┼──┼──┼──┤
    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤   tiny cells hugging the
    └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘   body, where it is
                  ▲                          (█ = your shape)
                  └── this is "refinement"
```

Cells near the body are made much smaller, because that's where the flow
changes fastest. Cells far away are big, because out there the air is just...
going along. This is what the **mesh resolution** and **refinement level**
settings control, and it's why "accurate" quality takes so much longer than
"screening": doubling the resolution gives you eight times as many boxes.

### 2.2 The box of air (the virtual wind tunnel)

Before you can chop the air up, you need to decide *how much* air. The tool
builds a rectangular box around your shape, padded by four times the shape's
own size in each direction ([`metrics.flow_domain`](../src/metrics.py)).

Why four times? If the box is too tight, the walls squeeze the flow and the
drag comes out too high — the same reason a real wind tunnel with a tiny test
section gives wrong answers (it's called *blockage*). Four body-lengths is the
usual rule of thumb for getting that error down to a fraction of a percent.

Each face of the box is given a job:

| Face | What it does |
|---|---|
| upstream | air comes in here, at your chosen speed |
| downstream | air leaves here; pressure is pinned to zero so the solver has a reference |
| sides & top | "free stream" — air can slide past, the box doesn't pinch |
| bottom | the road, if you switched it on |
| the shape itself | a wall: air sticks to it and cannot pass through |

**Both solvers get the identical box.** That's deliberate: if OpenFOAM and SU2
solved slightly different problems, comparing their answers would tell you
nothing.

### 2.3 The guess-and-correct loop

Now the actual simulation. Here is the entire algorithm:

1. Guess. (Usually: "the air is moving at the free-stream speed everywhere.")
2. For every cell, check whether the guess obeys the laws of physics —
   specifically, does mass balance (as much air flows out as flows in), and
   does force balance (pressure differences match how the air is accelerating).
3. It won't. Compute how wrong it is. That wrongness is called the **residual**.
4. Nudge every cell's numbers to make it less wrong.
5. Go back to step 2.

Repeat a few hundred times and the numbers stop moving. That's it. That's CFD.

The **iterations** setting is how many times round that loop you go: 150 for
screening, 400 for balanced, 1000 for accurate. Each pass is a big pile of
arithmetic over every cell, which is why more cells × more iterations = more
minutes.

> **Steady vs unsteady, and why we do the cheap one**
> Real air behind a blunt body is never still — it sheds swirls forever, like
> a flag flapping. Simulating that honestly means marching forward in time and
> is 10–100× more expensive. Instead we solve for the *time-averaged* flow —
> the picture you'd get by leaving a very slow camera shutter open. For drag
> that's the right trade: you want the average push, not the wobble.

### 2.4 Turbulence: the part we don't simulate

Air doesn't flow in neat layers; it tumbles, in swirls of every size from
metres down to fractions of a millimetre. Simulating every swirl would need
cells smaller than the smallest swirl — billions of them, weeks of compute.

So instead we cheat, in a specific, well-understood, 50-year-old way. We add
two extra numbers to every cell:

- **k** — how much swirling energy is in this cell
- **ω** — how fast that swirling energy is being ground down into heat

and we give those two numbers their own guess-and-correct equations. The
tumbling isn't simulated; its *average effect* — extra mixing, effectively
thicker "syrupiness" — is modelled. This particular recipe is called **k-ω
SST**, and it's the default here because it's the standard choice for exactly
this kind of problem: flow that hugs a surface and then separates from it.

This is the single biggest source of error in the whole pipeline. It's also
why the tool runs **two independent solvers** and tells you when they disagree
(see §2.9). A turbulence model is an approximation, and honest software says
so out loud.

The **laminar** option in Advanced turns the model off. That's only physical
for very slow or very small things — a model aircraft, honey — and it will be
badly wrong for a vehicle.

### 2.5 Reading the answer out

While the solver runs, a "function object" (OpenFOAM's word for a plugin)
adds up the pressure and friction on every triangle of your shape, projects
the total onto the wind direction, divides by ½ρV²A, and writes the resulting
Cd to a file, once per iteration.

That file — `postProcessing/forceCoeffsIncompressible/<t>/forceCoeffs.dat` —
has columns `Time Cm Cd Cl Cl(f) Cl(r)`. Cd is the **third** one. This matters
more than it sounds: reading the columns by position instead of by name is how
this repository once reported the *pitching moment* as the drag coefficient
for months.

The last wrinkle: even after the residuals settle, Cd typically still wobbles
by a percent or two, because a bluff body's wake never truly sits still. So
the tool doesn't take the final value — it **averages the last 20% of the
iterations**, and if the remaining spread is more than 5% of the mean it flags
the run as unconverged rather than pretending. See
[`openfoam.CoefficientHistory.averaged`](../src/openfoam.py).

### 2.6 The road, and why it has to move

This one is genuinely subtle and most beginner CFD gets it wrong.

The simulation happens in the **vehicle's frame**: the body is held still and
the air is fired at it. Fine. But what about the ground?

If you leave the ground standing still in that frame, you've built a wind
tunnel floor. Air sticks to it, so a sluggish layer grows along it — thicker
and thicker as it travels — and by the time it reaches your vehicle the air
underneath is moving far too slowly. Your underbody flow is wrong before it
arrives.

A real road doesn't do that, because *the road isn't moving relative to the
air — you are.*

```
   WRONG (static floor)              RIGHT (rolling road)

   air →→→→→→→→→→  [body]           air →→→→→→→→→→  [body]
   air →→→→→→→→                     air →→→→→→→→→→
   air →→→→                         air →→→→→→→→→→
   ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔               ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
   floor stuck at 0                  floor sliding at wind speed
   (fake boundary layer grows)       (no fake layer)
```

So in this tool a road that is switched on can also be **driven along**:

- **Road speed left blank (the default)** — the road matches the wind. This is
  the still-air case: park the atmosphere, and the wind you feel is exactly
  the speed you're doing. It tracks across the whole speed curve.
- **Road speed set to a number** — that's your speed over the ground, pinned.
  This is the windy-day case: 25 m/s over the ground into a 5 m/s headwind is
  30 m/s of wind over a 25 m/s road. Only the road speed can tell those two
  situations apart.

Both backends implement it for real: OpenFOAM gets a translating `fixedValue`
floor with wall functions, SU2 a `MARKER_MOVING` with a
`SURFACE_TRANSLATION_RATE`.

(And if you pin the road speed, Cd stops being one number for the whole speed
curve — the road's share of the picture changes at every other speed — so the
app says so and points you at solving each speed individually.)

### 2.7 Reynolds number: why one solve doesn't give you the whole curve

Naively, `drag = ½ρV²ACd` says drag is just speed squared. Double the speed,
quadruple the drag. So solve once, scale the curve, done.

That's true **only while Cd holds still**, and Cd doesn't. It depends on a
quantity called the **Reynolds number**:

```
    Re = ρ · V · L / μ        L = body length along the flow
                              μ = how sticky the air is
```

Re is, roughly, "how much does momentum win over stickiness here". At low Re
the air is syrupy and hugs your shape; at high Re it's punchy and breaks away.
Somewhere in between — the **transitional band**, about 2×10⁵ to 10⁶ — the
flow flips between the two and Cd can change by tens of percent over a small
speed change. (This is exactly why golf balls have dimples: the dimples trip
the flip early, and Cd drops off a cliff.)

So before solving, the tool computes Re at the slowest and fastest speed you
asked for and picks a strategy:

- **scale** — one solve, curve follows V². Chosen when Re stays in one regime
  and varies by less than 3× across the range.
- **sweep** — a full solve at every speed on the curve. Chosen when the Re
  range touches the transitional band, or varies by more than 3×.

For a vehicle doing 5–20 m/s this virtually always lands on **sweep**: Re
spans 4× and straddles the band. The app tells you which it chose and why, in
that yellow note under the Geometry panel, and you can override it.

### 2.8 Frontal area, measured properly

Remember A in the drag equation — your silhouette. It sounds trivial and it is
the easiest thing in the pipeline to get quietly wrong.

The cheap way, which this repo used to do, is to add up
`triangle_area × max(normal · wind_direction, 0)` over the mesh. For a convex
blob that's exactly right. For anything with a dent, a wheel well, or one
surface hiding behind another, it **counts the hidden surface twice**.

How wrong? On a torus (a doughnut) it over-reports by **62%** — 2.4 m² against
a true 1.479 m². And since Cd = force / (½ρV²·A), a 62% error in A is a 62%
error in every Cd you quote.

So the tool does it properly: project every triangle onto the plane facing the
wind, rasterise them into a 512×512 grid of pixels, and count how many pixels
got covered *at least once*. Overlap counts once. It's the same operation your
graphics card does to draw a triangle, used as a measuring instrument.
([`metrics.frontal_area`](../src/metrics.py))

That's also why the frontal-area tile updates live as you drag the wind
azimuth slider: your silhouette depends on which way you're facing.

### 2.9 Two solvers, on purpose

| Backend | Mesher | Solver | Reads drag from |
|---|---|---|---|
| **OpenFOAM 13** | `blockMesh` (box grid) → `snappyHexMesh` (carve out the body) | `foamRun -solver incompressibleFluid` | `forceCoeffs` function object |
| **SU2 8.4.0** | `gmsh` (tetrahedra) | `SU2_CFD` incompressible RANS | force coefficients `(CFx, CFy, CFz)` projected onto the wind |

They share nothing: different mesh types (hexes vs tetrahedra), different
codebases, different numerics. What they *do* share is the placed geometry,
the flow domain and the reference area — handed to both identically.

That makes the comparison meaningful. If two independent programs, given the
same problem, agree within a few percent, that's real evidence. If they
disagree by 25%, something is wrong with your mesh or your iteration count and
the app says so in plain words rather than letting you pick the answer you
like.

---

## 3. How the shape maker builds a fairing

Now the second machine. Input: a **payload** — the stuff that has to fit
inside, like a rider and three wheels. Output: **one smooth closed shell**
around it, with a chosen gap.

### 3.1 The rule: always exactly one body

The tool will never offer you separate pods around separate lumps. This is a
design decision with a reason:

The gap between two pods is a **channel**. Air squeezing through it grows a
sluggish sticky layer off *both* walls, which usually costs more than the
solid bridge you were trying to avoid. And to simulate that channel at all,
your mesh cells have to be finer than the gap, which explodes the cell count.

So the only remaining question isn't "how many bodies?" — it's "**how much
smoothing does it take to become one body?**" That's a length, not a decision,
and lengths can be searched for automatically.

### 3.2 Turning a shape into Lego

Computers are bad at "smooth blobby shapes" and excellent at "3D grids of
yes/no". So step one is **voxelisation**: lay a 3D grid over the payload
(default 128 cells along its longest axis, so ~16 mm cubes on a 2 m vehicle)
and mark each cell as inside-the-payload or not.

Now the payload is a Minecraft model. Every operation from here is a grid
operation, which means it's fast and has no special cases.

Two safeguards worth knowing about:

- If any separate part is thinner than ~3 voxels, the pitch is automatically
  made finer. A 50 mm wheel on a 2 m vehicle would otherwise *vanish* at the
  default pitch — and if a part vanishes, the count of separate bodies changes,
  and the entire search that follows is answering the wrong question.
- The grid is padded with empty space all around. Everything below is going to
  grow the shape outward, and growing into the edge of the array silently
  chops the result flat.

### 3.3 Shrink-wrap: dilate, then erode

This is the core trick, and it is genuinely simple.

**Dilate by r**: mark every empty cell that is within distance `r` of the
payload as filled. The shape puffs up like a balloon.

**Erode by r**: now un-mark every filled cell that is within distance `r` of
the outside. The shape shrinks back down.

Do both, in that order, and you get a **closing**. Here's the whole point:

```
   original:      ●●●    ●●●        two lumps with a gap

   dilate by r:  ▒●●●▒▒▒▒●●●▒       both puff up and MERGE
                     ▒▒▒▒▒

   erode by r:    ●●●▓▓▓▓●●●        shrink back, but the bridge
                                     that formed stays
```

A gap narrower than about `2r` gets bridged and stays bridged. A gap wider
than that opens back up. Everything else returns to almost exactly where it
was — closing never shrinks a shape, it only ever fills in.

It's the mathematical version of laying cling film over a bowl of fruit: small
dips get spanned, big gaps still show.

> **How it's actually computed** — not by stamping a big ball shape over every
> cell (slow), but with a **distance transform**: one pass that labels every
> cell with "how far is the nearest filled cell". Then "dilate by r" is just
> `distance ≤ r`, a single comparison over the array. Erosion is the same
> trick on the inverted image. Both are linear-time.
> ([`fairing.closed_mask`](../src/fairing.py))

### 3.4 The higher-or-lower game (bisection)

We need the *smallest* r that merges everything into one body — smallest,
because every extra millimetre of r is extra frontal area you're paying for
and getting nothing back.

Here's the property that makes this easy: **the number of separate bodies
never goes up as r goes up.** More smoothing can only merge things, never split
them. So the count looks like a staircase going down, and there's exactly one
threshold where it first hits 1:

```
  bodies
    3 ├●────●────●───●──┐
    2 │                 │
    1 │                 └●─────●─────●──→
      └──────────────────┴──────────────
      0                 67 mm          r
```

That means you can play higher-or-lower instead of trying every value:

1. Try a small r. Still 3 bodies? Double it. Repeat until you hit 1 body.
   Now you have a bracket: some r that fails, some r that works.
2. Try halfway between. 1 body → move the "works" end down. More than 1 →
   move the "fails" end up.
3. Repeat until the bracket is narrower than one voxel.

That's about 5 tries instead of 16 evenly spaced samples, *and* it locates the
threshold far more precisely — which matters, because the shell gets built
just above it. On the sample trike the real log reads:

```
  Closing radius  0 mm gives 3 bodies
  Closing radius 33 mm gives 3 bodies
  Closing radius 67 mm gives 1 body      ← bracket found
  Closing radius 50 mm gives 3 bodies
  Closing radius 58 mm gives 3 bodies    ← tightening
  → merges at 67 mm, shell built at 71 mm
```

(71 = 67 with a 6% safety margin. §3.7 explains why the margin isn't enough on
its own.)

### 3.5 Squashing the ball along the wind

One refinement: the "ball" we dilate with isn't a ball. It's a rugby ball,
stretched three times longer along the wind direction than across it.

Why? Because merging two lumps that sit **one behind the other** is nearly
free — you're filling in space that was already inside your silhouette, and
you're removing a nasty situation where the rear lump sits in the front lump's
wake. Merging two lumps that sit **side by side** means filling the entire
span between them, which is pure new frontal area.

```
   in line (cheap to merge):        side by side (expensive):

     ●●  ●●   →   ●●●●●●              ●●        ●●●●●●●●●●
                                            →
    silhouette unchanged                ●●        (all of that
                                                   is new area)
```

So the streamwise bias (default 3.0) says: bridge in-line gaps eagerly, bridge
spanwise gaps reluctantly. Implemented in one line, by lying to the distance
transform about how far apart the cells are along the flow axis
([`fairing._sampling`](../src/fairing.py)).

### 3.6 From Lego back to a smooth skin

Voxels have staircase edges. We want a smooth surface, and we want the gap
between the payload and the skin to be *exactly* the clearance you asked for.

The trick is to stop thinking about "filled/empty" and build a **signed
distance field** instead: for every point in the grid, store how far it is
from the surface of the closed shape — positive outside, negative inside.

```
   signed distance, one slice through the grid

    +30 +20 +10 ┌───────────┐ +10 +20 +30      the shape is where
    +30 +20 +10 │ -10  -20  │ +10 +20 +30      the field is negative;
    +30 +20 +10 └───────────┘ +10 +20 +30      "30 mm outside" is the
                              ▲                 line where field = 30
                              └── draw the skin here
```

Now "the skin, 30 mm outside the payload" is just **the surface where the
field equals 30** — a *level set*. Finding it is a standard algorithm called
**marching cubes**: walk through the grid, and wherever a cube of eight
neighbouring values straddles the value 30, emit the little triangles that cut
through it at the right place. Because it interpolates between grid values, the
result is smooth to *finer than one voxel* — the staircase is gone.

Two details that took work:

- **The field is blurred, not the mesh.** A distance field computed from
  blocky yes/no data carries ripples at exactly the voxel pitch. Smoothing the
  extracted triangle mesh can only polish locally and is bad at that
  wavelength. Blurring the *field* with a Gaussian kills the ripple at the
  source — and a Gaussian leaves straight-line gradients completely untouched,
  so flat and gently curved regions don't move at all. The blur width is
  capped well below the clearance, so tight corners get rounded by
  millimetres, never eaten.
- **Marching cubes points its triangles the wrong way here**, because the
  field increases outward. The code checks the sign of the enclosed volume and
  flips the whole mesh if needed. Get this wrong and every "is the payload
  inside?" test answers backwards.

### 3.7 Check, don't assume

The search for the merge radius runs on a **coarse** grid (counting bodies
tolerates a blurry payload, and coarse is fast). The final skin is built on a
**fine** grid, because that geometry is what gets meshed and flown.

Here's the catch: the fine grid can *resolve a narrow gap that the coarse grid
had blurred shut*. So the radius that looked like it merged everything can come
out in two pieces once actually built.

So the tool doesn't trust it. After building, it splits the mesh, counts the
bodies, and if there's more than one it opens the radius by 45% and rebuilds —
up to four times — and reports how many attempts it took. It separately
re-checks that the payload really is inside, by ray-casting a few thousand
payload vertices against the shell.

A two-piece "single shell" would mesh into a choked channel and report a drag
coefficient for a shape nobody chose. That's the failure this check exists to
prevent, and "it's guaranteed by the maths" was not a good enough answer,
because smoothing runs *after* the maths.

> One nice touch: `check_containment` returns three values — yes, no, and
> **"couldn't check"**. An earlier version caught every error and returned
> "no", so a missing ray-casting library was reported to the user as *"the
> payload sticks out"*. That is the most damaging thing that function could
> possibly get wrong.

### 3.8 Growing a tail

Everything so far decides *whether it is one body*. It says nothing about the
**profile** — and it can't, because a closing is bounded above by the convex
hull. On an already-convex payload it does literally nothing, and the skin
comes out as "the payload plus 30 mm". A cube in, a rounded cube out. Terrible
thing to fly.

Nothing in "wrap this as tightly as possible" will ever grow a tail, because a
tail is volume the payload doesn't need. So a second stage grows one on
purpose.

The rule comes from a hundred years of streamlining, and it's two numbers:

- **The tail may not taper steeper than ~12°.** Steeper than that and the air
  can't follow the surface inward; it separates, and you get the big swirly
  low-pressure hole that costs you everything the sleek nose saved.
- **The nose may not grow steeper than ~45°.** The nose is far more forgiving,
  because air accelerating around a front is happy to stay attached. A blunt
  nose costs little and saves a lot of length.

So the shell is defined as **the smallest body that contains the payload and
never breaks either rule.** Constructively: take every cross-section of the
payload, and from it cast a shallow cone backwards and a steep cone forwards.
The shell is the union of all those cones.

```
   payload cross-sections            each casts cones

        │█│  │██│  │█│            ╱▔▔▔▔▔▔▔▔▔▔▔▔▔╲
        │█│  │██│  │█│      45°  ╱ █    ██     █  ╲──────────╲  12°
        │█│  │██│  │█│          ╱                              ╲
        ───────────────        ─────────────────────────────────
                                  ↑ steep nose      ↑ shallow tail
                             the outline is the upper envelope
```

A cube goes in, a body of fineness ratio ~3.7 comes out, frontal area
unchanged, and the tail length is set by the 12° limit rather than by taste.

#### Why the shoulders come out sharp

It is not a teardrop, and the diagram above is drawn honestly: there is a
**corner** where each cone meets the payload's own section. Measured on the
sample cube at 30 mm clearance, the profile is

```
   45° cone   │  flat, 1.06 m  │      12° cone
  ───────────╱│                │╲──────────────────────
             ↑                 ↑
        crease here       crease here
   half-height goes 45°→0° in   and 0°→12° in one slice
```

and the transition takes about 90 mm out of a 3.94 m body — a knife edge at
body scale. The cross-sections are *squares* too (the level set of a distance
field inside a square is a smaller square), so the tail is a square pyramid
with four flat triangular faces, not a cone of revolution.

This is not a bug in the smoothing. It follows directly from the word
**minimal**:

- Minimality forces the envelope to equal the payload's own section right up
  to the trailing face — carrying any more section than that, anywhere, is by
  definition not the smallest body containing the payload.
- One voxel further downstream it is already shrinking at exactly tan(12°),
  because that is the fastest it is allowed to shrink and it has no reason to
  shrink slower.

So the surface tangent jumps from 0° to 12° across a single slice. A smooth
shoulder would need the body to start narrowing *before* the payload ends, or
to hold extra section *after* it — both mean deliberately carrying volume the
payload does not force, which is precisely the thing the minimal envelope is
defined to refuse. **Minimality and tangent continuity are in direct
conflict**, and this stage picks minimality.

The two smoothing passes cannot rescue it, by design:

- The Gaussian on the distance field is chosen to *preserve linear fields
  exactly* (§3.6), which is what stops it moving flat and gently curved
  regions. A crease is exactly where two linear pieces meet, so it is rounded
  only over about one voxel.
- Taubin on the mesh is local to a few triangle widths, and its whole point is
  not to shrink the surface into the clearance.

#### So why keep it? (You mostly shouldn't)

Because minimum *volume* was never the objective. The tool ranks on Cd·A, and
the frontal area that normalises is set by the payload's widest section — the
envelope cannot beat it and does not try. Minimality is therefore an
assumption the shape stage inherited, not a criterion anything downstream
asked for, and a crease at the shoulder is the price it charges for it.

So the shoulder treatment is a **profile setting**, not a fixed property:

| | |
|---|---|
| **Faceted** | the minimal envelope — flat panels, creased shoulders. Shortest and smallest body that satisfies the taper limits. |
| **Blended** | the shoulders filled out, tangent-continuous into both the flat and the taper. |

Blended costs wetted area and length and buys tangent continuity. Which wins
is a real question with a real answer, so it is put to the solver: the true
loop searches the **shoulder fill** as a third parameter alongside the two
angles, and its bracket reaches down to zero — so a blended search *contains*
the faceted shape and can hand it back if the fillet does not pay.

> **The setting is a radial fill, and that took two goes.** The obvious knob is
> the streamwise distance the shoulder spreads over — it reads naturally and it
> is what "smooth transition" sounds like. It behaves terribly. Turning 45°
> across a short distance demands a large fillet radius, so the same setting
> swelled the nose 6.5× harder than the tail. Filling by *radius* instead makes
> one number mean the same fill-out at both ends, and it is the quantity that
> actually costs wetted area rather than one that merely correlates with it.
>
> That rewrite also exposed a latent bug worth knowing about. `distance_transform_edt`
> returns **0 outside** the section, and that zero is an absence, not a value —
> maxing it into the carry pinned the field to exactly zero across the whole
> background of every slice the payload appears in. The faceted threshold `> 0`
> hid it by excluding zero *by a hair*, so the field sat flush against the line
> over a huge volume and any outward offset lifted all of it across at once:
> 2 mm of fill swallowed 40% of the grid. The faceted envelope is bit-identical
> before and after the fix, which is exactly why nobody noticed until the fill
> arrived.

> **How the fillet is computed.** A blended shoulder is an arc tangent to the
> flat at one end and to the taper at the other. The trick that makes it
> nearly free: **a concave curve is the infimum of its own tangent lines**, and
> each tangent line is just *a straight taper with an outward offset* — which
> the running-max recursion above already computes exactly. So the fillet is
> the lower envelope of eight tangent tapers, needs no new machinery, and the
> panels it produces are what a real fairing is made of anyway. Two properties
> fall out for free and both are verified in the code: every tangent envelope
> is at least the bare-taper one, so **containment survives**; and the family
> includes the flat tangent, so the blend can never exceed the payload's own
> widest section and **the frontal area does not move**. The fillet is bought
> in wetted area, never in silhouette.
> ([`fairing.shoulder_tangents`](../src/fairing.py))

Measured on the sample cube at 30 mm clearance, tail 12°, nose 45°:

| Shoulder fill | Frontal area | Wetted | Volume | Length |
|---|---|---|---|---|
| 0.00 (faceted) | 1.1268 m² | 11.22 m² | 2.330 m³ | 3.95 m |
| 0.05 | 1.1268 m² | 11.78 m² | 2.476 m³ | 4.09 m |
| 0.10 | 1.1268 m² | 12.28 m² | 2.607 m³ | 4.24 m |
| 0.20 | 1.1268 m² | 13.14 m² | 2.826 m³ | 4.48 m |

Frontal area does not move at all, while the wetted area and volume the fillet
costs climb smoothly. That is the trade in one table, and the reason the loop
is the thing that should settle it.

> **How it's computed exactly.** The obvious way — erode the shape by a
> sliver, slice by slice — fails, because tan(12°) × 16 mm is 3.4 mm, a fifth
> of a voxel, which rounds to nothing and the cone never appears. Instead the
> code carries a distance field along the flow axis, subtracting the taper per
> step and merging with a running maximum. Level sets of a maximum are unions
> of level sets, and level sets of (distance − c) are exact erosions — so the
> result is the exact union of cones with zero accumulated rounding error.
> ([`fairing.streamline_mask`](../src/fairing.py))

A side effect worth knowing: this stage **also merges bodies**, because a
leading lump's tail cone can reach a trailing lump's nose. That's
aerodynamically the right thing, and it's another reason the body count is
taken from the built mesh rather than trusted from the search.

Whether a 12° tail was worth its extra wetted area is **not decided here.**
That's the solver's question. Build the shell, fly it, compare Cd·A. Change
the angles, or untick "Streamlined envelope" (`--no-streamline` on the command
line), and see the difference in numbers.

---

## 4. The shape optimiser: putting the solver inside the loop

12° and 45° are good numbers *in general*. They are not necessarily the right
numbers for **your** payload at **your** speed. The trade they settle — tail
length buys attached flow but costs skin friction — genuinely depends on the
flow, and the only honest way to settle it is to fly the candidates.

That's the **True loop** option in the shape-solver dropdown.
([`src/shapeopt.py`](../src/shapeopt.py))

### 4.1 What it does

```
   build shell at (nose=45, tail=12)  →  solve  →  Cd·A = 0.2186   ← baseline
   build shell at (nose=45, tail=12.1) →  solve  →  Cd·A = ...
   build shell at (nose=45, tail=17.9) →  solve  →  Cd·A = ...
                        ...  ~10 solves total ...
   keep the best one, write its angles back into the run's knobs
```

Each step builds a real shell, runs a real CFD solve (screening quality, one
backend — fast and consistent), and reads Cd·A. Expect minutes to an hour,
watchable line by line in the progress log with a measured ETA from the first
solve onward.

### 4.2 Searching without wasting solves: golden section

Every evaluation costs minutes, so you can't afford to be sloppy. The tool
uses **golden-section search**, which is higher-or-lower for finding the
*bottom of a valley* rather than a threshold.

You have a bracket `[6°, 22°]` that you believe contains the best tail angle.
Probe two points inside it, at about 38% and 62% of the way across:

```
   6°        12.1°        15.9°           22°
   ├──────────┼─────────────┼─────────────┤
   a          x1            x2            b
              │             │
            Cd·A          Cd·A
            0.219         0.226      ← x1 is better, so the minimum
                                        cannot be to the right of x2
   6°        12.1°     15.9°
   ├──────────┼─────────┤              new bracket, 62% as wide
```

The clever part is the 0.618 ratio: it's chosen so that one of the two probes
lands exactly where a probe already is, so **each new bracket costs one solve,
not two.** After N solves the bracket is 0.618^N as wide — ten solves shrinks
16° down to about 1.2°.

Why golden section rather than something cleverer like gradient descent?

- It shrinks the bracket by a guaranteed factor every single solve. You know
  the budget before you start, which is what the ETA needs.
- A screening-quality solve carries a couple of percent of noise. Gradient
  methods can be sent off a cliff by that; the worst golden section does is
  stop a degree early.

The tool searches **tail first**, then nose with the tail held at its best —
because the tail dominates the trade and deserves the bigger share of a small
budget — then the tail again around the measured nose if the budget allows,
since the passes are only separable to the extent the knobs don't interact.

### 4.2b The trap: the search mesh is not the judging mesh

"A couple of percent of noise" is the optimistic reading, and on a real
payload it is wrong. Here is the sample trike, four shells differing only in
tail angle, each solved at **both** qualities. Frontal area is constant to
four decimal places **by construction** — the envelope can't exceed the
payload's widest section — so every difference below is pure Cd:

| Tail | Cd screening | Cd balanced | error |
|---|---|---|---|
| 9° | 0.4850 | 0.4581 | +5.9% |
| 12° | **0.6629** | 0.4606 | **+43.9%** |
| 16° | **0.3860** | 0.4470 | **−13.6%** |
| 20° | 0.4097 | 0.4199 | −2.4% |

The balanced column spans 9.7% across the whole bracket and moves smoothly:
that is the aerodynamics, and it is a **modest effect**. The screening column
spans 72% and isn't even monotone — going 9° → 12° → 16° it reports drag
going **up 37%, then down 42%** on a shape changing smoothly.

Signal ~10%, measurement error up to 44%. **The search is fitting noise.** Its
answer is close to random within the bracket, which is exactly why the loop
sometimes returns something better than the heuristic and sometimes something
worse. Screening picks 16° here; balanced says 20°.

Now the part that makes this genuinely nasty: **that 0.6629 solve converged.**
It wasn't oscillating, it raised no warning, it settled confidently on a
number 44% wrong — because a 26-cell mesh cannot resolve a long shallow
afterbody, and a converged solve on an inadequate mesh is just a stable wrong
answer. No convergence check will ever catch it. Only a finer mesh will.

Left alone, the loop would hand you 16° and you'd find out later.

So the loop **confirms**. After the search it solves its own winner *and* the
heuristic shell at the run's own quality, keeps whichever really wins, and
reports a revert honestly:

```
   search (screening):  16° wins at Cd·A 0.1832
   confirm (balanced):  16° → 0.xxxx   vs   12° → 0.xxxx
                        └── if 12° wins, keep it and say so
```

Two extra solves, charged *outside* the search budget — the budget buys
search, and a tight one must not silently drop the check. This is what makes
"the loop can never hand back something worse than the heuristic" a statement
about the mesh you read rather than the mesh it searched.

But be clear about what the confirmation can and cannot do. It can **refuse a
bad answer**. It cannot **manufacture a good one** — if the search ranked
noise, the best it can do is hand back the heuristic shell. So on a payload
like the trike, raise **Search quality** to balanced so the search ranks on a
mesh that can see a 10% effect. That is much slower and it is the honest fix;
the confirmation is the guard, not a substitute.

It is worth it here: at balanced quality a 20° tail beats the 12° heuristic by
**8.9% on Cd·A**. There is a real gain on this body — screening just cannot
find it, because its error bars are four times the prize.

Three further guards, two of which the trike actually trips:

- **Split shells are rejected unsolved.** At a 21.5° tail the trike's shell
  comes out in two pieces — the steeper tail cone stops reaching between the
  lumps. A split shell meshes into a choked channel and returns a *flattering*
  coefficient for a shape nobody chose: the most dangerous candidate a search
  can meet, because the number looks fine.
- **Shells that stop enclosing the payload are rejected unsolved.** The same
  21.5° shell fails this too.
- **Unconverged solves are flagged.** When a third or more of the search was
  still oscillating, the run says so, because then the ranking is noise
  wearing a tidy log. This is a *different* failure from the one above and
  does not catch it: the 12° screening solve converged cleanly and was still
  44% wrong.

### 4.3 Why only two knobs?

This is the design decision that makes the whole thing affordable, so it's
worth spelling out.

"Real" aerodynamic shape optimisation moves every vertex of the mesh
independently. That needs *adjoint gradients* (a second, backwards solve that
tells you the sensitivity of drag to every surface point) and typically
hundreds of iterations. It's a research project, not a button.

This searches the **shell generator's own two parameters** instead. The
consequences are all good:

- **Every candidate is buildable and contains the payload by construction.**
  No solve is ever wasted on a shape that turns out to be infeasible.
- **The space is 2D and, in practice, single-valleyed** — more tail is
  monotonically less pressure drag and more friction — so a dozen solves get
  you within a degree or two of the bottom.
- **The result is two numbers**, not a mesh delta. You can write it down,
  reproduce it, and start the next derivation from it.

### 4.4 It cannot make things worse

The heuristic shell is flown first, as the baseline. The answer returned is the
**argmin over everything flown, baseline included**. So the loop can never hand
back something worse than what the taper rules gave you for free — the worst
case is that it spends its budget confirming the heuristic was already right,
which is itself a useful thing to learn.

The measured angles are written into the run's own knobs (so deriving again
starts from them), recorded with the gain over the heuristic, and the full
evaluation history is kept in the run file.

---

## 5. The app around it

### 5.1 Runs, and why a solved one never changes

The unit of work is a **run**: one shape, the parameters it was given, and the
results that came back. Each tab is one.

**A run stops changing once it has been solved.** That single rule settles a
whole family of confusing situations:

- Edit the wind speed on a solved run and the knob is flagged with the value
  the run was *actually* solved at, with a link back. The chart stays on
  screen — it's still a real measurement — but never without saying what it
  belongs to.
- Press Compute on a solved run and it doesn't overwrite anything; it opens a
  **new** run carrying your edits, auto-titled *"Re-run of trike · drag #1
  with a different wind speed"*.
- Only the run being solved has its parameters frozen. Every other tab stays
  fully readable and editable while the solver grinds.
- Every run records where its shape came from — an imported file, or the shell
  of a shape run — so the chain from payload to final hull is always readable.

The mechanism is the **as-run snapshot** ([`src/runs.py`](../src/runs.py)):
the moment the solver is handed a scene, every parameter that could affect the
answer is copied and frozen. Anything not on that list (the run's title, its
description) can be edited freely without making the results stale.

### 5.2 The queue

CFD wants the whole machine. So there's exactly one solver at a time and
everything else lines up in a queue, which is itself a pinned tab: what's on
the machine now, what's waiting, in what order. Waiting runs can be stopped,
paused or reordered; the one already running has to finish or be killed.

### 5.3 Time estimates that learn

Every run is estimated before it starts, from a deliberately simple model:
cost ≈ cells × iterations, plus a fixed overhead for meshing and process
startup.

The interesting part is that the constants aren't shipped with the tool.
Every completed solve appends `(solver, cells, iterations, seconds)` to
`runtime_history.json`, and later predictions are fitted to *those*
measurements. A machine slower than the built-in guess converges on a correct
ETA after its first solve instead of insisting on the factory number. During a
run, the remaining time is also rescaled by how the run is actually going.

### 5.4 Where the solvers actually run

Both backends run their solver as an external Linux process reached through a
shim. There are three shims, probed in this order:

| Mode | Wins when | Why you'd want it |
|---|---|---|
| `native` | the solver is on `PATH` | fastest — no virtualisation layer |
| `docker` | a pinned image is present | reproducible: OpenFOAM 13 and SU2 8.4.0 exactly, not whatever the machine has |
| `wsl` | an existing WSL install | the original path, still supported |

`AERO_EXECUTION=docker` pins one instead of probing.
`python src/runner.py info` prints what each backend resolved to.

### 5.5 Parallel ranks, and the surprise

The solve gets **80% of the visible cores** by default — not all of them,
because the mesher, the GUI and the OS still need one, and an oversubscribed
MPI run is slower than a correctly sized one. "Visible" means what
`sched_getaffinity` and the cgroup CPU quota say, not `os.cpu_count()`, which
matters when the tool itself is in a container.

The surprise, measured on the sample cube (42k cells, 150 iterations,
OpenFOAM through WSL, 8-core laptop):

| Ranks | Wall time |
|---|---|
| 1 | 28.9 s |
| 2 | 29.7 s |
| 6 (the 80% default) | **37.8 s** |

More ranks is **slower** at screening size, because `decomposePar` and
`reconstructPar` are serial, MPI startup is a fixed cost, and the actual solve
is only a few seconds of a run dominated by meshing. Ranks earn their keep at
accurate quality, where there are far more cells. For screening sweeps, pin
`--processes 1`.

### 5.6 Why the containers are CPU-only

Not an oversight:

- OpenFOAM's GPU paths (PETSc4FOAM, AmgX) live on the **ESI** fork, not the
  Foundation build this tool drives. Adopting them means porting the backend
  and re-validating every Cd.
- They accelerate only the linear solve, so Amdahl's law caps the win at about
  1.5–2× — and only past ~1M cells, where the GPU has enough work to beat
  transfer and launch latency. The largest case here is 0.27M.
- `snappyHexMesh` and `gmsh` have no GPU path at all, and the measurements say
  meshing and fixed overhead *are* the runtime: a 266k-cell / 400-iteration
  solve came in at 9.7 s against 9–15 s for 42k cells at 150 iterations.

Cores are the lever that actually moves.

---

## 6. A worked example, end to end

Real numbers, from `sample2.stl` (a mock tadpole trike: a reclined rider
envelope plus three wheels), road at 0.15 m ride height, 15 m/s, OpenFOAM.

**Step 1 — load the payload.** 832 triangles, four parts (capsule + three
wheels, of which three read as separate lumps once voxelised), bounding box
2.03 × 0.89 × 0.83 m. The tool rasterises the silhouette: **frontal area
0.3012 m²**. It computes Re from 6.9×10⁵ to 2.8×10⁶ over 5–20 m/s, notices
that overlaps the transitional band, and recommends a **sweep**.

**Step 2 — measure its drag.** blockMesh lays the background grid, snappyHexMesh
carves the trike out of it, `foamRun` iterates 400 times, `forceCoeffs` writes
Cd each iteration, the last 20% get averaged:

```
   Cd = 0.4071    A = 0.3012 m²    →   Cd·A = 0.1226 m²
   drag at 15 m/s = ½ × 1.225 × 15² × 0.1226 = 16.9 N
```

**Step 3 — derive a shape.** Voxelise at 16.7 mm pitch into a 221×152×149 grid.
Sweep the closing radius by bisection: 0 mm → 3 bodies, 33 mm → 3, 67 mm → 1,
then tighten with 50 and 58. **Merges at 67 mm.** Build at 71 mm (6% margin),
apply the 45°/12° taper envelope, extract the 30 mm level set with marching
cubes, smooth, verify:

```
   bodies:        1  ✓ (rebuilt 0 times — the fine grid agreed)
   payload fit:   encloses  ✓
   watertight:    yes  ✓
   frontal area:  0.4747 m²   (+58% — this is the bill)
   ref length:    2.77 m      (was 2.03 — the tail)
   volume:        0.640 m³
   triangles:     64,336
```

**Step 4 — measure the shell.**

```
   Cd = 0.4606    A = 0.4747 m²    →   Cd·A = 0.2186 m²
```

**The verdict: the shell loses on this payload,** by +78% on drag area — and
the reason is entirely in the frontal area, not the shape. Bridging the gaps
between the rider capsule and three wheels is what made one body out of three,
and every bridge is silhouette that used to be open air. The tool charged 58%
more area for that merge; the streamlining did not buy 58% back. (At screening
quality the same pair shows the shell's Cd 15% *below* the payload's, which is
the streamlining doing its job — just not by enough.)

That is a completely legitimate result, and it is the whole point of the tool:
it *measured* instead of assuming. Where do you go from here?

- **Attack the frontal area, because that is where the loss is.** Reduce the
  clearance from 30 mm — area scales roughly with clearance around the whole
  silhouette. Or move the payload: pull the wheels in, and the bridges the
  closing has to build get shorter and cheaper.
- **Raise the streamwise bias** above 3.0 so in-line gaps close before
  spanwise ones. Bridging front-to-back costs almost no silhouette; bridging
  side-to-side costs all of it (§3.5).
- **Fly the true loop.** 12° may be far from optimal for this body; the loop
  will find out for a handful of solves.
- **Untick "Streamlined envelope"** and compare — is the tail earning anything
  at all here, or is the raw packaging skin better?
- **Try it on a payload with a genuinely bluff front**, which is where wrapping
  pays. A seated rider with no envelope, an open frame, exposed luggage.

---

## 7. Glossary

| Term | In one sentence |
|---|---|
| **Cd** | The "shape score" — drag divided by ½ρV²A. No units. Lower is better. |
| **Cd·A** | Drag area. Cd times frontal area. **The number that decides**, because Cd alone can be improved by getting bigger. |
| **Frontal area (A)** | Your silhouette seen from the wind, measured by rasterising the projected triangles so overlaps count once. |
| **Wetted area** | Total surface area. Drives friction drag. |
| **Reynolds number (Re)** | How much momentum wins over stickiness. Changes Cd, which is why one solve doesn't always give you the whole speed curve. |
| **Cell / mesh** | The little boxes the air is chopped into, and the pile of them. |
| **Iteration** | One pass of the guess-and-correct loop over every cell. |
| **Residual** | How badly the current guess violates physics. Should fall as iterations go up. |
| **Converged** | The numbers stopped moving. Here: the last 20% of Cd values spread by less than 5%. |
| **k-ω SST** | The recipe for modelling turbulence's average effect instead of simulating every swirl. |
| **Boundary layer** | The thin sluggish film of air stuck to any surface. |
| **Separation** | When that film gives up and peels away, leaving a big draggy wake. What the 12° tail limit exists to prevent. |
| **Voxel** | A cell in the 3D yes/no grid the shape maker works on. |
| **Dilate / erode / closing** | Puff up / shrink down / both in order. Closing bridges small gaps and leaves big ones. |
| **Signed distance field** | A grid storing "how far to the surface", negative inside. Makes exact offsets easy. |
| **Level set** | The surface where the field equals some value — e.g. "30 mm outside". |
| **Marching cubes** | The algorithm that turns a level set into triangles. |
| **Bisection** | Higher-or-lower search for a threshold. |
| **Golden-section search** | Higher-or-lower search for the bottom of a valley, at one solve per step. |
| **Payload** | The stuff that must fit inside the fairing. |
| **Shell / fairing** | The single closed body wrapped around the payload. |
| **Run** | One shape + its parameters + its results. One tab. Immutable once solved. |

---

## 8. Not black magic: the cheat sheet

| Looks like magic | Is actually |
|---|---|
| "It simulated the airflow" | Four numbers per box, nudged a few hundred times until they stop changing |
| "It knows the turbulence" | It doesn't. It models the *average effect* of turbulence with two extra numbers per box, using a 50-year-old recipe, and runs a second independent solver to check |
| "It found the drag coefficient" | It added up pressure and friction over every triangle, projected onto the wind, and divided by ½ρV²A — then averaged the last 20% of the iterations because the answer still wobbles |
| "It measured the frontal area" | It drew the silhouette into a 512×512 grid of pixels and counted the ones that got filled |
| "It decided to solve every speed" | It computed Reynolds number at both ends of your speed range and checked whether that interval overlaps a known band |
| "It wrapped my payload automatically" | Voxel grid, puff up by r, shrink back by r, count the separate pieces, play higher-or-lower on r until the count hits 1 |
| "It knew how much to smooth" | Bisection on a staircase function that is guaranteed monotone, so there is exactly one threshold to find |
| "It made a streamlined teardrop" | Every cross-section casts a 45° cone forward and a 12° cone back; the shell is the union — cones, so the shoulders are creases, not fillets |
| "It optimised the shape with CFD" | It walked two numbers (nose angle, tail angle) with golden-section search, ~10 solves, keeping the best — including the starting point, so it can't lose |
| "It knows how long it will take" | Cells × iterations × a rate constant it fitted to your own machine's past solves, stored in `runtime_history.json` |
| "It verified the shell is one body" | It literally split the mesh and counted, then rebuilt at a bigger radius if the count was wrong |
| "It verified the payload fits" | It ray-cast a few thousand payload vertices against the shell and checked every one landed inside |

Every one of those is a page or two of ordinary code in [`src/`](../src/).
Nothing in this repository does anything you couldn't do by hand with enough
graph paper and enough weekends.

---

*Where to go next: [the README](../README.md) for installing and running it;
[`src/fairing.py`](../src/fairing.py), [`src/shapeopt.py`](../src/shapeopt.py)
and [`src/solvers.py`](../src/solvers.py) all open with a long docstring
explaining the module's own reasoning.*
