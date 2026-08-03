"""CFD-in-the-loop refinement of the fairing envelope: the true loop.

The heuristic stage picks the envelope's two angles by rule of thumb -- a 12
degree tail because that is where afterbodies classically separate, a 45
degree nose because accelerating flow forgives bluntness. Those numbers are
right *in general*; they are not right for this payload at this Reynolds
number. The trade they settle -- tail length buys attached flow but costs
wetted area -- genuinely depends on the flow, and the only honest way to
settle it is to fly the candidates.

So that is all this module does: build the shell at candidate angles, solve
it, read Cd.A, walk downhill, repeat. The critical design decision is *what
to optimise*. Free-form vertex optimisation needs adjoint gradients and
hundreds of solves; this walks the **generator's own two parameters**, where

- every candidate is buildable and contains the payload by construction, so
  no solve is ever wasted on an infeasible shape;
- the search space is two-dimensional and unimodal in practice (more tail is
  monotonically less pressure drag and more friction), so a dozen solves get
  within a degree or two of the optimum;
- the result is reproducible from two numbers rather than a mesh delta.

The objective is Cd.A at the reference speed, one backend, at the search
quality (screening by default). Ranking candidates against each other needs
consistency far more than absolute accuracy -- the same reasoning the compare
step uses.

Consistency is not sufficient on its own, though, and this is the trap the
module fell into once already. A ranking taken on a coarse mesh transfers to
a fine one only if the two *agree on the order*, and on a body whose entire
design variable is the length of a shallow afterbody they need not: a long
shallow tail is exactly what a coarse mesh resolves worst.

Measured on the sample trike, tails of 9 / 12 / 16 / 20 degrees with the
frontal area constant to four decimals by construction, so every difference is
pure Cd:

    screening   0.4850   0.6629   0.3860   0.4097     spread 72%
    balanced    0.4581   0.4606   0.4470   0.4199     spread 9.7%

Balanced is the aerodynamics and it is a modest effect. Screening is not even
monotone, and at 12 degrees it is 44% high. Signal ~10%, error ~44%: the
search is fitting noise, and its answer is near random within the bracket --
which is why it lands better than the heuristic on some runs and worse on
others.

Note what does *not* help: that 0.6629 solve **converged**. It raised no
warning and settled confidently on a wrong number, because a converged solve
on an inadequate mesh is a stable wrong answer. The convergence flag catches a
different failure and will never catch this one.

So the loop closes with a confirmation: the winner and the heuristic shell are
both solved at the run's own quality, whichever really wins is kept, and a
revert is reported as such. Two solves, charged outside the search budget --
the budget buys search, and a tighter one must not silently drop the check.
That is what makes "never worse than the heuristic" a statement about the
mesh the user reads rather than the mesh the search used.

The confirmation can only *refuse* a bad answer, never manufacture a good one.
When the search itself ranked noise, the honest fix is ``refine_quality``:
search on the mesh you judge on. On the trike that is worth having -- at
balanced a 20 degree tail beats the 12 degree heuristic by 8.9% on Cd.A, a
real gain screening cannot find because its error bars are four times it.

The search is golden-section passes: tail first because it dominates the
trade, then nose with the tail held at its best, then the tail re-fitted
around the measured nose if any budget is left -- the passes are only
separable to the extent the two angles do not interact, and the refit is what
tests that rather than assuming it. Golden section over pattern descent
because every evaluation costs minutes: it guarantees bracket shrinkage per
solve, tolerates the few-percent noise a screening solve carries (the worst
case is stopping a degree early, not diverging), and its budget is known
before it starts, which is what the ETA needs.

Every bracket is built *around the angles the caller started from*, never
fixed, so both directions are always reachable: the loop can lengthen the
heuristic shell and shorten it, and neither is privileged by the bounds. Each
candidate is rebuilt from the original payload -- not from the heuristic shell
-- so containment is re-established from scratch at every step rather than
inherited from a shape that is being changed underneath it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import trimesh

import execution
import fairing
from scene import Geometry, Scene
from solvers import run_scene


# How far the search reaches either side of the angles it starts from, as a
# multiple of them. The bracket is built *around the start* rather than fixed,
# which is what lets the loop both add to the heuristic shell and subtract
# from it: a shallower tail is a longer body, a steeper one a shorter body,
# and either has to be reachable for the answer to mean anything. Fixed bounds
# could not promise that -- a user-chosen 25 degree tail fell outside a
# hard-coded [6, 22], so every candidate flown was shorter than the shell it
# started from and the loop could only ever cut.
#
# On the defaults these reproduce the brackets the loop already used: a 12
# degree tail gives [6.0, 21.6], a 45 degree nose [24.8, 72.0].
TAIL_REACH = (0.5, 1.8)
NOSE_REACH = (0.55, 1.6)

# Floor on the bracket width, so a start angle sitting near a validity bound
# still has room to move on both sides rather than a sliver on one.
MIN_SPAN_DEG = {"tail": 6.0, "nose": 14.0}

# How the solve budget is divided once the baseline is flown. The tail
# dominates the trade so it searches first and widest, but the nose is never
# starved to nothing -- with one shared pool a tight budget spent everything
# on the tail and handed back the heuristic's nose angle without ever having
# flown an alternative to it.
TAIL_SHARE = 0.6
# With a blend to search as well, the tail yields some of its share rather
# than the other two splitting what is left of a budget it already took.
TAIL_SHARE_FILLED = 0.45

# Fewest new solves a pass needs before it is descending rather than merely
# bracketing: golden section spends its first two probes on the bracket.
MIN_USEFUL_PASS = 3

# Angles closer than this are the same shape once meshed: the shell moves by
# less than the voxel pitch. Also the cache key granularity.
ANGLE_RESOLUTION_DEG = 0.5
# The blend is a fraction of the half-width, so its resolution is relative
# too: 0.05 of a half-width is millimetres of shoulder on any real payload.
FILL_RESOLUTION = 0.05

# Bracket widths each pass shrinks to before it calls the parameter settled.
TAIL_TOLERANCE_DEG = 1.5
NOSE_TOLERANCE_DEG = 4.0
FILL_TOLERANCE = 0.12

GOLDEN = 0.6180339887498949


@dataclass
class Evaluation:
    """One flown candidate: the shape parameters, and what the solver said."""

    nose_deg: float
    tail_deg: float
    drag_area: float | None  # None when the solve failed
    drag_coefficient: float | None
    frontal_area: float | None
    solve_seconds: float
    message: str = ""
    # Shoulder fill as a fraction of the cross-flow half-width. Zero is the
    # faceted envelope, so a filled search that finds nothing better than a
    # crease can say so in the same units.
    fill: float = 0.0
    # Did this solve settle? An unconverged candidate still carries a number,
    # but the ranking it takes part in is worth less than it looks.
    converged: bool = True

    def to_dict(self) -> dict:
        return {
            "nose_deg": self.nose_deg,
            "tail_deg": self.tail_deg,
            "fill": self.fill,
            "converged": self.converged,
            "drag_area": self.drag_area,
            "drag_coefficient": self.drag_coefficient,
            "frontal_area": self.frontal_area,
            "solve_seconds": self.solve_seconds,
            "message": self.message,
        }


@dataclass
class RefineResult:
    """The loop's outcome: the best shell and the full audit trail."""

    best: Evaluation
    baseline: Evaluation
    history: list[Evaluation] = field(default_factory=list)
    solves: int = 0
    backend: str = ""
    shell: fairing.Shell = field(repr=False, default=None)
    # What the search was allowed to consider, so a result can be read against
    # its own bounds rather than as if the whole angle range had been flown.
    tail_bracket: tuple[float, float] | None = None
    nose_bracket: tuple[float, float] | None = None
    fill_bracket: tuple[float, float] | None = None
    at_bracket_edge: list[str] = field(default_factory=list)
    # What the search ranked on, and what the winner was then checked at.
    search_quality: str = "screening"
    confirm_quality: str | None = None
    confirmed_best: float | None = None
    confirmed_baseline: float | None = None
    # The search's winner lost at the finer mesh, so the heuristic shell was
    # kept. Not a failure -- the loop doing exactly what it promises.
    reverted_to_baseline: bool = False
    # The shell that was actually handed back, measured at the run's own
    # quality: Cd, frontal area, Cd.A. This is a real solve of the real shell,
    # so the caller can show the achieved drag without asking for it again.
    delivered_point: dict | None = None

    @property
    def improvement(self) -> float | None:
        """Fractional Cd.A gain of the shell actually handed back.

        Describes what was *delivered*, not what was considered. A reverted
        loop delivers the heuristic shell, so its gain is exactly zero -- the
        margin by which its own candidate lost is real information, but it
        belongs to ``rejected_margin``, not here. Reporting the loss as the
        outcome would say the loop made the shape worse when it did the
        opposite.

        Quoted from the confirmation solves when there are any: those are the
        pair measured at the quality the run is read at, and a gain that only
        exists on the search mesh is not a gain.
        """
        if self.reverted_to_baseline:
            return 0.0
        if self.confirmed_best is not None and self.confirmed_baseline:
            return (self.confirmed_best - self.confirmed_baseline) / self.confirmed_baseline
        if not self.baseline.drag_area or not self.best.drag_area:
            return None
        return (self.best.drag_area - self.baseline.drag_area) / self.baseline.drag_area

    @property
    def rejected_margin(self) -> float | None:
        """How badly the search's own winner lost at the confirmation quality.

        Only meaningful on a revert, where it is the size of the disagreement
        between the search mesh and the judging one -- i.e. how much the proxy
        was worth trusting. A large value is the signal to raise the search
        quality rather than to distrust the loop.
        """
        if not self.reverted_to_baseline:
            return None
        if self.confirmed_best is None or not self.confirmed_baseline:
            return None
        return (self.confirmed_best - self.confirmed_baseline) / self.confirmed_baseline

    def to_dict(self) -> dict:
        return {
            "best": self.best.to_dict(),
            "baseline": self.baseline.to_dict(),
            "history": [item.to_dict() for item in self.history],
            "solves": self.solves,
            "backend": self.backend,
            "improvement": self.improvement,
            "tail_bracket": list(self.tail_bracket) if self.tail_bracket else None,
            "nose_bracket": list(self.nose_bracket) if self.nose_bracket else None,
            "fill_bracket": list(self.fill_bracket) if self.fill_bracket else None,
            "at_bracket_edge": list(self.at_bracket_edge),
            "search_quality": self.search_quality,
            "confirm_quality": self.confirm_quality,
            "confirmed_best": self.confirmed_best,
            "confirmed_baseline": self.confirmed_baseline,
            "reverted_to_baseline": self.reverted_to_baseline,
            "rejected_margin": self.rejected_margin,
            "delivered_point": self.delivered_point,
        }


def _quantise(value: float) -> float:
    return round(value / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG


def _quantise_fill(value: float) -> float:
    return round(value / FILL_RESOLUTION) * FILL_RESOLUTION


def bracket_around(start: float, reach: tuple[float, float], kind: str) -> tuple[float, float]:
    """Search bracket straddling ``start``, inside the generator's own bounds.

    Built around the angle the caller actually started from, so both
    directions are always reachable. The clamp to ``fairing.angle_bounds`` is
    what keeps the loop from spending solves outside them, where the envelope
    builder clamps every candidate to the same shape and the search would
    read a flat objective as a plateau.

    ``start`` itself is kept inside the returned bracket even when the clamps
    would have excluded it, so the baseline shell is always a point the search
    can come back to.
    """
    low_bound, high_bound = fairing.angle_bounds(kind)
    start = float(min(max(start, low_bound), high_bound))

    low, high = start * reach[0], start * reach[1]
    span = MIN_SPAN_DEG[kind]
    if high - low < span:
        middle = 0.5 * (low + high)
        low, high = middle - 0.5 * span, middle + 0.5 * span

    low = max(low, low_bound)
    high = min(high, high_bound)
    return (min(low, start), max(high, start))


def _at_edge(value: float, bracket: tuple[float, float], tolerance: float) -> str:
    """Name the bracket edge ``value`` is sitting on, if it is on one.

    A minimum found at an edge is not a minimum -- it is the search running
    into a wall it was told not to cross -- and saying so is the difference
    between a measured angle and an artefact of the bounds.
    """
    if value - bracket[0] <= tolerance:
        return "low"
    if bracket[1] - value <= tolerance:
        return "high"
    return ""


def _trial_scene(
    scene: Scene, shell_mesh: trimesh.Trimesh, backend: str, quality: str
) -> Scene:
    """The parent run's conditions around a candidate shell, set up to rank.

    Same wind, road and fluid as the run that asked -- the optimum depends on
    them -- but one solve scaled and one backend, at ``quality``. The loop
    compares candidates against each other, so consistency is what matters.

    Consistency is *not* sufficient on its own, though, which is what the
    confirmation stage downstream exists for: a ranking taken at a coarse mesh
    only transfers to a fine one if the two agree on the order, and on a body
    whose whole point is a long shallow tail they need not. Screening resolves
    exactly that tail worst.
    """
    trial = scene.without_results()
    trial.geometry = Geometry.from_bytes(
        shell_mesh.export(file_type="stl"), source_name="refine_candidate.stl"
    )
    trial.payload = None
    trial.fairing = None
    trial.solver.apply_preset(quality)
    trial.solver.sweep_mode = "scale"
    trial.solver.backends = [backend]
    return trial


def _drag_area(results) -> tuple[float | None, float | None, float | None, str, bool]:
    """Cd.A at the reference point, and whether the solve actually settled.

    The convergence flag is returned rather than folded into the number
    because an unconverged solve is not a failure -- it produced a
    coefficient, and discarding it would leave the search with nothing on a
    bluff body at screening iterations. It is a *ranking* hazard: two solves
    still swinging by several percent differ by their noise as much as by
    their shapes, so a search that cannot see the flag will happily chase it.
    """
    for solver_run in results.runs:
        if solver_run.status != "ok":
            return None, None, None, solver_run.message or f"{solver_run.solver} failed", False
        point = solver_run.reference_point()
        if point is None:
            return None, None, None, f"{solver_run.solver} returned no reference point", False
        converged = bool(getattr(solver_run, "converged", True))
        return (
            point.drag_coefficient * point.frontal_area,
            point.drag_coefficient,
            point.frontal_area,
            "" if converged else "still oscillating at the last iteration",
            converged,
        )
    return None, None, None, "no solver ran", False


def refine_envelope(
    scene: Scene,
    payload_mesh: trimesh.Trimesh,
    coarse_grid: fairing.PayloadGrid,
    sweep: fairing.SweepResult,
    backend: str,
    direction,
    baseline_shell: fairing.Shell,
    max_solves: int = 10,
    progress=None,
    solve=None,
) -> RefineResult:
    """Tune the envelope's (nose, tail) angles with the solver in the loop.

    ``baseline_shell`` is the heuristic shell the caller already built; it is
    flown first, both as the reference the improvement is quoted against and
    so the loop can never return something worse than the heuristic -- the
    final answer is the argmin over everything flown, baseline included.

    ``solve`` is injectable so the loop's mechanics can be tested against a
    synthetic objective without a CFD licence attached. Resolved at call
    time, not in the signature, so patching this module's ``run_scene`` works.
    """
    if solve is None:
        solve = run_scene
    packaging = scene.packaging
    evaluations: dict[tuple[float, float, float], Evaluation] = {}
    history: list[Evaluation] = []
    best_shell: dict = {"key": None, "shell": None}
    budget = {"left": int(max_solves)}
    filled = packaging.envelope_profile == "blended"
    # What the search ranks on, and what the answer will be read at. When they
    # differ the ranking is a proxy, and the confirmation stage checks it.
    search_quality = packaging.refine_quality
    final_quality = scene.solver.quality

    def emit(message: str) -> None:
        if progress is not None:
            progress({"phase": "optimise", "message": message})

    def label(key: tuple[float, float, float]) -> str:
        text = f"tail {key[1]:.1f}°, nose {key[0]:.1f}°"
        return f"{text}, fill {key[2]:.2f}" if filled else text

    def build_shell(nose: float, tail: float, fill: float) -> fairing.Shell:
        fine = fairing.build_grid(
            payload_mesh,
            direction=direction,
            resolution=packaging.resolution,
            anisotropy=packaging.anisotropy,
            streamline=(nose, tail),
            clearance=packaging.clearance,
            shoulder_fill=fill,
        )
        return fairing.build_single_shell(
            coarse_grid,
            payload_mesh,
            sweep,
            direction=direction,
            clearance=packaging.clearance,
            build_grid_override=fine,
            streamline=(nose, tail),
            shoulder_fill=fill,
        )

    def evaluate(
        nose: float, tail: float, fill: float, shell: fairing.Shell | None = None
    ) -> Evaluation:
        key = (_quantise(nose), _quantise(tail), _quantise_fill(fill if filled else 0.0))
        if key in evaluations:
            return evaluations[key]
        if budget["left"] <= 0:
            # Out of solves: report the worst plausible value so the search
            # simply stops moving that way instead of raising mid-bracket.
            return Evaluation(key[0], key[1], None, None, None, 0.0, "budget exhausted", key[2])
        budget["left"] -= 1

        started = time.time()
        # Building a candidate shell is in-process work between two solves, so
        # it is where a stop would otherwise go unnoticed for minutes.
        execution.checkpoint()
        if shell is None:
            shell = build_shell(*key)

        # Two ways a candidate can be disqualified before it is worth a solve,
        # and both really happen: on the sample trike a 21.5 degree tail comes
        # out in two pieces *and* stops enclosing the payload, because a
        # steeper tail cone no longer reaches from one lump to the next and
        # the merge has to be re-opened past what the grid can hold.
        reject = ""
        if shell.bodies > 1:
            # A split shell meshes into a choked channel between the pieces
            # and reports a coefficient for a shape nobody chose -- often a
            # flattering one, since the gap is a low-pressure region the
            # coarse screening mesh barely resolves. That is the single most
            # dangerous candidate the search can meet: plausible number,
            # meaningless shape.
            reject = f"the shell came out as {shell.bodies} separate bodies"
        elif shell.contains_payload is False:
            # The envelope is extensive by construction, so this is the
            # smoothing having eaten a tight corner. Whatever such a shell's
            # Cd.A is, it is not a number for a shape that holds the payload,
            # and letting it win would hand back a fairing the thing does not
            # fit inside.
            reject = "the shell did not enclose the payload"

        if reject:
            # Ranked behind every real candidate rather than solved, so the
            # search backs away from it instead of chasing it.
            evaluation = Evaluation(
                key[0], key[1], None, None, None, time.time() - started, reject, key[2],
            )
            evaluations[key] = evaluation
            history.append(evaluation)
            emit(f"[{len(history)}/{max_solves}] {label(key)} rejected: {reject}")
            return evaluation

        trial = _trial_scene(scene, shell.mesh, backend, search_quality)
        try:
            results = solve(trial, backends=[backend])
            drag_area, cd, area, message, converged = _drag_area(results)
        except Exception as error:
            drag_area, cd, area, message = None, None, None, f"{type(error).__name__}: {error}"
            converged = False

        evaluation = Evaluation(
            nose_deg=key[0],
            tail_deg=key[1],
            drag_area=drag_area,
            drag_coefficient=cd,
            frontal_area=area,
            solve_seconds=time.time() - started,
            message=message,
            fill=key[2],
            converged=converged,
        )
        evaluations[key] = evaluation
        history.append(evaluation)

        if drag_area is not None:
            finite = [e for e in history if e.drag_area is not None]
            best_now = min(finite, key=lambda e: e.drag_area)
            if best_now is evaluation:
                best_shell["key"] = key
                best_shell["shell"] = shell
            # The first solve prices all the others: quote the remaining time
            # from measured solves, not from a model.
            eta = ""
            if budget["left"] > 0:
                average = sum(e.solve_seconds for e in finite) / len(finite)
                eta = f" · ≈{max(average * budget['left'] / 60.0, 1.0):.0f} min left"
            emit(
                f"[{len(history)}/{max_solves}] {label(key)} "
                f"→ Cd·A {drag_area:.4f} m²"
                + ("" if converged else " (unconverged)")
                + f" (best {best_now.drag_area:.4f}){eta}"
            )
        else:
            emit(f"[{len(history)}/{max_solves}] {label(key)} failed: {message}")
        return evaluation

    def score(evaluation: Evaluation) -> float:
        # Failed solves rank behind every real number, so the search backs
        # away from whatever broke the mesher instead of chasing it.
        return evaluation.drag_area if evaluation.drag_area is not None else float("inf")

    def _confirm(shell: fairing.Shell, what: str) -> dict | None:
        """Measure one shell at the run's own quality, outside the budget.

        Deliberately not charged to ``max_solves``: that budget buys search,
        and this is the check on what the search returned. Charging it would
        make a tighter budget silently drop the check.

        Returns the whole point rather than just Cd.A, because this *is* a
        real measurement of a real shell at the quality the run is read at --
        the same solve the user would otherwise run by hand afterwards.
        Throwing away Cd and the area only to re-derive them later would be
        asking the solver the same question twice.
        """
        execution.checkpoint()
        trial = _trial_scene(scene, shell.mesh, backend, final_quality)
        try:
            drag_area, cd, area, message, converged = _drag_area(
                solve(trial, backends=[backend])
            )
        except Exception as error:
            drag_area, cd, area, converged = None, None, None, False
            message = f"{type(error).__name__}: {error}"
        if drag_area is None:
            emit(f"Confirmation solve for {what} failed: {message}")
            return None
        emit(
            f"Confirmation: {what} is Cd·A {drag_area:.4f} m² "
            f"(Cd {cd:.4f}, A {area:.4f} m²) at {final_quality}."
            + ("" if converged else " Unconverged.")
        )
        return {
            "drag_area": drag_area,
            "drag_coefficient": cd,
            "frontal_area": area,
            "converged": converged,
            "quality": final_quality,
            "backend": backend,
        }

    def golden_section(low: float, high: float, run_eval, min_width: float, cap: int) -> None:
        """Shrink [low, high] around the minimum until the bracket is tight.

        Plain golden section, with three twists: evaluations are cached at the
        angle resolution so a re-probed point is free, ``cap`` bounds the new
        solves this pass may spend so a later pass is never starved, and the
        loop stops when the bracket is inside ``min_width``, the pass's cap is
        reached or the whole budget runs out -- whichever bites first.

        Cached hits deliberately do not count against ``cap``: they cost
        nothing, so charging for them would shorten a pass for free.
        """
        if cap <= 0 or high - low <= min_width:
            return
        start_left = budget["left"]
        a, b = low, high
        x1 = b - GOLDEN * (b - a)
        x2 = a + GOLDEN * (b - a)
        f1, f2 = score(run_eval(x1)), score(run_eval(x2))
        while (b - a) > min_width and budget["left"] > 0 and (start_left - budget["left"]) < cap:
            if f1 <= f2:
                b, x2, f2 = x2, x1, f1
                x1 = b - GOLDEN * (b - a)
                f1 = score(run_eval(x1))
            else:
                a, x1, f1 = x1, x2, f2
                x2 = a + GOLDEN * (b - a)
                f2 = score(run_eval(x2))

    def best_so_far() -> Evaluation:
        finite = [item for item in history if item.drag_area is not None]
        return min(finite, key=lambda item: item.drag_area)

    # ---------------------------------------------------------------- search
    nose0 = float(packaging.nose_angle_deg)
    tail0 = float(packaging.tail_angle_deg)
    fill0 = float(packaging.fill)

    # Both angle brackets straddle the heuristic values, so every pass can
    # lengthen the shell as readily as it shorten it.
    tail_bracket = bracket_around(tail0, TAIL_REACH, "tail")
    nose_bracket = bracket_around(nose0, NOSE_REACH, "nose")
    # The blend bracket always reaches down to zero, which *is* the faceted
    # envelope. So a blended search contains the sharp-shouldered shape as a
    # candidate and can hand it back if the fillet does not pay -- the profile
    # choice is a starting point, not a verdict the loop is made to defend.
    fill_bracket = fairing.fill_bounds() if filled else None

    plan = "tail first, then nose" + (", then the shoulder fill" if filled else "")
    ranges = (
        f"Searching tail {tail_bracket[0]:.0f}–{tail_bracket[1]:.0f}°, "
        f"nose {nose_bracket[0]:.0f}–{nose_bracket[1]:.0f}°"
    )
    if filled:
        ranges += f", fill {fill_bracket[0]:.2f}–{fill_bracket[1]:.2f}"
    emit(
        f"True loop: {max_solves} screening solves with {backend}, {plan}. "
        f"Heuristic start: tail {tail0:.0f}°, nose {nose0:.0f}°"
        + (f", fill {fill0:.2f}" if filled else "")
        + f". {ranges}."
    )
    baseline = evaluate(nose0, tail0, fill0, shell=baseline_shell)
    if baseline.drag_area is None:
        raise RuntimeError(
            f"The baseline shell failed to solve ({baseline.message}); "
            "the loop has nothing to compare against."
        )

    # Split what the baseline left. The tail searches first and widest because
    # it dominates the trade; every later pass gets a reserved share rather
    # than the remainder, so a tight budget cannot spend the lot on the tail
    # and return an unexamined nose angle as if it had been measured.
    #
    # Below one useful pass each there is nothing to split: golden section
    # spends its first probes establishing a bracket and only then starts
    # descending, so halving a small budget buys two passes that bracket and
    # neither that descends. There the tail takes the lot, which is the same
    # order of preference, just with nothing left over.
    passes = 3 if filled else 2
    left = max(budget["left"], 0)
    if left >= passes * MIN_USEFUL_PASS:
        share = TAIL_SHARE if passes == 2 else TAIL_SHARE_FILLED
        tail_cap = min(
            max(MIN_USEFUL_PASS, int(round(share * left))),
            left - (passes - 1) * MIN_USEFUL_PASS,
        )
    else:
        tail_cap = left
    golden_section(
        *tail_bracket,
        run_eval=lambda tail: evaluate(nose0, tail, fill0),
        min_width=TAIL_TOLERANCE_DEG,
        cap=tail_cap,
    )
    tail_best = best_so_far().tail_deg

    # Nose: flatter, cheaper, searched at the best tail found. It inherits
    # whatever the tail pass left unspent -- converging early is a saving, not
    # a forfeit -- minus a reserve for the passes still to come.
    #
    # The blend is a primary parameter and reserves a full pass; the refit is
    # a bonus and only gets what is spare. Reserving for the refit as though
    # it were primary takes solves from the nose, which is a real design
    # variable, to re-check one that has already been searched.
    reserve = (MIN_USEFUL_PASS if filled else 0) + min(2, budget["left"] // 4)
    reserve = min(reserve, max(budget["left"] - MIN_USEFUL_PASS, 0))
    golden_section(
        *nose_bracket,
        run_eval=lambda nose: evaluate(nose, tail_best, fill0),
        min_width=NOSE_TOLERANCE_DEG,
        cap=budget["left"] - reserve,
    )
    nose_best = best_so_far().nose_deg

    # The shoulder blend, at the angles measured so far. It goes last because
    # it is the one parameter that cannot change the frontal area -- the
    # fillet is bounded by the payload's own widest section by construction --
    # so it trades pure wetted area against pure separation, and reading that
    # trade is only meaningful once the body it sits on has stopped moving.
    fill_best = fill0
    if filled and budget["left"] > 1:
        reserve = min(MIN_USEFUL_PASS, budget["left"] // 3)
        golden_section(
            *fill_bracket,
            run_eval=lambda fill: evaluate(nose_best, tail_best, fill),
            min_width=FILL_TOLERANCE,
            cap=budget["left"] - reserve,
        )
        fill_best = best_so_far().fill

    # The passes are sequential, so the tail was settled against the heuristic
    # nose and no blend at all. Spend anything left re-fitting it around the
    # winner: the passes are only separable to the extent the parameters do
    # not interact, and this is what tests that rather than assuming it.
    if budget["left"] > 1 and (nose_best != nose0 or fill_best != fill0):
        span = tail_bracket[1] - tail_bracket[0]
        refit = (
            max(tail_bracket[0], tail_best - 0.25 * span),
            min(tail_bracket[1], tail_best + 0.25 * span),
        )
        golden_section(
            *refit,
            run_eval=lambda tail: evaluate(nose_best, tail, fill_best),
            min_width=TAIL_TOLERANCE_DEG,
            cap=budget["left"],
        )

    best = best_so_far()
    shell = best_shell["shell"] if best_shell["shell"] is not None else baseline_shell

    # ------------------------------------------------------------ confirm
    # The search ranked candidates at `search_quality`; this shell will be
    # read at the run's own. Those are different meshes, and a ranking taken
    # on a coarse one transfers to a fine one only if the two agree on the
    # order -- which, on a body whose entire design variable is the length of
    # a shallow tail, they need not: a long shallow afterbody is precisely
    # what a coarse mesh resolves worst. Measured on the sample trike, the
    # orderings disagree.
    #
    # "The loop can never hand back something worse than the heuristic" is
    # therefore only worth stating if it is checked at the quality it will be
    # read at. Two solves buy that, and the fallback makes the promise true
    # rather than merely intended.
    best_point = baseline_point = delivered = None
    confirmed_best = confirmed_baseline = None
    reverted = False
    changed = (best.nose_deg, best.tail_deg, best.fill) != (
        _quantise(nose0), _quantise(tail0), _quantise_fill(fill0),
    )
    if changed and final_quality != search_quality:
        emit(
            f"Confirming at {final_quality} quality: the search ranked at "
            f"{search_quality}, and a coarser mesh need not order a long tail "
            "the same way a finer one does. Two solves."
        )
        best_point = _confirm(shell, "the loop's shell")
        baseline_point = _confirm(baseline_shell, "the heuristic shell")
        confirmed_best = best_point["drag_area"] if best_point else None
        confirmed_baseline = baseline_point["drag_area"] if baseline_point else None
        delivered = best_point
        if confirmed_best is not None and confirmed_baseline is not None:
            delta = (confirmed_best - confirmed_baseline) / confirmed_baseline
            if confirmed_best > confirmed_baseline:
                delivered = baseline_point
                reverted = True
                shell = baseline_shell
                best = baseline
                emit(
                    f"At {final_quality} quality the loop's shell is "
                    f"{delta * 100:+.1f}% on Cd·A, so the heuristic shell is kept. "
                    f"The screening ranking did not survive the finer mesh — raise "
                    f"'Search quality' to {final_quality} to search on the mesh you "
                    "judge on, at the cost of a slower loop."
                )
            else:
                emit(
                    f"Confirmed at {final_quality} quality: {delta * 100:+.1f}% "
                    "Cd·A against the heuristic shell."
                )
        else:
            emit(
                "The confirmation solves did not both succeed, so the result is "
                f"the {search_quality} ranking only — treat it as provisional."
            )

    # A winner sitting on a bracket edge means the search hit its bounds, not
    # a minimum. Say so: the honest reading is "at least this far", and the
    # user can move the starting angle and derive again to go further.
    edges = []
    tail_edge = _at_edge(best.tail_deg, tail_bracket, TAIL_TOLERANCE_DEG)
    nose_edge = _at_edge(best.nose_deg, nose_bracket, NOSE_TOLERANCE_DEG)
    if tail_edge:
        edges.append(
            f"the tail is at the {tail_edge} end of its search range "
            f"({tail_bracket[0]:.0f}–{tail_bracket[1]:.0f}°)"
        )
    if nose_edge:
        edges.append(
            f"the nose is at the {nose_edge} end of its search range "
            f"({nose_bracket[0]:.0f}–{nose_bracket[1]:.0f}°)"
        )
    # Only the top of the blend range is worth flagging. Landing on zero is
    # not a wall -- it is the loop reporting that this payload wanted the
    # faceted shoulder after all, which is a result rather than a limit.
    if filled and _at_edge(best.fill, fill_bracket, FILL_TOLERANCE) == "high":
        edges.append(
            f"the shoulder fill is at the top of its range "
            f"({fill_bracket[0]:.2f}–{fill_bracket[1]:.2f})"
        )

    result = RefineResult(
        best=best,
        baseline=baseline,
        history=history,
        solves=len(history),
        backend=backend,
        shell=shell,
        tail_bracket=tail_bracket,
        nose_bracket=nose_bracket,
        fill_bracket=fill_bracket,
        at_bracket_edge=edges,
        search_quality=search_quality,
        confirm_quality=final_quality if confirmed_best is not None else None,
        confirmed_best=confirmed_best,
        confirmed_baseline=confirmed_baseline,
        reverted_to_baseline=reverted,
        delivered_point=delivered,
    )
    gain = result.improvement
    # Quote one mesh per sentence. Mixing the search's Cd.A with a confirmed
    # percentage reads as though both came off the same solve, which is the
    # confusion this whole stage exists to remove.
    if reverted:
        margin = result.rejected_margin
        emit(
            f"Loop done after {result.solves} solves: keeping the heuristic shell "
            f"(tail {best.tail_deg:.1f}°, nose {best.nose_deg:.1f}°"
            + (f", fill {best.fill:.2f}" if filled else "")
            + f"). Its Cd·A at {final_quality} is {confirmed_baseline:.4f} m²"
            + (
                f", against {confirmed_best:.4f} m² for what the {search_quality} "
                f"search picked — {margin * 100:+.1f}%."
                if margin is not None
                else "."
            )
        )
    else:
        quality = result.confirm_quality or search_quality
        value = confirmed_best if confirmed_best is not None else best.drag_area
        emit(
            f"Loop done after {result.solves} solves: tail {best.tail_deg:.1f}°, "
            f"nose {best.nose_deg:.1f}°"
            + (f", fill {best.fill:.2f}" if filled else "")
            + f", Cd·A {value:.4f} m² at {quality}"
            + (f" ({gain * 100:+.1f}% vs the heuristic angles)" if gain is not None else "")
        )
    if filled and best.fill <= FILL_TOLERANCE and not reverted:
        emit(
            "The fill came back at zero: on this payload the faceted shoulder "
            "measured no worse than any fillet the loop tried."
        )
    # A search made mostly of unconverged solves has ranked noise, and the
    # angles it reports are not measurements however tidy the log looks.
    flown = [item for item in history if item.drag_area is not None]
    unsettled = [item for item in flown if not item.converged]
    if flown and len(unsettled) * 3 >= len(flown):
        emit(
            f"Warning: {len(unsettled)} of {len(flown)} candidates were still "
            f"oscillating at the last iteration, so much of this ranking is "
            f"solver noise rather than shape. Raise the iteration count, or the "
            f"search quality above {search_quality}, before trusting these angles."
        )
    for note in edges:
        emit(
            f"Note: {note}, so the optimum may lie past it. Set that value "
            "closer to the edge and derive again to search further."
        )
    return result
