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

The objective is Cd.A at the reference speed, screening preset, one backend.
Ranking candidates against each other needs consistency far more than
absolute accuracy -- the same reasoning the compare step uses -- and the
winning shell gets its proper multi-backend, accurate solve afterwards like
any other run.

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
TAIL_SHARE_BLENDED = 0.45

# Fewest new solves a pass needs before it is descending rather than merely
# bracketing: golden section spends its first two probes on the bracket.
MIN_USEFUL_PASS = 3

# Angles closer than this are the same shape once meshed: the shell moves by
# less than the voxel pitch. Also the cache key granularity.
ANGLE_RESOLUTION_DEG = 0.5
# The blend is a fraction of the half-width, so its resolution is relative
# too: 0.05 of a half-width is millimetres of shoulder on any real payload.
BLEND_RESOLUTION = 0.05

# Bracket widths each pass shrinks to before it calls the parameter settled.
TAIL_TOLERANCE_DEG = 1.5
NOSE_TOLERANCE_DEG = 4.0
BLEND_TOLERANCE = 0.12

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
    # Shoulder blend length as a fraction of the cross-flow half-width. Zero
    # is the faceted envelope, so a blended search that finds nothing better
    # than a crease can say so in the same units.
    blend: float = 0.0

    def to_dict(self) -> dict:
        return {
            "nose_deg": self.nose_deg,
            "tail_deg": self.tail_deg,
            "blend": self.blend,
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
    blend_bracket: tuple[float, float] | None = None
    at_bracket_edge: list[str] = field(default_factory=list)

    @property
    def improvement(self) -> float | None:
        """Fractional Cd.A gain over the heuristic angles, negative is better."""
        if not self.baseline.drag_area or not self.best.drag_area:
            return None
        return (self.best.drag_area - self.baseline.drag_area) / self.baseline.drag_area

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
            "blend_bracket": list(self.blend_bracket) if self.blend_bracket else None,
            "at_bracket_edge": list(self.at_bracket_edge),
        }


def _quantise(value: float) -> float:
    return round(value / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG


def _quantise_blend(value: float) -> float:
    return round(value / BLEND_RESOLUTION) * BLEND_RESOLUTION


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


def _screening_scene(scene: Scene, shell_mesh: trimesh.Trimesh, backend: str) -> Scene:
    """The parent run's conditions around a candidate shell, set up to rank.

    Same wind, road and fluid as the run that asked -- the optimum depends on
    them -- but screening quality, one solve scaled, one backend. The loop
    compares candidates against each other, so consistency is what matters;
    the winner gets its accurate multi-backend solve later like any run.
    """
    trial = scene.without_results()
    trial.geometry = Geometry.from_bytes(
        shell_mesh.export(file_type="stl"), source_name="refine_candidate.stl"
    )
    trial.payload = None
    trial.fairing = None
    trial.solver.apply_preset("screening")
    trial.solver.sweep_mode = "scale"
    trial.solver.backends = [backend]
    return trial


def _drag_area(results) -> tuple[float | None, float | None, float | None, str]:
    for solver_run in results.runs:
        if solver_run.status != "ok":
            return None, None, None, solver_run.message or f"{solver_run.solver} failed"
        point = solver_run.reference_point()
        if point is None:
            return None, None, None, f"{solver_run.solver} returned no reference point"
        return (
            point.drag_coefficient * point.frontal_area,
            point.drag_coefficient,
            point.frontal_area,
            "",
        )
    return None, None, None, "no solver ran"


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
    blended = packaging.envelope_profile == "blended"

    def emit(message: str) -> None:
        if progress is not None:
            progress({"phase": "optimise", "message": message})

    def label(key: tuple[float, float, float]) -> str:
        text = f"tail {key[1]:.1f}°, nose {key[0]:.1f}°"
        return f"{text}, blend {key[2]:.2f}" if blended else text

    def build_shell(nose: float, tail: float, blend: float) -> fairing.Shell:
        fine = fairing.build_grid(
            payload_mesh,
            direction=direction,
            resolution=packaging.resolution,
            anisotropy=packaging.anisotropy,
            streamline=(nose, tail),
            clearance=packaging.clearance,
            shoulder_blend=blend,
        )
        return fairing.build_single_shell(
            coarse_grid,
            payload_mesh,
            sweep,
            direction=direction,
            clearance=packaging.clearance,
            build_grid_override=fine,
            streamline=(nose, tail),
            shoulder_blend=blend,
        )

    def evaluate(
        nose: float, tail: float, blend: float, shell: fairing.Shell | None = None
    ) -> Evaluation:
        key = (_quantise(nose), _quantise(tail), _quantise_blend(blend if blended else 0.0))
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

        if shell.contains_payload is False:
            # The envelope is extensive by construction, so this is the
            # smoothing having eaten a tight corner. Whatever such a shell's
            # Cd.A is, it is not a number for a shape that holds the payload,
            # and letting it win would hand back a fairing the thing does not
            # fit inside. Rank it behind every real candidate instead of
            # spending a solve on it.
            evaluation = Evaluation(
                key[0], key[1], None, None, None, time.time() - started,
                "the shell did not enclose the payload", key[2],
            )
            evaluations[key] = evaluation
            history.append(evaluation)
            emit(
                f"[{len(history)}/{max_solves}] {label(key)} "
                "rejected: the shell did not enclose the payload"
            )
            return evaluation

        trial = _screening_scene(scene, shell.mesh, backend)
        try:
            results = solve(trial, backends=[backend])
            drag_area, cd, area, message = _drag_area(results)
        except Exception as error:
            drag_area, cd, area, message = None, None, None, f"{type(error).__name__}: {error}"

        evaluation = Evaluation(
            nose_deg=key[0],
            tail_deg=key[1],
            drag_area=drag_area,
            drag_coefficient=cd,
            frontal_area=area,
            solve_seconds=time.time() - started,
            message=message,
            blend=key[2],
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
                f"→ Cd·A {drag_area:.4f} m² (best {best_now.drag_area:.4f}){eta}"
            )
        else:
            emit(f"[{len(history)}/{max_solves}] {label(key)} failed: {message}")
        return evaluation

    def score(evaluation: Evaluation) -> float:
        # Failed solves rank behind every real number, so the search backs
        # away from whatever broke the mesher instead of chasing it.
        return evaluation.drag_area if evaluation.drag_area is not None else float("inf")

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
    blend0 = float(packaging.blend)

    # Both angle brackets straddle the heuristic values, so every pass can
    # lengthen the shell as readily as it shorten it.
    tail_bracket = bracket_around(tail0, TAIL_REACH, "tail")
    nose_bracket = bracket_around(nose0, NOSE_REACH, "nose")
    # The blend bracket always reaches down to zero, which *is* the faceted
    # envelope. So a blended search contains the sharp-shouldered shape as a
    # candidate and can hand it back if the fillet does not pay -- the profile
    # choice is a starting point, not a verdict the loop is made to defend.
    blend_bracket = fairing.blend_bounds() if blended else None

    plan = "tail first, then nose" + (", then the shoulder blend" if blended else "")
    ranges = (
        f"Searching tail {tail_bracket[0]:.0f}–{tail_bracket[1]:.0f}°, "
        f"nose {nose_bracket[0]:.0f}–{nose_bracket[1]:.0f}°"
    )
    if blended:
        ranges += f", blend {blend_bracket[0]:.2f}–{blend_bracket[1]:.2f}"
    emit(
        f"True loop: {max_solves} screening solves with {backend}, {plan}. "
        f"Heuristic start: tail {tail0:.0f}°, nose {nose0:.0f}°"
        + (f", blend {blend0:.2f}" if blended else "")
        + f". {ranges}."
    )
    baseline = evaluate(nose0, tail0, blend0, shell=baseline_shell)
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
    passes = 3 if blended else 2
    left = max(budget["left"], 0)
    if left >= passes * MIN_USEFUL_PASS:
        share = TAIL_SHARE if passes == 2 else TAIL_SHARE_BLENDED
        tail_cap = min(
            max(MIN_USEFUL_PASS, int(round(share * left))),
            left - (passes - 1) * MIN_USEFUL_PASS,
        )
    else:
        tail_cap = left
    golden_section(
        *tail_bracket,
        run_eval=lambda tail: evaluate(nose0, tail, blend0),
        min_width=TAIL_TOLERANCE_DEG,
        cap=tail_cap,
    )
    tail_best = best_so_far().tail_deg

    # Nose: flatter, cheaper, searched at the best tail found. It inherits
    # whatever the tail pass left unspent -- converging early is a saving, not
    # a forfeit -- minus a reserve for the passes still to come.
    remaining_passes = 2 if blended else 1
    reserve = min(MIN_USEFUL_PASS * remaining_passes, budget["left"] // 2)
    golden_section(
        *nose_bracket,
        run_eval=lambda nose: evaluate(nose, tail_best, blend0),
        min_width=NOSE_TOLERANCE_DEG,
        cap=budget["left"] - reserve,
    )
    nose_best = best_so_far().nose_deg

    # The shoulder blend, at the angles measured so far. It goes last because
    # it is the one parameter that cannot change the frontal area -- the
    # fillet is bounded by the payload's own widest section by construction --
    # so it trades pure wetted area against pure separation, and reading that
    # trade is only meaningful once the body it sits on has stopped moving.
    blend_best = blend0
    if blended and budget["left"] > 1:
        reserve = min(MIN_USEFUL_PASS, budget["left"] // 3)
        golden_section(
            *blend_bracket,
            run_eval=lambda blend: evaluate(nose_best, tail_best, blend),
            min_width=BLEND_TOLERANCE,
            cap=budget["left"] - reserve,
        )
        blend_best = best_so_far().blend

    # The passes are sequential, so the tail was settled against the heuristic
    # nose and no blend at all. Spend anything left re-fitting it around the
    # winner: the passes are only separable to the extent the parameters do
    # not interact, and this is what tests that rather than assuming it.
    if budget["left"] > 1 and (nose_best != nose0 or blend_best != blend0):
        span = tail_bracket[1] - tail_bracket[0]
        refit = (
            max(tail_bracket[0], tail_best - 0.25 * span),
            min(tail_bracket[1], tail_best + 0.25 * span),
        )
        golden_section(
            *refit,
            run_eval=lambda tail: evaluate(nose_best, tail, blend_best),
            min_width=TAIL_TOLERANCE_DEG,
            cap=budget["left"],
        )

    best = best_so_far()
    shell = best_shell["shell"] if best_shell["shell"] is not None else baseline_shell

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
    if blended and _at_edge(best.blend, blend_bracket, BLEND_TOLERANCE) == "high":
        edges.append(
            f"the shoulder blend is at the top of its range "
            f"({blend_bracket[0]:.2f}–{blend_bracket[1]:.2f})"
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
        blend_bracket=blend_bracket,
        at_bracket_edge=edges,
    )
    gain = result.improvement
    emit(
        f"Loop done after {result.solves} solves: tail {best.tail_deg:.1f}°, "
        f"nose {best.nose_deg:.1f}°"
        + (f", blend {best.blend:.2f}" if blended else "")
        + f", Cd·A {best.drag_area:.4f} m²"
        + (f" ({gain * 100:+.1f}% vs the heuristic angles)" if gain is not None else "")
    )
    if blended and best.blend <= BLEND_TOLERANCE:
        emit(
            "The blend came back at zero: on this payload the faceted shoulder "
            "measured no worse than any fillet the loop tried."
        )
    for note in edges:
        emit(
            f"Note: {note}, so the optimum may lie past it. Set that value "
            "closer to the edge and derive again to search further."
        )
    return result
