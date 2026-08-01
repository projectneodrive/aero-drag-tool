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

The search is two golden-section passes: tail first because it dominates the
trade, then nose with the tail held at its best. Golden section over pattern
descent because every evaluation costs minutes: it guarantees bracket
shrinkage per solve, tolerates the few-percent noise a screening solve
carries (the worst case is stopping a degree early, not diverging), and its
budget is known before it starts, which is what the ETA needs.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import trimesh

import fairing
from scene import Geometry, Scene
from solvers import run_scene


# The bracket the loop searches, in degrees from the flow axis. Deliberately
# wider than the heuristic's comfort zone -- finding the optimum at an edge is
# a finding -- but inside the envelope maths' own validity bounds.
TAIL_BOUNDS = (6.0, 22.0)
NOSE_BOUNDS = (25.0, 70.0)

# Angles closer than this are the same shape once meshed: the shell moves by
# less than the voxel pitch. Also the cache key granularity.
ANGLE_RESOLUTION_DEG = 0.5

GOLDEN = 0.6180339887498949


@dataclass
class Evaluation:
    """One flown candidate: the angles, and what the solver said."""

    nose_deg: float
    tail_deg: float
    drag_area: float | None  # None when the solve failed
    drag_coefficient: float | None
    frontal_area: float | None
    solve_seconds: float
    message: str = ""

    def to_dict(self) -> dict:
        return {
            "nose_deg": self.nose_deg,
            "tail_deg": self.tail_deg,
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
        }


def _quantise(value: float) -> float:
    return round(value / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG


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
    evaluations: dict[tuple[float, float], Evaluation] = {}
    history: list[Evaluation] = []
    best_shell: dict = {"key": None, "shell": None}
    budget = {"left": int(max_solves)}

    def emit(message: str) -> None:
        if progress is not None:
            progress({"phase": "optimise", "message": message})

    def build_shell(nose: float, tail: float) -> fairing.Shell:
        fine = fairing.build_grid(
            payload_mesh,
            direction=direction,
            resolution=packaging.resolution,
            anisotropy=packaging.anisotropy,
            streamline=(nose, tail),
            clearance=packaging.clearance,
        )
        return fairing.build_single_shell(
            coarse_grid,
            payload_mesh,
            sweep,
            direction=direction,
            clearance=packaging.clearance,
            build_grid_override=fine,
            streamline=(nose, tail),
        )

    def evaluate(nose: float, tail: float, shell: fairing.Shell | None = None) -> Evaluation:
        key = (_quantise(nose), _quantise(tail))
        if key in evaluations:
            return evaluations[key]
        if budget["left"] <= 0:
            # Out of solves: report the worst plausible value so the search
            # simply stops moving that way instead of raising mid-bracket.
            return Evaluation(key[0], key[1], None, None, None, 0.0, "budget exhausted")
        budget["left"] -= 1

        started = time.time()
        if shell is None:
            shell = build_shell(*key)
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
                f"[{len(history)}/{max_solves}] tail {key[1]:.1f}°, nose {key[0]:.1f}° "
                f"→ Cd·A {drag_area:.4f} m² (best {best_now.drag_area:.4f}){eta}"
            )
        else:
            emit(f"[{len(history)}/{max_solves}] tail {key[1]:.1f}°, nose {key[0]:.1f}° failed: {message}")
        return evaluation

    def score(evaluation: Evaluation) -> float:
        # Failed solves rank behind every real number, so the search backs
        # away from whatever broke the mesher instead of chasing it.
        return evaluation.drag_area if evaluation.drag_area is not None else float("inf")

    def golden_section(low: float, high: float, run_eval, min_width: float) -> None:
        """Shrink [low, high] around the minimum until the bracket is tight.

        Plain golden section, with two twists: evaluations are cached at the
        angle resolution so a re-probed point is free, and the loop stops
        when the bracket is inside ``min_width`` or the budget runs out --
        whichever bites first.
        """
        a, b = low, high
        x1 = b - GOLDEN * (b - a)
        x2 = a + GOLDEN * (b - a)
        f1, f2 = score(run_eval(x1)), score(run_eval(x2))
        while (b - a) > min_width and budget["left"] > 0:
            if f1 <= f2:
                b, x2, f2 = x2, x1, f1
                x1 = b - GOLDEN * (b - a)
                f1 = score(run_eval(x1))
            else:
                a, x1, f1 = x1, x2, f2
                x2 = a + GOLDEN * (b - a)
                f2 = score(run_eval(x2))

    # ---------------------------------------------------------------- search
    nose0 = float(packaging.nose_angle_deg)
    tail0 = float(packaging.tail_angle_deg)

    emit(
        f"True loop: {max_solves} screening solves with {backend}, tail first, then nose. "
        f"Heuristic start: tail {tail0:.0f}°, nose {nose0:.0f}°."
    )
    baseline = evaluate(nose0, tail0, shell=baseline_shell)
    if baseline.drag_area is None:
        raise RuntimeError(
            f"The baseline shell failed to solve ({baseline.message}); "
            "the loop has nothing to compare against."
        )

    # Tail: the dominant trade, gets the larger share of the budget.
    golden_section(
        *TAIL_BOUNDS,
        run_eval=lambda tail: evaluate(nose0, tail),
        min_width=1.5,
    )
    finite = [e for e in history if e.drag_area is not None]
    tail_best = min(finite, key=lambda e: e.drag_area).tail_deg

    # Nose: flatter, cheaper, searched at the best tail found.
    golden_section(
        *NOSE_BOUNDS,
        run_eval=lambda nose: evaluate(nose, tail_best),
        min_width=4.0,
    )

    finite = [e for e in history if e.drag_area is not None]
    best = min(finite, key=lambda e: e.drag_area)
    shell = best_shell["shell"] if best_shell["shell"] is not None else baseline_shell

    result = RefineResult(
        best=best,
        baseline=baseline,
        history=history,
        solves=len(history),
        backend=backend,
        shell=shell,
    )
    gain = result.improvement
    emit(
        f"Loop done after {result.solves} solves: tail {best.tail_deg:.1f}°, "
        f"nose {best.nose_deg:.1f}°, Cd·A {best.drag_area:.4f} m²"
        + (f" ({gain * 100:+.1f}% vs the heuristic angles)" if gain is not None else "")
    )
    return result
