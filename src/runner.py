"""Command line front end: build scenes, compute them, inspect the results.

This is the offline half of the tool. The GUI writes a scene file; this runs
it on whatever machine has the solvers installed and writes the results back
into a file of the same format, which the GUI can then load.

    python runner.py info
    python runner.py new --stl hull.stl --ground 0.15 -o case.aero.json
    python runner.py run case.aero.json -o case.solved.json --solver openfoam --solver su2
    python runner.py show case.solved.json
    python runner.py export case.aero.json --dir cases/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from scene import Scene, Wind
from execution import available_cores, default_processes
from solvers import available_solvers, run_scene


def _parse_vector(text: str) -> np.ndarray:
    parts = [float(value.strip()) for value in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma separated numbers, e.g. 10,0,0")
    return np.array(parts, dtype=float)


def _apply_common_options(scene: Scene, args: argparse.Namespace) -> Scene:
    if getattr(args, "wind", None) is not None:
        scene.wind = Wind.from_dict({"vector": args.wind.tolist(), "speed": None})
    if getattr(args, "speed", None) is not None:
        scene.wind.speed = float(args.speed)
    if getattr(args, "azimuth", None) is not None:
        scene.wind.azimuth_deg = float(args.azimuth)
    if getattr(args, "elevation", None) is not None:
        scene.wind.elevation_deg = float(args.elevation)

    if getattr(args, "yaw", None) is not None:
        scene.orientation.yaw_deg = float(args.yaw)
    if getattr(args, "pitch", None) is not None:
        scene.orientation.pitch_deg = float(args.pitch)
    if getattr(args, "roll", None) is not None:
        scene.orientation.roll_deg = float(args.roll)

    if getattr(args, "no_road", False):
        scene.road.enabled = False
    if getattr(args, "ground", None) is not None:
        scene.road.enabled = True
        scene.road.ride_height = float(args.ground)
    if getattr(args, "moving_road", False):
        scene.road.moving = True
    if getattr(args, "road_speed", None) is not None:
        # Naming a speed is only meaningful for a road that runs, so asking for
        # one turns the motion on rather than being silently ignored.
        scene.road.moving = True
        scene.road.speed = float(args.road_speed)
    if getattr(args, "road_tracks_wind", False):
        scene.road.moving = True
        scene.road.speed = None

    if getattr(args, "density", None) is not None:
        scene.fluid.density = float(args.density)
    if getattr(args, "viscosity", None) is not None:
        scene.fluid.viscosity = float(args.viscosity)

    if getattr(args, "speeds", None):
        low, high, count = args.speeds.split(",")
        scene.solver.speed_min = float(low)
        scene.solver.speed_max = float(high)
        scene.solver.speed_points = int(count)
    if getattr(args, "reference_speed", None) is not None:
        scene.solver.reference_speed = float(args.reference_speed)
    if getattr(args, "mode", None):
        scene.solver.sweep_mode = args.mode
    if getattr(args, "turbulence", None):
        scene.solver.turbulence = args.turbulence
    if getattr(args, "iterations", None) is not None:
        scene.solver.iterations = int(args.iterations)
    if getattr(args, "mesh_resolution", None) is not None:
        scene.solver.mesh_resolution = int(args.mesh_resolution)
    if getattr(args, "processes", None) is not None:
        scene.solver.processes = int(args.processes)
    if getattr(args, "solver", None):
        scene.solver.backends = list(args.solver)
    return scene


def _add_scene_options(parser: argparse.ArgumentParser) -> None:
    flow = parser.add_argument_group("flow")
    flow.add_argument("--wind", type=_parse_vector, help="Wind vector as x,y,z (sets speed and angles)")
    flow.add_argument("--speed", type=float, help="Wind speed in m/s")
    flow.add_argument("--azimuth", type=float, help="Wind azimuth in degrees from +X toward +Y")
    flow.add_argument("--elevation", type=float, help="Wind elevation in degrees above the ground plane")
    flow.add_argument("--density", type=float, help="Air density in kg/m^3")
    flow.add_argument("--viscosity", type=float, help="Dynamic viscosity in Pa.s")

    placement = parser.add_argument_group("placement")
    placement.add_argument("--yaw", type=float, help="Hull yaw in degrees")
    placement.add_argument("--pitch", type=float, help="Hull pitch in degrees")
    placement.add_argument("--roll", type=float, help="Hull roll in degrees")
    placement.add_argument("--ground", type=float, help="Ride height above the road in metres")
    placement.add_argument("--no-road", action="store_true", help="Remove the road entirely")
    placement.add_argument(
        "--moving-road",
        action="store_true",
        help="Road slides underneath the body, as one does under a moving vehicle",
    )
    placement.add_argument(
        "--road-speed",
        type=float,
        metavar="V",
        help="Ground speed of the vehicle in m/s (implies --moving-road). "
        "Omit it and the road tracks the wind, which is the still-air case",
    )
    placement.add_argument(
        "--road-tracks-wind",
        action="store_true",
        help="Unpin the road speed so it follows the wind again",
    )

    solving = parser.add_argument_group("solving")
    solving.add_argument("--solver", action="append", choices=["openfoam", "su2"], help="Repeatable")
    solving.add_argument("--speeds", help="Speed curve as min,max,points (e.g. 5,20,7)")
    solving.add_argument("--reference-speed", type=float, help="Speed solved when scaling the curve")
    solving.add_argument("--mode", choices=["auto", "scale", "sweep"], help="Speed curve strategy")
    solving.add_argument("--turbulence", choices=["kOmegaSST", "laminar"], help="Turbulence model")
    solving.add_argument("--iterations", type=int, help="Solver iterations")
    solving.add_argument("--mesh-resolution", type=int, help="Background mesh resolution")
    solving.add_argument(
        "--processes",
        type=int,
        metavar="N",
        help=(
            f"MPI ranks for the solve (default {default_processes()}: "
            f"80%% of the {available_cores()} cores visible here)"
        ),
    )


def command_info(args: argparse.Namespace) -> int:
    processes = getattr(args, "processes", None)
    print("Solver availability:\n")
    for info in available_solvers(processes):
        mark = "yes" if info.available else "no "
        print(f"  [{mark}] {info.label:<12} {info.detail}")
    print()
    print(f"  cores visible : {available_cores()}")
    print(f"  default ranks : {default_processes()} (80% of them; override with --processes)")
    print()
    return 0


def command_new(args: argparse.Namespace) -> int:
    scene = Scene.from_stl_file(args.stl, name=args.name)
    scene = _apply_common_options(scene, args)
    output = Path(args.output) if args.output else Path(args.stl).with_suffix(".aero.json")
    scene.save(output)

    metrics = scene.metrics()
    advice = scene.reynolds_advice(metrics.streamwise_length)
    print(f"Wrote {output}")
    print(f"  triangles     : {metrics.triangle_count}")
    print(f"  watertight    : {metrics.watertight}")
    print(f"  frontal area  : {metrics.frontal_area:.6g} m^2")
    print(f"  speed handling: {advice.mode}")
    for warning in advice.warnings:
        print(f"  ! {warning}")
    return 0


def command_run(args: argparse.Namespace) -> int:
    scene = Scene.load(args.scene)
    scene = _apply_common_options(scene, args)

    def progress(event: dict) -> None:
        message = event.get("message")
        if message:
            print(f"  {message}", flush=True)

    print(f"Computing {args.scene}")
    results = run_scene(
        scene,
        backends=scene.solver.backends,
        progress=progress,
        keep_cases=bool(args.keep_cases),
        case_root=args.keep_cases,
    )
    scene.results = results

    output = Path(args.output) if args.output else Path(args.scene)
    scene.save(output)
    print(f"\nWrote {output}\n")
    _print_results(scene)
    return 0 if any(run.status == "ok" for run in results.runs) else 1


def command_show(args: argparse.Namespace) -> int:
    scene = Scene.load(args.scene)
    print(f"Scene   : {scene.name}  ({scene.geometry.source_name})")
    print(f"Wind    : {scene.wind.speed:.4g} m/s, azimuth {scene.wind.azimuth_deg:.4g} deg, "
          f"elevation {scene.wind.elevation_deg:.4g} deg")
    print(f"Attitude: yaw {scene.orientation.yaw_deg:.4g}, pitch {scene.orientation.pitch_deg:.4g}, "
          f"roll {scene.orientation.roll_deg:.4g} deg")
    if scene.road.enabled:
        if not scene.road.moving:
            kind = "static"
        elif scene.road.speed is None:
            kind = f"moving with the wind ({scene.road.ground_speed(scene.wind_vector()):.4g} m/s)"
        else:
            kind = f"moving at {scene.road.speed:.4g} m/s"
        print(f"Road    : {kind}, ride height {scene.road.ride_height:.4g} m")
    else:
        print("Road    : disabled")
    print(f"Fluid   : rho {scene.fluid.density:.4g} kg/m^3, mu {scene.fluid.viscosity:.4g} Pa.s")
    print()
    if not scene.computed:
        print("Not computed yet. Run:  python runner.py run <scene>")
        return 0
    _print_results(scene)
    return 0


def _print_results(scene: Scene) -> None:
    results = scene.results
    if results is None:
        print("No results.")
        return

    geometry = results.geometry or {}
    print(f"Computed {results.computed_at} on {results.host}")
    if geometry:
        print(f"Frontal area : {geometry.get('frontal_area', float('nan')):.6g} m^2")
        print(f"Ref length   : {geometry.get('streamwise_length', float('nan')):.6g} m")
    print()

    for run in results.runs:
        header = f"{run.solver} [{run.status}, {run.mode}, {run.wall_time_s:.1f}s]"
        print(header)
        print("-" * len(header))
        if run.status != "ok":
            print(f"  {run.message}")
            if run.log_excerpt:
                print("  " + "\n  ".join(run.log_excerpt.splitlines()[-8:]))
            print()
            continue
        reference = run.reference_point()
        if reference is not None:
            print(f"  Cd = {reference.drag_coefficient:.5g}   (A = {reference.frontal_area:.5g} m^2)")
        print(f"  {'speed m/s':>10}  {'Re':>10}  {'drag N':>10}  source")
        for point in run.points:
            print(
                f"  {point.speed:>10.4g}  {point.reynolds:>10.3g}  "
                f"{point.drag_force:>10.5g}  {point.source}"
            )
        if run.message:
            print(f"  ! {run.message}")
        print()

    for warning in results.warnings:
        print(f"! {warning}")


def command_export(args: argparse.Namespace) -> int:
    """Write ready-to-run solver cases without solving anything."""
    scene = Scene.load(args.scene)
    scene = _apply_common_options(scene, args)
    root = Path(args.dir)
    root.mkdir(parents=True, exist_ok=True)

    mesh = scene.placed_mesh()
    metrics = scene.metrics()
    speed = scene.solver.reference_speed
    wind = scene.wind_vector(speed)
    written: list[str] = []

    backends = scene.solver.backends if not args.solver else args.solver

    if "openfoam" in backends:
        from openfoam import prepare_openfoam_case

        case = root / "openfoam"
        prepare_openfoam_case(
            mesh,
            wind,
            scene.fluid.density,
            scene.fluid.viscosity,
            scene.ground_offset(),
            case,
            turbulent=scene.solver.turbulence != "laminar",
            iterations=scene.solver.iterations,
            mesh_resolution=scene.solver.mesh_resolution,
            refinement_level=scene.solver.refinement_level,
            road_velocity=scene.road_velocity(speed),
            reference_area=metrics.frontal_area,
        )
        (case / "Allrun").write_text(
            "#!/bin/sh\n"
            "blockMesh > log.blockMesh 2>&1\n"
            "snappyHexMesh -overwrite > log.snappyHexMesh 2>&1\n"
            "foamRun -solver incompressibleFluid > log.foamRun 2>&1\n",
            encoding="utf-8",
        )
        written.append(str(case))

    if "su2" in backends:
        from su2 import prepare_su2_case

        case = root / "su2"
        prepare_su2_case(
            mesh,
            wind,
            case,
            density=scene.fluid.density,
            viscosity=scene.fluid.viscosity,
            ground_offset=scene.ground_offset(),
            turbulent=scene.solver.turbulence != "laminar",
            iterations=scene.solver.iterations,
            surface_cells=max(scene.solver.mesh_resolution // 2, 8),
            refinement_level=scene.solver.refinement_level,
            reference_area=metrics.frontal_area,
            road_velocity=scene.road_velocity(speed),
        )
        (case / "Allrun").write_text("#!/bin/sh\nSU2_CFD case.cfg > log.su2 2>&1\n", encoding="utf-8")
        written.append(str(case))

    for path in written:
        print(f"Wrote {path}")
    print(f"\nReference area {metrics.frontal_area:.6g} m^2 at {speed:.4g} m/s.")
    return 0


def command_fair(args: argparse.Namespace) -> int:
    """Payload STL in, one single-body shell out, as a scene ready to solve."""
    import fairing as fairing_module
    from scene import FairingSpec, Geometry

    payload_path = Path(args.payload)
    scene = Scene.from_stl_file(payload_path, name=payload_path.stem)
    scene = _apply_common_options(scene, args)
    scene.payload = scene.geometry

    payload_mesh = scene.payload.raw_mesh()
    direction = scene.wind.direction()
    streamline = None if args.no_streamline else (args.nose_angle, args.tail_angle)
    blend = args.shoulder_blend if args.profile == "blended" else 0.0
    scene.packaging.envelope_profile = args.profile
    scene.packaging.shoulder_blend = args.shoulder_blend

    print(f"Voxelising {payload_path.name}")
    grid = fairing_module.build_grid(
        payload_mesh, direction=direction,
        resolution=max(args.resolution // 2, 32), anisotropy=args.anisotropy,
    )
    print(f"  grid {grid.occupancy.shape}, pitch {grid.pitch * 1000:.1f} mm")

    result = fairing_module.sweep(
        grid, anisotropy=args.anisotropy, clearance=args.clearance,
        progress=lambda event: print(f"  {event['message']}"),
    )
    if result.merge_radius is None:
        print(
            "\nThe payload never closes into a single body within the grid. Raise "
            "--anisotropy or --clearance, which both bridge gaps sooner."
        )
        for warning in grid.warnings:
            print(f"! {warning}")
        return 1

    print(
        f"\n{result.bodies_at_zero} separate bodies uncovered, merging into one at "
        f"r = {result.merge_radius * 1000:.0f} mm."
    )

    # The skin grid gets the streamwise room the nose and tail cones need;
    # the sweep grid above does not, since merging needs no tail.
    fine = fairing_module.build_grid(
        payload_mesh, direction=direction, resolution=args.resolution,
        anisotropy=args.anisotropy, streamline=streamline, clearance=args.clearance,
        shoulder_blend=blend,
    )
    shell = fairing_module.build_single_shell(
        grid, payload_mesh, result, direction=direction,
        clearance=args.clearance,
        progress=lambda event: print(f"  {event['message']}"),
        build_grid_override=fine, streamline=streamline, shoulder_blend=blend,
    )

    scene.geometry = Geometry.from_bytes(
        shell.mesh.export(file_type="stl"), source_name=f"{payload_path.stem}_shell.stl"
    )
    scene.fairing = FairingSpec(
        closing_radius=shell.radius,
        clearance=shell.clearance,
        anisotropy=shell.anisotropy,
        components=1,
        resolution=args.resolution,
        streamlined=shell.streamlined,
        nose_angle_deg=shell.nose_angle_deg,
        tail_angle_deg=shell.tail_angle_deg,
        envelope_profile=args.profile,
        shoulder_blend=shell.shoulder_blend,
    )
    scene.name = f"{payload_path.stem}_shell"

    output = Path(args.output) if args.output else Path(f"{scene.name}.aero.json")
    scene.save(output)

    shape = (
        f"streamlined, tail {shell.tail_angle_deg:.0f} deg, "
        + (
            f"shoulders blended over {shell.shoulder_blend:.2f} half-widths"
            if shell.shoulder_blend > 0
            else "faceted shoulders"
        )
        if shell.streamlined
        else "raw closing skin"
    )
    print(f"\nWrote {output}")
    print(f"  closing radius : {shell.radius * 1000:.0f} mm ({shape})")
    print(f"  frontal area   : {shell.frontal_area:.4f} m^2")
    print(f"  volume         : {'-' if shell.volume is None else f'{shell.volume:.4f} m^3'}")
    print(f"  bodies         : {shell.bodies}")
    print(f"  watertight     : {shell.watertight}")
    print(
        "  payload fit    : "
        + {True: "encloses", False: "STICKS OUT", None: "unverified"}[shell.contains_payload]
    )

    if shell.bodies > 1:
        print("\n! The shell is still in pieces. Raise --anisotropy or --clearance.")
    for warning in list(grid.warnings) + list(fine.warnings):
        print(f"\n! {warning}")

    print(f"\nCompute it with:  python src/runner.py run {output}")
    return 0


def command_compare(args: argparse.Namespace) -> int:
    """Compute several scenes and rank them by drag area."""
    rows = []
    for path in args.scenes:
        scene = Scene.load(path)
        scene = _apply_common_options(scene, args)
        if args.quality:
            scene.solver.apply_preset(args.quality)
        print(f"\n=== {Path(path).name} ===")
        results = run_scene(
            scene, backends=scene.solver.backends,
            progress=lambda event: print(f"  {event['message']}") if event.get("message") else None,
        )
        scene.results = results
        scene.save(path)

        for run in results.runs:
            if run.status != "ok":
                continue
            point = run.reference_point()
            if point is None:
                continue
            rows.append(
                {
                    "scene": Path(path).name,
                    "solver": run.solver,
                    "cd": point.drag_coefficient,
                    "area": point.frontal_area,
                    "drag_area": point.drag_coefficient * point.frontal_area,
                }
            )
            break

    if not rows:
        print("\nNothing to rank: no CFD backend produced a result.")
        return 1

    rows.sort(key=lambda item: item["drag_area"])
    print("\nRanking by drag area (Cd x A, lower is better):\n")
    print(f"  {'scene':<34} {'solver':<9} {'Cd':>8} {'A m^2':>9} {'Cd.A m^2':>10}")
    for row in rows:
        print(
            f"  {row['scene']:<34} {row['solver']:<9} {row['cd']:>8.4f} "
            f"{row['area']:>9.4f} {row['drag_area']:>10.5f}"
        )
    print(f"\nBest: {rows[0]['scene']}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="runner.py",
        description="Build, compute and inspect aero drag scenes.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    info = subparsers.add_parser("info", help="Show which solvers are usable here")
    info.add_argument("--processes", type=int, metavar="N", help="Preview a specific rank count")
    info.set_defaults(handler=command_info)

    new = subparsers.add_parser("new", help="Create a scene file from an STL")
    new.add_argument("--stl", required=True, help="Input STL")
    new.add_argument("-o", "--output", help="Output scene file")
    new.add_argument("--name", help="Scene name")
    _add_scene_options(new)
    new.set_defaults(handler=command_new)

    run = subparsers.add_parser("run", help="Compute a scene and store the results in it")
    run.add_argument("scene", help="Scene file")
    run.add_argument("-o", "--output", help="Write to this file instead of in place")
    run.add_argument("--keep-cases", help="Keep the generated solver cases in this directory")
    _add_scene_options(run)
    run.set_defaults(handler=command_run)

    show = subparsers.add_parser("show", help="Print a scene and its results")
    show.add_argument("scene")
    show.set_defaults(handler=command_show)

    fair = subparsers.add_parser(
        "fair", help="Wrap a payload STL in a single-body fairing"
    )
    fair.add_argument("--payload", required=True, help="Payload STL: what has to fit inside")
    fair.add_argument("-o", "--output", help="Scene file to write (default <payload>_shell.aero.json)")
    fair.add_argument("--clearance", type=float, default=0.03, help="Gap between payload and skin (m)")
    fair.add_argument("--anisotropy", type=float, default=3.0, help="Streamwise bias of the closing")
    fair.add_argument("--resolution", type=int, default=128, help="Voxel resolution for the skin")
    fair.add_argument(
        "--nose-angle", type=float, default=45.0,
        help="Nose growth limit in degrees from the flow axis (blunter is shorter)",
    )
    fair.add_argument(
        "--tail-angle", type=float, default=12.0,
        help="Tail taper limit in degrees; much past 15 the afterbody separates",
    )
    fair.add_argument(
        "--no-streamline", action="store_true",
        help="Skip the taper-bounded envelope and emit the raw closing skin",
    )
    fair.add_argument(
        "--profile", choices=["faceted", "blended"], default="faceted",
        help="Shoulder treatment where the tapers meet the payload's own section: "
        "'faceted' is the minimal envelope, with a crease at each shoulder; "
        "'blended' rounds them, at the cost of wetted area and length "
        "(frontal area is identical either way)",
    )
    fair.add_argument(
        "--shoulder-blend", type=float, default=0.5,
        help="How far a blended shoulder spreads along the flow, as a fraction of "
        "the payload's cross-flow half-width (default 0.5, ignored when faceted)",
    )
    _add_scene_options(fair)
    fair.set_defaults(handler=command_fair)

    compare = subparsers.add_parser("compare", help="Compute several scenes and rank them")
    compare.add_argument("scenes", nargs="+", help="Scene files to compare")
    compare.add_argument("--quality", choices=["screening", "balanced", "accurate"], help="Preset")
    _add_scene_options(compare)
    compare.set_defaults(handler=command_compare)

    export = subparsers.add_parser("export", help="Write ready-to-run solver cases")
    export.add_argument("scene")
    export.add_argument("--dir", required=True, help="Destination directory")
    _add_scene_options(export)
    export.set_defaults(handler=command_export)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
