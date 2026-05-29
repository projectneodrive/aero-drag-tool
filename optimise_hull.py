from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
import trimesh

from aero import estimate_drag, finite_difference_gradient
from geometry import (
    ConvexSupportModel,
    center_mesh,
    convex_hull,
    fit_support_model,
    load_stl,
    sample_convex_hull_support,
    save_surface_function,
    surface_mesh_from_model,
)


def parse_vector(text: str) -> np.ndarray:
    parts = [float(value.strip()) for value in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("wind must be formatted as x,y,z")
    return np.array(parts, dtype=float)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Optimise a convex smooth hull from an STL file.")
    parser.add_argument("--input", required=True, help="Input STL file")
    parser.add_argument("--output_dir", default="results", help="Output directory")
    parser.add_argument("--wind", type=parse_vector, default=parse_vector("10,0,0"), help="Wind vector as x,y,z")
    parser.add_argument("--ground", type=float, default=None, help="Ground plane offset above z=0")
    parser.add_argument("--density", type=float, default=1.225, help="Air density in kg/m^3")
    parser.add_argument("--viscosity", type=float, default=1.8e-5, help="Dynamic viscosity in Pa*s")
    parser.add_argument("--solver", choices=["proxy", "openfoam", "auto"], default="proxy", help="Drag backend")
    parser.add_argument("--max_iter", type=int, default=30, help="Optimisation iterations")
    parser.add_argument("--n_directions", type=int, default=256, help="Support sample directions")
    parser.add_argument("--n_centers", type=int, default=96, help="RBF centers on the unit sphere")
    parser.add_argument("--subdivisions", type=int, default=3, help="Icosphere subdivisions for mesh export")
    parser.add_argument("--visualize", action="store_true", help="Save a comparison plot")
    return parser


def model_to_mesh(model: ConvexSupportModel, centroid: np.ndarray, ground_offset: float | None, subdivisions: int) -> trimesh.Trimesh:
    mesh = surface_mesh_from_model(model, subdivisions=subdivisions)
    mesh.apply_translation(centroid)
    if ground_offset is not None:
        z_shift = ground_offset - float(np.min(mesh.vertices[:, 2]))
        mesh.apply_translation([0.0, 0.0, z_shift])
    return mesh


def objective_from_model(
    parameters: np.ndarray,
    base_mesh: trimesh.Trimesh,
    centroid: np.ndarray,
    centers: np.ndarray,
    wind: np.ndarray,
    density: float,
    ground_offset: float | None,
    subdivisions: int,
    penalty_scale: float,
) -> float:
    model = ConvexSupportModel(bias=float(parameters[0]), weights=np.maximum(parameters[1:], 0.0), centers=centers)
    mesh = model_to_mesh(model, centroid, ground_offset, subdivisions)
    drag = estimate_drag(mesh, wind, density=density)

    hull = convex_hull(base_mesh)
    centered_hull, _ = center_mesh(hull)
    hull_vertices = centered_hull.vertices
    directions = hull_vertices / np.clip(np.linalg.norm(hull_vertices, axis=1, keepdims=True), 1e-12, None)
    distances = np.linalg.norm(directions[:, None, :] - centers[None, :, :], axis=-1)
    basis = np.column_stack([np.ones(len(directions)), np.clip(1.0 - 0.5 * distances, 0.0, None) ** 6])
    support = basis @ np.concatenate([[model.bias], model.weights])
    target = np.sum(hull_vertices * directions, axis=1)
    violation = np.clip(target - support, 0.0, None)
    return float(drag + penalty_scale * np.sum(violation * violation))


def visualise_result(original: trimesh.Trimesh, final_mesh: trimesh.Trimesh, output_path: Path, ground_offset: float | None, wind: np.ndarray) -> None:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    orig_vertices = np.asarray(original.vertices)
    final_vertices = np.asarray(final_mesh.vertices)
    orig_faces = np.asarray(original.faces)
    final_faces = np.asarray(final_mesh.faces)

    ax.plot_trisurf(orig_vertices[:, 0], orig_vertices[:, 1], orig_vertices[:, 2], triangles=orig_faces, color="#5b8def", alpha=0.18, linewidth=0.2)
    ax.plot_trisurf(final_vertices[:, 0], final_vertices[:, 1], final_vertices[:, 2], triangles=final_faces, color="#f59e0b", alpha=0.7, linewidth=0.15)

    if ground_offset is not None:
        bounds = final_mesh.bounds
        x = np.linspace(bounds[0, 0], bounds[1, 0], 2)
        y = np.linspace(bounds[0, 1], bounds[1, 1], 2)
        xx, yy = np.meshgrid(x, y)
        zz = np.full_like(xx, ground_offset)
        ax.plot_surface(xx, yy, zz, color="#64748b", alpha=0.22, linewidth=0)

    origin = final_mesh.centroid
    wind_vec = wind / np.linalg.norm(wind) * (np.linalg.norm(final_mesh.extents) * 0.5)
    ax.quiver(origin[0], origin[1], origin[2], wind_vec[0], wind_vec[1], wind_vec[2], color="#111827", linewidth=2.0, arrow_length_ratio=0.1)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=18, azim=35)
    plt.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    original_mesh = load_stl(input_path)
    hull = convex_hull(original_mesh)
    centered_hull, centroid = center_mesh(hull)

    raw_support, centers, source_centroid = sample_convex_hull_support(original_mesh, args.n_directions, args.n_centers)
    if np.linalg.norm(source_centroid - centroid) > 1e-8:
        centroid = source_centroid

    np.save(output_dir / "raw_support.npy", raw_support)
    np.save(output_dir / "centers.npy", centers)
    hull.export(output_dir / "initial_hull.stl")

    sample_dirs = raw_support[:, :3]
    sample_values = raw_support[:, 3]
    model = fit_support_model(sample_dirs, sample_values, centers, enclosure_vertices=centered_hull.vertices)

    initial_mesh = model_to_mesh(model, centroid, args.ground, args.subdivisions)
    np.save(output_dir / "weights.npy", model.weights)
    save_surface_function(output_dir / "surface_function.py", model)
    initial_mesh.export(output_dir / "initial_smooth_hull.stl")

    parameter_vector = np.concatenate([[model.bias], model.weights])
    history: list[dict[str, float]] = []

    def objective(parameters: np.ndarray) -> float:
        return objective_from_model(
            parameters,
            original_mesh,
            centroid,
            centers,
            args.wind,
            args.density,
            args.ground,
            args.subdivisions,
            penalty_scale=10_000.0,
        )

    def gradient(parameters: np.ndarray) -> np.ndarray:
        return finite_difference_gradient(objective, parameters, epsilon=1e-3)

    bounds = [(0.0, None)] * len(parameter_vector)

    def callback(parameters: np.ndarray) -> None:
        value = objective(parameters)
        history.append({"iteration": float(len(history)), "drag_proxy": float(value)})

    result = minimize(
        objective,
        parameter_vector,
        method="L-BFGS-B",
        jac=gradient,
        bounds=bounds,
        options={"maxiter": args.max_iter, "ftol": 1e-9},
        callback=callback,
    )

    final_parameters = np.maximum(result.x, 0.0)
    final_model = ConvexSupportModel(bias=float(final_parameters[0]), weights=final_parameters[1:], centers=centers)
    final_mesh = model_to_mesh(final_model, centroid, args.ground, args.subdivisions)
    final_mesh.export(output_dir / "final_hull.stl")
    save_surface_function(output_dir / "surface_function.py", final_model)

    final_drag = estimate_drag(final_mesh, args.wind, density=args.density, viscosity=args.viscosity, solver=args.solver, ground_offset=args.ground)
    final_penalized = objective(final_parameters)

    with (output_dir / "results.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["metric", "value"])
        writer.writeheader()
        writer.writerow({"metric": "initial_proxy", "value": objective(parameter_vector)})
        writer.writerow({"metric": "final_proxy", "value": final_penalized})
        writer.writerow({"metric": "final_drag_n", "value": final_drag})
        writer.writerow({"metric": "success", "value": float(result.success)})

    if history:
        with (output_dir / "history.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["iteration", "drag_proxy"])
            writer.writeheader()
            writer.writerows(history)

        fig = plt.figure(figsize=(8, 4))
        ax = fig.add_subplot(111)
        ax.plot([row["iteration"] for row in history], [row["drag_proxy"] for row in history], color="#0f766e", linewidth=2.0)
        ax.set_xlabel("Iteration")
        ax.set_ylabel("Proxy objective")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(output_dir / "optimisation_history.png", dpi=220)
        plt.close(fig)

    if args.visualize:
        visualise_result(hull, final_mesh, output_dir / "comparison.png", args.ground, args.wind)

    print("Initial proxy:", objective(parameter_vector))
    print("Final proxy:", final_penalized)
    print("Final drag estimate (N):", final_drag)
    if not result.success:
        print("Optimiser status:", result.message)
    print("Output directory:", output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())