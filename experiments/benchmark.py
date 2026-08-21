"""Benchmarking harness: runs A* and Dijkstra across many randomly generated
environments (two difficulty tiers) plus repeated dynamic-obstacle
simulations, and writes results/results.csv plus results/algorithm_comparison.png.

Run with:
    python -m experiments.benchmark
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.environment import DynamicObstacle, GridEnvironment
from src.planners.astar import astar
from src.planners.dijkstra import dijkstra
from src.simulation import Simulation
from src.utils.metrics import TrialResult, save_results_csv, summarize

RESULTS_DIR = ROOT / "results"

STATIC_TIERS = {
    "simple": dict(width=25, height=15, density=0.15, seeds=range(0, 15)),
    "complex": dict(width=40, height=25, density=0.28, seeds=range(100, 115)),
}
DYNAMIC_TRIALS = dict(width=25, height=15, density=0.12, seeds=range(200, 215))
START = (1, 1)


def generate_solvable_environment(width, height, density, seed, start, goal, max_attempts=20):
    """Generate a random obstacle field and verify (via Dijkstra) that a path
    exists from start to goal, retrying with a derived seed if not."""
    for attempt in range(max_attempts):
        env = GridEnvironment(width=width, height=height, seed=seed * 1000 + attempt)
        env.set_start(start)
        env.set_goal(goal)
        env.add_random_obstacles(density)
        if dijkstra(env, start, goal).success:
            return env
    raise RuntimeError(f"Could not generate a solvable environment for seed={seed} after {max_attempts} attempts")


def run_static_benchmark() -> list[TrialResult]:
    results: list[TrialResult] = []
    for tier_name, cfg in STATIC_TIERS.items():
        goal = (cfg["height"] - 2, cfg["width"] - 2)
        for trial, seed in enumerate(cfg["seeds"]):
            env = generate_solvable_environment(
                cfg["width"], cfg["height"], cfg["density"], seed, START, goal
            )
            for planner_fn, name in [(astar, "astar"), (dijkstra, "dijkstra")]:
                r = planner_fn(env, START, goal)
                results.append(
                    TrialResult(
                        scenario=tier_name,
                        algorithm=name,
                        trial=trial,
                        success=r.success,
                        planning_time_sec=r.planning_time,
                        path_length=r.path_length,
                        nodes_expanded=r.nodes_expanded,
                    )
                )
    return results


def run_dynamic_benchmark() -> list[TrialResult]:
    cfg = DYNAMIC_TRIALS
    results: list[TrialResult] = []
    goal = (cfg["height"] - 2, cfg["width"] - 2)
    for trial, seed in enumerate(cfg["seeds"]):
        env = generate_solvable_environment(cfg["width"], cfg["height"], cfg["density"], seed, START, goal)
        # Two patrolling obstacles sweeping across the environment.
        env.add_dynamic_obstacle(
            DynamicObstacle(waypoints=[(1, cfg["width"] // 3), (cfg["height"] - 2, cfg["width"] // 3)])
        )
        env.add_dynamic_obstacle(
            DynamicObstacle(
                waypoints=[(cfg["height"] - 2, 2 * cfg["width"] // 3), (1, 2 * cfg["width"] // 3)]
            )
        )
        sim = Simulation(env, astar, max_steps=400, max_replans=100)
        result = sim.run()
        results.append(
            TrialResult(
                scenario="dynamic",
                algorithm="astar",
                trial=trial,
                success=result.success,
                planning_time_sec=result.total_planning_time_sec,
                path_length=result.final_path_length,
                nodes_expanded=0,
                simulation_steps=result.steps_taken,
                replan_count=result.replan_count,
                total_planning_time_sec=result.total_planning_time_sec,
            )
        )
    return results


def plot_algorithm_comparison(df, out_path: Path) -> None:
    static_df = df[df["scenario"].isin(["simple", "complex"])]
    summary = summarize(static_df)

    scenarios = ["simple", "complex"]
    algorithms = ["astar", "dijkstra"]
    x = range(len(scenarios))
    width = 0.35

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    for i, algo in enumerate(algorithms):
        times = [
            summary[(summary.scenario == s) & (summary.algorithm == algo)]["mean_planning_time_sec"].iloc[0]
            for s in scenarios
        ]
        offset = (i - 0.5) * width
        axes[0].bar([xi + offset for xi in x], times, width, label=algo)
    axes[0].set_xticks(list(x))
    axes[0].set_xticklabels(scenarios)
    axes[0].set_ylabel("Mean planning time (s)")
    axes[0].set_title("Planning Time by Algorithm")
    axes[0].legend()

    for i, algo in enumerate(algorithms):
        nodes = [
            summary[(summary.scenario == s) & (summary.algorithm == algo)]["mean_nodes_expanded"].iloc[0]
            for s in scenarios
        ]
        offset = (i - 0.5) * width
        axes[1].bar([xi + offset for xi in x], nodes, width, label=algo)
    axes[1].set_xticks(list(x))
    axes[1].set_xticklabels(scenarios)
    axes[1].set_ylabel("Mean nodes expanded")
    axes[1].set_title("Search Effort by Algorithm")
    axes[1].legend()

    fig.suptitle("A* vs Dijkstra: Planning Time and Search Effort")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    RESULTS_DIR.mkdir(exist_ok=True)
    print("Running static benchmark (A* vs Dijkstra)...")
    static_results = run_static_benchmark()
    print("Running dynamic obstacle / replanning benchmark...")
    dynamic_results = run_dynamic_benchmark()

    all_results = static_results + dynamic_results
    df = save_results_csv(all_results, RESULTS_DIR / "results.csv")
    print(f"Saved {len(df)} trial rows to {RESULTS_DIR / 'results.csv'}")

    summary = summarize(df)
    print("\n=== Summary ===")
    print(summary.to_string(index=False))

    plot_algorithm_comparison(df, RESULTS_DIR / "algorithm_comparison.png")
    print(f"\nSaved comparison plot to {RESULTS_DIR / 'algorithm_comparison.png'}")


if __name__ == "__main__":
    main()
