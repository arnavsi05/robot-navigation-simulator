"""Generates results/static_navigation.png: A* paths on the simple and
complex static scenarios, side by side.

Run with:
    python -m scripts.run_static_demo
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

from src.environment import GridEnvironment
from src.planners.astar import astar
from src.utils.visualization import draw_environment, legend_once

SCENARIOS = ROOT / "scenarios"
RESULTS_DIR = ROOT / "results"


def main():
    RESULTS_DIR.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, name in zip(axes, ["simple.json", "complex.json"]):
        env = GridEnvironment.from_scenario_file(SCENARIOS / name)
        result = astar(env, env.start, env.goal)
        assert result.success, f"{name} should be solvable"
        title = (
            f"{name.replace('.json', '').title()} scenario\n"
            f"A* path length={result.path_length:.1f}, "
            f"nodes expanded={result.nodes_expanded}, "
            f"time={result.planning_time * 1000:.2f} ms"
        )
        draw_environment(ax, env, path=result.path, title=title)

    legend_once(fig, axes[0])
    fig.suptitle("Static Navigation: A* Path Planning", fontsize=14)
    fig.tight_layout(rect=[0, 0.06, 1, 0.95])

    out_path = RESULTS_DIR / "static_navigation.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
