"""Runs the dynamic-obstacle scenario, records every simulation step, and
compiles the frames into results/dynamic_replanning.gif.

Run with:
    python -m scripts.run_dynamic_demo
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.environment import GridEnvironment
from src.planners.astar import astar
from src.simulation import Simulation
from src.utils.visualization import draw_environment, legend_once

SCENARIOS = ROOT / "scenarios"
RESULTS_DIR = ROOT / "results"
FRAMES_DIR = RESULTS_DIR / "frames"


def main():
    RESULTS_DIR.mkdir(exist_ok=True)
    FRAMES_DIR.mkdir(exist_ok=True)

    env = GridEnvironment.from_scenario_file(SCENARIOS / "dynamic.json")
    sim = Simulation(env, astar, max_steps=300, max_replans=50)
    result = sim.run(record_frames=True)

    print(f"Success: {result.success}")
    print(f"Steps taken: {result.steps_taken}")
    print(f"Replan events: {result.replan_count}")
    print(f"Total planning time: {result.total_planning_time_sec * 1000:.2f} ms")
    assert result.success, f"dynamic scenario simulation failed: {result.failure_reason}"

    original_path = result.initial_path
    frame_paths = []
    traveled = []

    fig, ax = plt.subplots(figsize=(8, 6))
    for frame in result.frames:
        traveled.append(frame.robot_position)
        status = "REPLANNED" if frame.replanned else ""
        title = f"Dynamic Obstacle Replanning — step {frame.step} {status}".strip()
        draw_environment(
            ax,
            env,
            path=frame.path,
            original_path=original_path,
            robot_pos=frame.robot_position,
            dynamic_positions=frame.dynamic_positions,
            traveled=traveled,
            title=title,
        )
        if frame.step == 0:
            legend_once(fig, ax)
            fig.tight_layout(rect=[0, 0.08, 1, 1])
        frame_path = FRAMES_DIR / f"frame_{frame.step:04d}.png"
        fig.savefig(frame_path, dpi=110)
        frame_paths.append(frame_path)
    plt.close(fig)

    images = [imageio.imread(p) for p in frame_paths]
    # Hold the final "reached goal" frame a little longer.
    images += [images[-1]] * 6
    out_path = RESULTS_DIR / "dynamic_replanning.gif"
    imageio.mimsave(out_path, images, duration=0.15, loop=0)
    print(f"Saved {out_path} ({len(images)} frames)")

    shutil.rmtree(FRAMES_DIR)


if __name__ == "__main__":
    main()
