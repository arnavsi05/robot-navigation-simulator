"""Matplotlib rendering helpers shared by the demo scripts.

Kept separate from the core simulation/planning code so that the algorithmic
modules have no plotting dependency.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

Cell = Tuple[int, int]


def draw_environment(
    ax,
    env,
    path: Optional[List[Cell]] = None,
    original_path: Optional[List[Cell]] = None,
    robot_pos: Optional[Cell] = None,
    dynamic_positions: Optional[List[Cell]] = None,
    title: Optional[str] = None,
    traveled: Optional[List[Cell]] = None,
):
    """Render one occupancy grid frame: obstacles, start/goal, path(s),
    robot position, and dynamic obstacle positions.

    Cells are (row, col); plotted with col on the x-axis and row on the
    y-axis (inverted so row 0 is at the top, matching the grid array).
    """
    ax.clear()
    ax.imshow(env.grid, cmap="Greys", origin="upper", vmin=0, vmax=1, interpolation="nearest")

    if original_path:
        rows, cols = zip(*original_path)
        ax.plot(cols, rows, color="#a0a0ff", linewidth=2, linestyle="--", label="Original plan", zorder=2)

    if traveled and len(traveled) > 1:
        rows, cols = zip(*traveled)
        ax.plot(cols, rows, color="#2ca02c", linewidth=2.5, alpha=0.6, label="Traveled", zorder=3)

    if path:
        rows, cols = zip(*path)
        ax.plot(cols, rows, color="#1f77b4", linewidth=2.5, label="Current plan", zorder=4)

    if env.start is not None:
        ax.scatter(*env.start[::-1], c="#2ca02c", s=90, marker="o", zorder=5, label="Start", edgecolors="black")
    if env.goal is not None:
        ax.scatter(*env.goal[::-1], c="gold", s=150, marker="*", zorder=5, label="Goal", edgecolors="black")

    if dynamic_positions:
        for pos in dynamic_positions:
            ax.scatter(pos[1], pos[0], c="#d62728", s=110, marker="s", zorder=6, edgecolors="black")

    if robot_pos:
        ax.scatter(robot_pos[1], robot_pos[0], c="#ff7f0e", s=130, marker="o", zorder=7, edgecolors="black", label="Robot")

    ax.set_xlim(-0.5, env.width - 0.5)
    ax.set_ylim(env.height - 0.5, -0.5)
    ax.set_xticks([])
    ax.set_yticks([])
    if title:
        ax.set_title(title, fontsize=11)
    ax.set_aspect("equal")


def legend_once(fig, ax):
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    fig.legend(by_label.values(), by_label.keys(), loc="lower center", ncol=len(by_label), fontsize=9)
