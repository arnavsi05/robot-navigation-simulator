"""2D occupancy-grid environment for the robot navigation simulator.

The environment is a rectangular grid where each cell is either free (0)
or occupied by an obstacle (1). Positions are represented as ``(row, col)``
tuples, with ``row`` increasing downward (matching numpy array indexing and
``matplotlib``'s default ``imshow`` orientation).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]

FREE = 0
OBSTACLE = 1


@dataclass
class DynamicObstacle:
    """An obstacle that moves through the environment over time.

    The obstacle follows a "patrol" policy: it walks back and forth along a
    list of waypoints, moving one cell per simulation step. This is a simple
    but effective way to create a moving hazard without a physics engine.

    Internally the full back-and-forth patrol is precomputed once as a
    cyclic list of cells, so stepping is just a modulo index increment --
    no special-casing is needed at the turnaround points.
    """

    waypoints: List[Cell]
    speed_cells_per_step: int = 1
    _cycle: List[Cell] = field(default_factory=list, repr=False)
    _index: int = field(default=0, repr=False)

    def __post_init__(self) -> None:
        if len(self.waypoints) < 2:
            raise ValueError("DynamicObstacle needs at least 2 waypoints")
        forward_cells: List[Cell] = []
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            leg = _bresenham_line(a, b)
            if forward_cells:
                leg = leg[1:]  # drop the point shared with the previous leg
            forward_cells.extend(leg)
        # Bounce back along the same cells, excluding both endpoints (each
        # is visited once per full cycle, at the start of a forward/backward leg).
        backward_cells = forward_cells[-2:0:-1]
        self._cycle = forward_cells + backward_cells
        self._index = 0
        self.position: Cell = self._cycle[0]

    def step(self) -> Cell:
        """Advance the obstacle by one simulation tick and return its new cell."""
        for _ in range(self.speed_cells_per_step):
            self._index = (self._index + 1) % len(self._cycle)
            self.position = self._cycle[self._index]
        return self.position

    def reset(self) -> None:
        self._index = 0
        self.position = self._cycle[0]


def _bresenham_line(a: Cell, b: Cell) -> List[Cell]:
    """Return grid cells on the straight line from a to b (inclusive), Bresenham's algorithm."""
    r0, c0 = a
    r1, c1 = b
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


class GridEnvironment:
    """A 2D occupancy-grid world with a start, a goal, and static obstacles."""

    def __init__(self, width: int, height: int, seed: Optional[int] = None):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=np.int8)
        self.start: Optional[Cell] = None
        self.goal: Optional[Cell] = None
        self.dynamic_obstacles: List[DynamicObstacle] = []
        self.rng = np.random.default_rng(seed)
        self.seed = seed

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    def set_start(self, cell: Cell) -> None:
        self.start = tuple(cell)

    def set_goal(self, cell: Cell) -> None:
        self.goal = tuple(cell)

    def add_obstacle(self, cell: Cell) -> None:
        r, c = cell
        self.grid[r, c] = OBSTACLE

    def add_obstacle_rect(self, top_left: Cell, bottom_right: Cell) -> None:
        """Fill a rectangular block of obstacle cells (inclusive bounds)."""
        r0, c0 = top_left
        r1, c1 = bottom_right
        self.grid[r0 : r1 + 1, c0 : c1 + 1] = OBSTACLE

    def add_random_obstacles(self, density: float) -> None:
        """Randomly fill cells as obstacles, using this environment's seeded RNG.

        The start and goal cells (if already set) are protected from being
        overwritten.
        """
        mask = self.rng.random((self.height, self.width)) < density
        self.grid[mask] = OBSTACLE
        if self.start is not None:
            self.grid[self.start] = FREE
        if self.goal is not None:
            self.grid[self.goal] = FREE

    def add_dynamic_obstacle(self, obstacle: DynamicObstacle) -> None:
        self.dynamic_obstacles.append(obstacle)

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_static_free(self, cell: Cell) -> bool:
        r, c = cell
        return self.grid[r, c] == FREE

    def is_free(self, cell: Cell, include_dynamic: bool = True) -> bool:
        """A cell is free if it is in bounds, has no static obstacle, and
        (optionally) is not currently occupied by a dynamic obstacle."""
        if not self.in_bounds(cell):
            return False
        if not self.is_static_free(cell):
            return False
        if include_dynamic and cell in self.dynamic_positions():
            return False
        return True

    def dynamic_positions(self) -> List[Cell]:
        return [obs.position for obs in self.dynamic_obstacles]

    def step_dynamic_obstacles(self) -> List[Cell]:
        """Advance all dynamic obstacles by one tick and return their new positions."""
        return [obs.step() for obs in self.dynamic_obstacles]

    def is_path_collision_free(self, path: List[Cell], include_dynamic: bool = False) -> bool:
        """Check that every cell of a path is free of static (and optionally
        dynamic) obstacles. Used to validate planner output and to detect
        when a previously-valid path has been invalidated by a moving obstacle.
        """
        for cell in path:
            if not self.is_static_free(cell) or not self.in_bounds(cell):
                return False
            if include_dynamic and cell in self.dynamic_positions():
                return False
        return True

    def reset_dynamic_obstacles(self) -> None:
        for obs in self.dynamic_obstacles:
            obs.reset()

    # ------------------------------------------------------------------
    # Scenario I/O
    # ------------------------------------------------------------------
    @classmethod
    def from_scenario_file(cls, path: str | Path) -> "GridEnvironment":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> "GridEnvironment":
        env = cls(width=data["width"], height=data["height"], seed=data.get("seed"))
        env.set_start(tuple(data["start"]))
        env.set_goal(tuple(data["goal"]))

        for rect in data.get("obstacle_rects", []):
            env.add_obstacle_rect(tuple(rect["top_left"]), tuple(rect["bottom_right"]))
        for cell in data.get("obstacle_cells", []):
            env.add_obstacle(tuple(cell))
        if "random_obstacle_density" in data:
            env.add_random_obstacles(data["random_obstacle_density"])

        for dyn in data.get("dynamic_obstacles", []):
            waypoints = [tuple(wp) for wp in dyn["waypoints"]]
            env.add_dynamic_obstacle(
                DynamicObstacle(
                    waypoints=waypoints,
                    speed_cells_per_step=dyn.get("speed_cells_per_step", 1),
                )
            )
        return env

    def snapshot_for_planning(self) -> "GridEnvironment":
        """Return a copy of this environment where every dynamic obstacle's
        *current* cell is baked into the static grid as an obstacle. Planners
        only ever see static grids, so this lets them route around movers
        without needing any time-aware logic of their own."""
        snap = GridEnvironment(self.width, self.height, seed=self.seed)
        snap.grid = self.grid.copy()
        for cell in self.dynamic_positions():
            if snap.in_bounds(cell):
                snap.grid[cell] = OBSTACLE
        snap.start = self.start
        snap.goal = self.goal
        snap.dynamic_obstacles = []
        return snap

    def copy(self) -> "GridEnvironment":
        """Deep-enough copy for replanning: grid + start/goal + dynamic obstacle state."""
        env = GridEnvironment(self.width, self.height, seed=self.seed)
        env.grid = self.grid.copy()
        env.start = self.start
        env.goal = self.goal
        env.dynamic_obstacles = self.dynamic_obstacles  # shared reference: same live obstacles
        return env
