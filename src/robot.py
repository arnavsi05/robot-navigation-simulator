"""A simple mobile robot abstraction that follows a planned path.

The robot has no physically accurate dynamics (no velocity/acceleration
limits, no wheel kinematics). It moves cell-by-cell along its current path,
one cell per simulation step, which is sufficient for evaluating planning
and replanning behavior.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

Cell = Tuple[int, int]


class Robot:
    """A point robot that tracks a position and follows a path of grid cells."""

    def __init__(self, start: Cell, goal: Cell):
        self.position: Cell = tuple(start)
        self.goal: Cell = tuple(goal)
        self.path: List[Cell] = []
        self.path_index: int = 0
        self.history: List[Cell] = [self.position]

    def set_path(self, path: List[Cell]) -> None:
        """Assign a new path to follow. The path must start at the robot's
        current position (or immediately adjacent to it after a replan)."""
        self.path = list(path)
        self.path_index = 0
        # If the path includes the robot's current cell as its first
        # waypoint, skip it so the next step moves forward.
        if self.path and self.path[0] == self.position:
            self.path_index = 1

    def at_goal(self) -> bool:
        return self.position == self.goal

    def remaining_path(self) -> List[Cell]:
        """The portion of the path not yet traversed, including the current cell."""
        return [self.position] + self.path[self.path_index :]

    def step(self) -> Optional[Cell]:
        """Advance one cell along the current path. Returns the new position,
        or None if there is no path left to follow."""
        if self.path_index >= len(self.path):
            return None
        self.position = self.path[self.path_index]
        self.path_index += 1
        self.history.append(self.position)
        return self.position
