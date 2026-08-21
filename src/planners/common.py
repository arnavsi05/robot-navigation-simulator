"""Shared data structures and grid-movement helpers used by both planners."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Cell = Tuple[int, int]

# 8-connected movement: (row delta, col delta, step cost)
_MOVES = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (1, 1, math.sqrt(2)),
]


@dataclass
class PlanResult:
    """Standardized output of a path-planning call."""

    success: bool
    path: List[Cell]
    planning_time: float
    nodes_expanded: int
    path_length: float
    algorithm: str
    metadata: dict = field(default_factory=dict)


def neighbors(env, cell: Cell, allow_diagonal: bool = True):
    """Yield (neighbor_cell, step_cost) pairs that are free to move into.

    Diagonal moves are blocked if both orthogonal cells adjacent to the
    diagonal are obstacles ("corner cutting" prevention), which keeps paths
    physically plausible for a robot with non-zero footprint.
    """
    r, c = cell
    moves = _MOVES if allow_diagonal else _MOVES[:4]
    for dr, dc, cost in moves:
        nr, nc = r + dr, c + dc
        neighbor = (nr, nc)
        if not env.is_free(neighbor, include_dynamic=False):
            continue
        if dr != 0 and dc != 0:
            # Prevent cutting across a diagonal formed by two blocked cells.
            side_a = (r + dr, c)
            side_b = (r, c + dc)
            if not env.is_free(side_a, include_dynamic=False) and not env.is_free(
                side_b, include_dynamic=False
            ):
                continue
        yield neighbor, cost


def reconstruct_path(came_from: dict, current: Cell) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def path_length(path: List[Cell]) -> float:
    """Euclidean length of a cell path (sum of segment distances)."""
    total = 0.0
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        total += math.hypot(r1 - r0, c1 - c0)
    return total
