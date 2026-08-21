"""A* path planning on a 2D occupancy grid.

A* explores nodes in order of f(n) = g(n) + h(n), where g(n) is the known
cost from the start and h(n) is a heuristic estimate of the remaining cost
to the goal. With an admissible heuristic (one that never overestimates the
true cost), A* is guaranteed to find the shortest path while typically
expanding far fewer nodes than Dijkstra's algorithm.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Tuple

from .common import Cell, PlanResult, neighbors, path_length, reconstruct_path


def manhattan(a: Cell, b: Cell) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def octile(a: Cell, b: Cell) -> float:
    """Admissible heuristic for 8-connected grids (matches diagonal cost)."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


_HEURISTICS = {"manhattan": manhattan, "euclidean": euclidean, "octile": octile}


def astar(env, start: Cell, goal: Cell, heuristic: str = "octile") -> PlanResult:
    """Run A* search from start to goal on the given GridEnvironment.

    Returns a PlanResult with the path (empty if no path exists), whether
    planning succeeded, wall-clock planning time, and the number of nodes
    expanded (useful for benchmarking against Dijkstra).
    """
    t0 = time.perf_counter()
    h = _HEURISTICS[heuristic]

    if not env.is_free(start, include_dynamic=False) or not env.is_free(goal, include_dynamic=False):
        return PlanResult(False, [], time.perf_counter() - t0, 0, 0.0, "astar")

    open_heap: list[Tuple[float, int, Cell]] = []
    counter = 0  # tie-breaker so heap never compares Cell tuples directly
    heapq.heappush(open_heap, (h(start, goal), counter, start))

    g_score = {start: 0.0}
    came_from: dict = {}
    closed = set()
    nodes_expanded = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        nodes_expanded += 1

        if current == goal:
            path = reconstruct_path(came_from, current)
            return PlanResult(
                True,
                path,
                time.perf_counter() - t0,
                nodes_expanded,
                path_length(path),
                "astar",
                metadata={"heuristic": heuristic},
            )

        for neighbor, cost in neighbors(env, current):
            if neighbor in closed:
                continue
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                counter += 1
                f = tentative_g + h(neighbor, goal)
                heapq.heappush(open_heap, (f, counter, neighbor))

    return PlanResult(False, [], time.perf_counter() - t0, nodes_expanded, 0.0, "astar")
