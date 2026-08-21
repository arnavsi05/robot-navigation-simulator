"""Dijkstra's algorithm for shortest-path planning on a 2D occupancy grid.

Dijkstra explores nodes in order of known cost-from-start g(n), with no
heuristic guiding the search toward the goal. It is guaranteed optimal and
serves as the baseline we compare A* against: A* should reach the same path
cost while expanding fewer nodes, since its heuristic focuses the search.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Tuple

from .common import Cell, PlanResult, neighbors, path_length, reconstruct_path


def dijkstra(env, start: Cell, goal: Cell) -> PlanResult:
    """Run Dijkstra's algorithm from start to goal on the given GridEnvironment."""
    t0 = time.perf_counter()

    if not env.is_free(start, include_dynamic=False) or not env.is_free(goal, include_dynamic=False):
        return PlanResult(False, [], time.perf_counter() - t0, 0, 0.0, "dijkstra")

    open_heap: list[Tuple[float, int, Cell]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start))

    g_score = {start: 0.0}
    came_from: dict = {}
    closed = set()
    nodes_expanded = 0

    while open_heap:
        cost, _, current = heapq.heappop(open_heap)
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
                "dijkstra",
            )

        for neighbor, step_cost in neighbors(env, current):
            if neighbor in closed:
                continue
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(open_heap, (tentative_g, counter, neighbor))

    return PlanResult(False, [], time.perf_counter() - t0, nodes_expanded, 0.0, "dijkstra")
