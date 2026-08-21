from pathlib import Path

import pytest

from src.environment import GridEnvironment
from src.planners.astar import astar
from src.planners.dijkstra import dijkstra

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"


def test_dijkstra_finds_path_in_open_grid():
    env = GridEnvironment(width=10, height=10)
    env.set_start((0, 0))
    env.set_goal((9, 9))
    result = dijkstra(env, env.start, env.goal)
    assert result.success
    assert result.path[0] == (0, 0)
    assert result.path[-1] == (9, 9)


def test_dijkstra_path_avoids_obstacles():
    env = GridEnvironment(width=10, height=10)
    env.set_start((0, 0))
    env.set_goal((9, 9))
    env.add_obstacle_rect((3, 0), (6, 9))
    env.grid[3:7, 9] = 0  # punch a full-height gap at col 9 to keep it solvable
    result = dijkstra(env, env.start, env.goal)
    assert result.success
    assert env.is_path_collision_free(result.path)


def test_dijkstra_reports_failure_when_goal_unreachable():
    env = GridEnvironment(width=6, height=6)
    env.set_start((0, 0))
    env.set_goal((5, 5))
    env.add_obstacle_rect((2, 0), (2, 5))
    result = dijkstra(env, env.start, env.goal)
    assert not result.success


def test_astar_and_dijkstra_agree_on_optimal_cost():
    """A* with an admissible heuristic must find a path whose cost matches
    Dijkstra's optimal cost, even though it typically expands fewer nodes."""
    for name in ["simple.json", "complex.json"]:
        env = GridEnvironment.from_scenario_file(SCENARIOS / name)
        a_result = astar(env, env.start, env.goal)
        d_result = dijkstra(env, env.start, env.goal)
        assert a_result.success and d_result.success
        assert a_result.path_length == pytest.approx(d_result.path_length, abs=1e-6)
        assert a_result.nodes_expanded <= d_result.nodes_expanded
