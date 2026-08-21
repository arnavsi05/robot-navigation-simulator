from pathlib import Path

from src.environment import GridEnvironment
from src.planners.astar import astar

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"


def test_astar_finds_path_in_open_grid():
    env = GridEnvironment(width=10, height=10)
    env.set_start((0, 0))
    env.set_goal((9, 9))
    result = astar(env, env.start, env.goal)
    assert result.success
    assert result.path[0] == (0, 0)
    assert result.path[-1] == (9, 9)


def test_astar_path_avoids_obstacles():
    env = GridEnvironment(width=10, height=10)
    env.set_start((0, 0))
    env.set_goal((9, 9))
    env.add_obstacle_rect((3, 0), (6, 9))  # full-width wall
    env.grid[3:7, 9] = 0  # punch a full-height gap at col 9 to keep it solvable
    result = astar(env, env.start, env.goal)
    assert result.success
    assert env.is_path_collision_free(result.path)


def test_astar_reports_failure_when_goal_unreachable():
    env = GridEnvironment(width=6, height=6)
    env.set_start((0, 0))
    env.set_goal((5, 5))
    env.add_obstacle_rect((2, 0), (2, 5))  # full-width wall, no gap
    result = astar(env, env.start, env.goal)
    assert not result.success
    assert result.path == []


def test_astar_on_all_scenario_files():
    for name in ["simple.json", "complex.json", "dynamic.json"]:
        env = GridEnvironment.from_scenario_file(SCENARIOS / name)
        result = astar(env, env.start, env.goal)
        assert result.success, f"scenario {name} should be solvable by A*"
        assert env.is_path_collision_free(result.path)
        assert result.path[0] == env.start
        assert result.path[-1] == env.goal
