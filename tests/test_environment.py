from pathlib import Path

from src.environment import DynamicObstacle, GridEnvironment

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"


def test_basic_grid_free_and_obstacle_cells():
    env = GridEnvironment(width=5, height=5)
    env.set_start((0, 0))
    env.set_goal((4, 4))
    env.add_obstacle((2, 2))

    assert env.is_free((0, 0))
    assert not env.is_free((2, 2))
    assert not env.is_free((-1, 0))  # out of bounds
    assert not env.is_free((0, 5))  # out of bounds


def test_obstacle_rect_fills_block():
    env = GridEnvironment(width=10, height=10)
    env.add_obstacle_rect((2, 2), (4, 4))
    for r in range(2, 5):
        for c in range(2, 5):
            assert not env.is_static_free((r, c))
    assert env.is_static_free((0, 0))
    assert env.is_static_free((5, 5))


def test_random_obstacles_deterministic_with_seed():
    env_a = GridEnvironment(width=20, height=20, seed=42)
    env_a.add_random_obstacles(0.2)
    env_b = GridEnvironment(width=20, height=20, seed=42)
    env_b.add_random_obstacles(0.2)
    assert (env_a.grid == env_b.grid).all()


def test_dynamic_obstacle_patrols_and_bounces():
    obs = DynamicObstacle(waypoints=[(0, 0), (0, 3)])
    positions = [obs.position]
    for _ in range(8):
        positions.append(obs.step())
    # Should walk 0->3 then bounce back toward 0.
    assert positions[:5] == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 2)]


def test_scenarios_load_and_are_solvable_in_principle():
    for name in ["simple.json", "complex.json", "dynamic.json"]:
        env = GridEnvironment.from_scenario_file(SCENARIOS / name)
        assert env.start is not None and env.goal is not None
        assert env.is_free(env.start)
        assert env.is_free(env.goal)


def test_path_collision_free_detects_obstacle_overlap():
    env = GridEnvironment(width=5, height=5)
    env.add_obstacle((1, 1))
    assert env.is_path_collision_free([(0, 0), (0, 1), (0, 2)])
    assert not env.is_path_collision_free([(0, 0), (1, 1), (2, 2)])
