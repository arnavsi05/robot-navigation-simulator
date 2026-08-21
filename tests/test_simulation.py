from pathlib import Path

from src.environment import DynamicObstacle, GridEnvironment
from src.planners.astar import astar
from src.robot import Robot
from src.simulation import Simulation

SCENARIOS = Path(__file__).resolve().parent.parent / "scenarios"


def test_robot_follows_path_step_by_step():
    robot = Robot(start=(0, 0), goal=(0, 2))
    robot.set_path([(0, 0), (0, 1), (0, 2)])
    assert robot.step() == (0, 1)
    assert robot.step() == (0, 2)
    assert robot.at_goal()
    assert robot.step() is None


def test_simulation_succeeds_with_no_dynamic_obstacles():
    env = GridEnvironment.from_scenario_file(SCENARIOS / "simple.json")
    sim = Simulation(env, astar)
    result = sim.run()
    assert result.success
    assert result.replan_count == 0


def test_replanning_is_triggered_when_dynamic_obstacle_blocks_path():
    """A dynamic obstacle that starts clear of the robot's straight-line
    route but then moves onto it must force at least one replan. The
    obstacle starts in the wall gap (off the row-1 corridor) so the initial
    plan is the direct route; it then steps into the corridor, invalidating
    that route and requiring a detour through the same gap."""
    env = GridEnvironment(width=10, height=3)
    env.set_start((1, 0))
    env.set_goal((1, 9))
    # Walls above and below force the robot through row 1 -- a corridor.
    env.add_obstacle_rect((0, 0), (0, 9))
    env.add_obstacle_rect((2, 0), (2, 9))
    # A 3-wide gap so the detour is still reachable even while the obstacle
    # itself sits on the middle gap column (corner-cutting is disallowed,
    # so a single-column gap would become unreachable once blocked).
    env.grid[0, 4:7] = 0
    env.grid[2, 4:7] = 0

    # Patrols vertically through the gap column, starting outside the
    # corridor row so it does not affect the initial plan.
    obstacle = DynamicObstacle(waypoints=[(0, 5), (2, 5)])
    env.add_dynamic_obstacle(obstacle)

    sim = Simulation(env, astar, max_steps=100)
    result = sim.run()

    assert result.success
    assert result.replan_count >= 1


def test_dynamic_scenario_file_requires_replanning_or_succeeds():
    env = GridEnvironment.from_scenario_file(SCENARIOS / "dynamic.json")
    sim = Simulation(env, astar, max_steps=200)
    result = sim.run()
    assert result.success
