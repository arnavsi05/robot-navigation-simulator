"""Simulation loop: plans a path, drives the robot along it, moves dynamic
obstacles, detects path invalidation, and triggers replanning.

Dynamic obstacles are handled with a "snapshot planning" strategy: whenever
the planner is invoked (initially, or after a replan trigger), the current
position of every dynamic obstacle is baked into the grid as a temporary
static obstacle. This keeps the planner implementations themselves free of
any time-dependent logic, at the cost of not reasoning about where a moving
obstacle will be several steps in the future -- a reasonable simplification
for a grid-based simulator (see README "Limitations").
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from .environment import GridEnvironment
from .robot import Robot

Cell = Tuple[int, int]
PlannerFn = Callable[[GridEnvironment, Cell, Cell], "object"]  # returns PlanResult


@dataclass
class FrameSnapshot:
    """One rendered frame's worth of state, used for GIF generation."""

    step: int
    robot_position: Cell
    dynamic_positions: List[Cell]
    path: List[Cell]
    replanned: bool = False


@dataclass
class SimulationResult:
    success: bool
    steps_taken: int
    replan_count: int
    total_planning_time_sec: float
    initial_path: List[Cell]
    final_path_length: float
    failure_reason: Optional[str] = None
    frames: List[FrameSnapshot] = field(default_factory=list)


class Simulation:
    """Drives a robot from start to goal in a GridEnvironment, replanning
    around dynamic obstacles as needed."""

    def __init__(
        self,
        env: GridEnvironment,
        planner: PlannerFn,
        max_steps: int = 300,
        max_replans: int = 50,
    ):
        self.env = env
        self.planner = planner
        self.max_steps = max_steps
        self.max_replans = max_replans

    def _plan_from(self, start: Cell) -> "object":
        """Plan a path from `start` to the environment goal, treating each
        dynamic obstacle's current cell as a temporary static obstacle."""
        snapshot = self.env.snapshot_for_planning()
        snapshot.start = start
        return self.planner(snapshot, start, self.env.goal)

    def _path_blocked(self, remaining_path: List[Cell]) -> bool:
        dynamic_cells = set(self.env.dynamic_positions())
        return any(cell in dynamic_cells for cell in remaining_path)

    def run(self, record_frames: bool = False) -> SimulationResult:
        env = self.env
        robot = Robot(env.start, env.goal)

        initial_result = self._plan_from(robot.position)
        total_planning_time = initial_result.planning_time
        if not initial_result.success:
            return SimulationResult(
                success=False,
                steps_taken=0,
                replan_count=0,
                total_planning_time_sec=total_planning_time,
                initial_path=[],
                final_path_length=0.0,
                failure_reason="no_initial_path",
            )

        robot.set_path(initial_result.path)
        replan_count = 0
        frames: List[FrameSnapshot] = []
        final_path_length = initial_result.path_length

        if record_frames:
            frames.append(
                FrameSnapshot(0, robot.position, env.dynamic_positions(), list(robot.path))
            )

        step = 0
        failure_reason = None
        while not robot.at_goal():
            if step >= self.max_steps:
                failure_reason = "max_steps_exceeded"
                break

            env.step_dynamic_obstacles()
            replanned_this_step = False

            if self._path_blocked(robot.remaining_path()):
                if replan_count >= self.max_replans:
                    failure_reason = "max_replans_exceeded"
                    break
                result = self._plan_from(robot.position)
                total_planning_time += result.planning_time
                replan_count += 1
                replanned_this_step = True
                if not result.success:
                    failure_reason = "no_path_after_replan"
                    break
                robot.set_path(result.path)
                final_path_length += result.path_length

            robot.step()
            step += 1

            if record_frames:
                frames.append(
                    FrameSnapshot(
                        step,
                        robot.position,
                        env.dynamic_positions(),
                        list(robot.path),
                        replanned=replanned_this_step,
                    )
                )

        success = robot.at_goal() and failure_reason is None
        return SimulationResult(
            success=success,
            steps_taken=step,
            replan_count=replan_count,
            total_planning_time_sec=total_planning_time,
            initial_path=initial_result.path,
            final_path_length=final_path_length,
            failure_reason=failure_reason,
            frames=frames,
        )
