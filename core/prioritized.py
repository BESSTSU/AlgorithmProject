"""Prioritized planning: plan robots one at a time, each avoiding earlier reservations."""
import random
from dataclasses import dataclass

from core.battery import BatteryModel
from core.grid_map import Cell, GridMap
from core.reservation import ReservationTable
from core.scenario import Robot
from core.st_astar import failure_reason, plan_path


@dataclass(frozen=True)
class PlanResult:
    paths: dict[str, list[Cell]] | None  # keyed by robot name in input order; None if failed
    failure: str | None = None           # FAIL_BATTERY or FAIL_NO_PATH when planning failed
    failed_robot: str | None = None      # the robot that could not be planned


def plan_with_charging(grid: GridMap, robot: Robot, battery: BatteryModel, t_max: int,
                       reservations: ReservationTable) -> tuple[list[Cell] | None, str | None]:
    """Main method: one search over the robot's whole task list, with charging as an action.

    Returns (path, None) or (None, failure reason).
    """
    path = plan_path(grid, robot.start, robot.tasks, robot.battery, battery, t_max, reservations)
    if path is None:
        return None, failure_reason(grid, robot.start, robot.tasks, battery, t_max, reservations)
    return path, None


def plan_prioritized(
    grid: GridMap,
    robots: list[Robot],
    battery: BatteryModel,
    t_max: int,
    priority: str,
    max_restarts: int,
    rng: random.Random,
    plan_robot=plan_with_charging,
    fixed: tuple[list[Cell], ...] = (),
) -> PlanResult:
    """Plan all robots in priority order with the single-robot planner `plan_robot`.

    `fixed` are paths of other robots that are already under way (time 0 = now);
    every robot planned here avoids them, and each one parks on its last cell.

    priority:
      "fixed"            plan in the given robot order (robot number order)
      "battery_urgency"  lowest starting battery first
      "random_restart"   try the given order, then up to `max_restarts` shuffled orders

    If every order fails, the result reports the failure of the first one.
    """
    if priority == "fixed":
        orders = [list(robots)]
    elif priority == "battery_urgency":
        orders = [sorted(robots, key=lambda r: r.battery)]
    elif priority == "random_restart":
        orders = [list(robots)]
        for _ in range(max_restarts):
            shuffled = list(robots)
            rng.shuffle(shuffled)
            orders.append(shuffled)
    else:
        raise ValueError(f"unknown priority strategy {priority!r}")

    first_failure = None
    for order in orders:
        result = _plan_in_order(grid, order, battery, t_max, plan_robot, fixed)
        if result.paths is not None:
            return PlanResult({r.name: result.paths[r.name] for r in robots})
        first_failure = first_failure or result
    return first_failure


def _plan_in_order(grid, order, battery, t_max, plan_robot, fixed) -> PlanResult:
    table = ReservationTable()
    for path in fixed:
        table.reserve_path(path)
    for robot in order:
        table.reserve_start(robot.start)
    paths = {}
    for robot in order:
        table.release_start(robot.start)
        path, failure = plan_robot(grid, robot, battery, t_max, table)
        if path is None:
            return PlanResult(None, failure, robot.name)
        table.reserve_path(path)
        paths[robot.name] = path
    return PlanResult(paths)
