"""Rule-based baseline: charge to full whenever the battery is below a threshold.

It uses the same space-time A*, reservation table and prioritized planning as
the main method. The only difference is how charging is decided: by a fixed
rule checked before each task, not inside the search.
"""
import random
from collections import deque
from dataclasses import replace
from functools import partial

from core.battery import BatteryModel
from core.grid_map import CHARGER, Cell, GridMap
from core.prioritized import PlanResult, plan_prioritized
from core.reservation import ReservationTable
from core.scenario import Robot
from core.st_astar import FAIL_NO_PATH, failure_reason, plan_path


def nearest_charger(grid: GridMap, cell: Cell) -> Cell | None:
    """Charger cell with the shortest walking distance from `cell`, or None if unreachable."""
    seen = {cell}
    queue = deque([cell])
    while queue:
        current = queue.popleft()
        if grid.symbol(current) == CHARGER:
            return current
        for nxt in grid.neighbors(current):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return None


def plan_rule_based(grid: GridMap, robot: Robot, battery: BatteryModel, t_max: int,
                    reservations: ReservationTable,
                    threshold: float) -> tuple[list[Cell] | None, str | None]:
    """Plan one robot segment by segment. Returns (path, None) or (None, failure reason).

    Before each task, if the battery is below `threshold` of the maximum, a
    segment "go to the nearest charger and charge to full" is inserted. Task
    segments are searched with charging disabled.
    """
    path = [robot.start]
    level = robot.battery
    transit = replace(battery, min_final=0)           # no reserve needed between segments
    charged = replace(battery, min_final=battery.max)  # a charging segment ends with a full battery

    # (goal, battery model, charging allowed, must be able to park)
    for i, task in enumerate(robot.tasks):
        last = i == len(robot.tasks) - 1
        segments = [(task, battery if last else transit, False, last)]
        if level < threshold * battery.max:
            charger = nearest_charger(grid, path[-1])
            if charger is None:
                return None, FAIL_NO_PATH
            segments.insert(0, (charger, charged, True, False))
        for goal, model, charging, park in segments:
            t_start = len(path) - 1
            segment = plan_path(grid, path[-1], [goal], level, model, t_max, reservations,
                                allow_charging=charging, t_start=t_start, park=park)
            if segment is None:
                return None, failure_reason(grid, path[-1], [goal], battery, t_max,
                                            reservations, t_start=t_start, park=park)
            level = model.trace(grid, segment, level, charging)[-1]
            path.extend(segment[1:])
    return path, None


def plan_baseline(
    grid: GridMap,
    robots: list[Robot],
    battery: BatteryModel,
    t_max: int,
    priority: str,
    max_restarts: int,
    rng: random.Random,
    threshold: float,
) -> PlanResult:
    """Prioritized planning with the rule-based single-robot planner."""
    return plan_prioritized(grid, robots, battery, t_max, priority, max_restarts, rng,
                            plan_robot=partial(plan_rule_based, threshold=threshold))
