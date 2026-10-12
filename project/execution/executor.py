"""Plan executor: runs paths in ADG order, not by the clock, and tracks the real battery.

The executor only uses RobotInterface and a `step` callable that advances the
world by one tick, so the same code drives grid_sim, CoppeliaSim and real robots.
"""
from dataclasses import dataclass
from typing import Callable

from core.battery import BatteryModel
from core.grid_map import CHARGER, Cell, GridMap
from execution.adg import ActionDependencyGraph
from robots.base import RobotInterface


@dataclass(frozen=True)
class ExecutionResult:
    finish_ticks: dict[str, int]   # tick at which each robot completed its plan
    battery: dict[str, list[int]]  # real battery level after each tick, until the robot finished

    @property
    def makespan(self) -> int:
        return max(self.finish_ticks.values())

    @property
    def lowest_battery(self) -> int:
        return min(min(levels) for levels in self.battery.values())


def execute(
    grid: GridMap,
    paths: dict[str, list[Cell]],
    robots: dict[str, RobotInterface],
    start_levels: dict[str, int],
    battery: BatteryModel,
    step: Callable[[], None],
) -> ExecutionResult:
    """Drive every robot along its planned path and return what actually happened.

    Each robot must already stand on the first cell of its path. Per tick:
      - a robot starts its next move once the moves it depends on are completed
        (a started move costs move_cost; every further tick it takes costs wait_cost);
      - otherwise it waits (wait_cost), or charges if it stands on a charger;
      - on a charger it stays at least as long as planned, and until its real
        battery reaches the level the plan has when leaving that cell. Waiting on a
        charger is free, so the planner parks spare time there; leaving early would
        turn it into costly waiting elsewhere. Time lost on the way is made up by
        charging longer.
    The real battery may go negative; that is recorded, not enforced.
    Raises RuntimeError if no robot can make progress any more.
    """
    adg = ActionDependencyGraph(paths)
    planned = {name: battery.trace(grid, paths[name], start_levels[name]) for name in paths}
    completed = {name: 0 for name in paths}  # moves completed by each robot
    levels = {name: [start_levels[name]] for name in paths}
    arrived = {name: 0 for name in paths}    # tick at which each robot reached its current cell
    moving: set[str] = set()
    finish_ticks: dict[str, int] = {}
    tick = 0
    while True:
        for name in list(moving):
            if robots[name].is_arrived():
                moving.discard(name)
                completed[name] += 1
                arrived[name] = tick

        held_on_charger = False
        for name in paths:
            if name in finish_ticks:
                continue
            level = levels[name][-1]
            if name in moving:  # a move that takes longer than one tick
                level -= battery.wait_cost
            else:
                k = completed[name]
                visit = adg.visits[name][k]
                ready = True
                if grid.symbol(visit.cell) == CHARGER:
                    ready = (tick - arrived[name] >= visit.t_out - visit.t_in
                             and level >= planned[name][visit.t_out])
                    held_on_charger = held_on_charger or not ready
                if ready and k == adg.n_moves(name):
                    finish_ticks[name] = tick
                    continue
                if ready and adg.can_start(name, k, completed):
                    target = adg.visits[name][k + 1].cell
                    robots[name].go_to_cell(target)
                    moving.add(name)
                    level = battery.next_level(grid, level, visit.cell, target)
                else:  # wait for other robots, or stay on the charger
                    level = battery.next_level(grid, level, visit.cell, visit.cell)
            levels[name].append(level)
            robots[name].battery = level

        if len(finish_ticks) == len(paths):
            return ExecutionResult(finish_ticks, levels)
        if not moving and not held_on_charger:
            waiting = sorted(set(paths) - set(finish_ticks))
            raise RuntimeError(f"deadlock at tick {tick}: {waiting} wait on each other")
        step()
        tick += 1
