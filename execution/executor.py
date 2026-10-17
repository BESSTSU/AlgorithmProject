"""Plan executor: runs paths in ADG order, not by the clock, and tracks the real battery.

The executor only uses RobotInterface and a `step` callable that advances the
world by one tick, so the same code drives grid_sim, CoppeliaSim and real robots.
"""
import math
import random
from dataclasses import dataclass, replace
from typing import Callable

from core.battery import BatteryModel
from core.grid_map import CHARGER, Cell, GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot
from execution.adg import ActionDependencyGraph
from robots.base import RobotInterface

_EPSILON = 1e-9  # tolerance for battery levels accumulated in fractions of a step


@dataclass(frozen=True)
class ExecutionResult:
    finish_ticks: dict[str, int]     # tick at which each robot completed its plan
    battery: dict[str, list[float]]  # real battery level after each tick, until the robot finished

    @property
    def makespan(self) -> int:
        return max(self.finish_ticks.values())

    @property
    def lowest_battery(self) -> float:
        return min(min(levels) for levels in self.battery.values())


def execute(
    grid: GridMap,
    paths: dict[str, list[Cell]],
    robots: dict[str, RobotInterface],
    start_levels: dict[str, int],
    battery: BatteryModel,
    step: Callable[[], None],
    steps_per_tick: float = 1,
) -> ExecutionResult:
    """Drive every robot along its planned path and return what actually happened.

    Each robot must already stand on the first cell of its path. The rules of a
    tick are described in Executor.update().
    Raises RuntimeError if no robot can make progress any more.
    """
    executor = Executor(grid, paths, robots, start_levels, battery, steps_per_tick)
    while not executor.update():
        step()
        executor.tick += 1
    return ExecutionResult(executor.finish_ticks, executor.levels)


class Executor:
    """The executor's state, advanced one tick at a time by update().

    execute() runs it until every robot is done. The lifelong mode
    (execution/lifelong.py) also gives robots that are done a new path with
    replan(), while the others carry on with the rest of theirs.
    """

    def __init__(self, grid: GridMap, paths: dict[str, list[Cell]],
                 robots: dict[str, RobotInterface], start_levels: dict[str, float],
                 battery: BatteryModel, steps_per_tick: float = 1, idle_battery: bool = False,
                 fill_up: bool = False):
        """`steps_per_tick` says how many plan steps one tick lasts. It is 1 in
        grid_sim, where a tick is a plan step, and (tick duration / duration of one
        plan step) on a backend that runs in real time. All battery rates are
        scaled by it. With `idle_battery`, robots that are done keep waiting (or
        charging on a charger) instead of dropping out of the battery bookkeeping.
        With `fill_up`, a robot that stops on a charger stays until it is full, not
        only until it has what the plan needs.
        """
        self.grid, self.robots, self.battery = grid, robots, battery
        self.steps_per_tick = steps_per_tick
        self.ticks_per_step = round(1 / steps_per_tick)
        self.idle_battery = idle_battery
        self.fill_up = fill_up
        self.tick = 0
        self.levels = {name: [start_levels[name]] for name in paths}
        self.arrived = {name: 0 for name in paths}  # tick at which each robot reached its current cell
        self.started = {name: 0 for name in paths}  # tick at which each robot began its current move
        self.moving: set[str] = set()
        self.finish_ticks: dict[str, int] = {}
        self._use(paths, {name: battery.trace(grid, paths[name], start_levels[name]) for name in paths})

    def _use(self, paths: dict[str, list[Cell]], planned: dict[str, list[float]]) -> None:
        self.paths = paths
        self.planned = planned  # planned battery level at every time step of each path
        self.adg = ActionDependencyGraph(paths)
        self.completed = {name: 0 for name in paths}  # moves of the current paths completed

    def update(self) -> bool:
        """Process one tick; True once every robot is done. Call step() after it.

        Per tick:
          - a robot starts its next move once the moves it depends on are completed.
            Starting costs move_cost, which covers one plan step of driving; any time
            the move takes beyond that is paid at the waiting rate;
          - otherwise it waits (wait_cost per plan step), or charges if it stands on a
            charger (charge_rate per plan step);
          - on a charger it stays at least as long as planned, and until its real
            battery reaches the level the plan has when leaving that cell. Waiting on a
            charger is free, so the planner parks spare time there; leaving early would
            turn it into costly waiting elsewhere. Time lost on the way is made up by
            charging longer.
        The real battery may go negative; that is recorded, not enforced.
        Raises RuntimeError if no robot can make progress any more.
        """
        grid, battery, adg = self.grid, self.battery, self.adg
        for name in list(self.moving):
            if self.robots[name].is_arrived():
                self.moving.discard(name)
                self.completed[name] += 1
                self.arrived[name] = self.tick

        held_on_charger = False
        for name in self.paths:
            level = self.levels[name][-1]
            if name in self.finish_ticks:
                if not self.idle_battery:
                    continue
                level = self._wait(name, level)
            elif name in self.moving:
                if self.tick - self.started[name] >= self.ticks_per_step:  # slower than one plan step
                    level -= battery.wait_cost * self.steps_per_tick
            else:
                k = self.completed[name]
                visit = adg.visits[name][k]
                on_charger = grid.symbol(visit.cell) == CHARGER
                ready = True
                if on_charger:
                    dwell_ticks = (visit.t_out - visit.t_in) * self.ticks_per_step
                    target = self.planned[name][visit.t_out]
                    if self.fill_up and visit.t_out > visit.t_in:  # a planned stop, not driving across
                        target = battery.max
                    ready = (self.tick - self.arrived[name] >= dwell_ticks
                             and level >= target - _EPSILON)
                    held_on_charger = held_on_charger or not ready
                if ready and k == adg.n_moves(name):
                    self.finish_ticks[name] = self.tick
                    continue
                if ready and adg.can_start(name, k, self.completed):
                    self.robots[name].go_to_cell(adg.visits[name][k + 1].cell)
                    self.moving.add(name)
                    self.started[name] = self.tick
                    level -= battery.move_cost
                else:  # charge or stay parked on the charger, or wait for other robots
                    level = self._wait(name, level)
            self.levels[name].append(level)
            self.robots[name].battery = level

        if len(self.finish_ticks) == len(self.paths):
            return True
        if not self.moving and not held_on_charger:
            waiting = sorted(set(self.paths) - set(self.finish_ticks))
            raise RuntimeError(f"deadlock at tick {self.tick}: {waiting} wait on each other")
        return False

    def _wait(self, name: str, level: float) -> float:
        """Battery after one tick standing still: charging on a charger, waiting elsewhere."""
        cell = self.adg.visits[name][self.completed[name]].cell
        if self.grid.symbol(cell) == CHARGER:
            return min(level + self.battery.charge_rate * self.steps_per_tick, self.battery.max)
        return level - self.battery.wait_cost * self.steps_per_tick

    # --- lifelong mode ---------------------------------------------------------

    def remaining_path(self, name: str) -> list[Cell]:
        """The rest of a robot's path from where it is now, as a path starting at t = 0.

        A robot standing on a cell starts with the whole planned stay on it. A robot
        on the way to its next cell starts on the cell it is leaving and is on the
        next one at t = 1, so a plan made against this path cannot use that cell first.
        """
        return self.paths[name][self._offset(name):]

    def _offset(self, name: str) -> int:
        visits, k = self.adg.visits[name], self.completed[name]
        return visits[k + 1].t_in - 1 if name in self.moving else visits[k].t_in

    def replan(self, new_paths: dict[str, list[Cell]]) -> None:
        """Give robots that are done a new path; every other robot keeps the rest of its own.

        Each new path starts at t = 0 on the robot's current cell and must have been
        planned against remaining_path() of all the other robots, so that the one
        dependency graph built over all of them keeps everyone apart.
        """
        for name in new_paths:
            if name not in self.finish_ticks:
                raise ValueError(f"{name} still has a path to follow")
        paths, planned = {}, {}
        for name in self.paths:
            if name in new_paths:
                start_level = math.floor(self.levels[name][-1] + _EPSILON)
                paths[name] = new_paths[name]
                planned[name] = self.battery.trace(self.grid, new_paths[name], start_level)
                del self.finish_ticks[name]
                self.arrived[name] = self.tick
            else:
                offset = self._offset(name)
                paths[name] = self.paths[name][offset:]
                planned[name] = self.planned[name][offset:]
        self._use(paths, planned)

    def status(self, name: str) -> str:
        """'done', 'moving', 'charging' or 'waiting'."""
        if name in self.finish_ticks:
            return "done"
        if name in self.moving:
            return "moving"
        cell = self.adg.visits[name][self.completed[name]].cell
        return "charging" if self.grid.symbol(cell) == CHARGER else "waiting"


def plan_for_execution(grid: GridMap, robots: list[Robot],
                       config: dict) -> tuple[dict[str, list[Cell]], BatteryModel]:
    """Plan a scenario the way the real-time backends run it: with `execution.reserve`.

    Returns (paths, battery model used). Raises RuntimeError if planning fails.
    """
    planner = config["planner"]
    battery = replace(BatteryModel(**config["battery"]), reserve=config["execution"]["reserve"])
    plan = plan_prioritized(grid, robots, battery, planner["t_max"], planner["priority"],
                            planner["max_restarts"], random.Random(config["experiments"]["seed"]))
    if plan.paths is None:
        raise RuntimeError(f"planning failed: {plan.failure} ({plan.failed_robot})")
    return plan.paths, battery


def summary_lines(robots: list[Robot], result: ExecutionResult, paths: dict[str, list[Cell]],
                  seconds_per_tick: float, step_seconds: float) -> list[str]:
    """One line per robot: real against planned finish time, and its battery."""
    lines = []
    for robot in robots:
        levels = result.battery[robot.name]
        real = result.finish_ticks[robot.name] * seconds_per_tick
        planned = (len(paths[robot.name]) - 1) * step_seconds
        lines.append(f"{robot.name}: reached {robot.tasks[-1]} at {real:.1f} s (plan {planned:.0f} s),"
                     f" battery {levels[0]} -> {levels[-1]:.1f}, lowest {min(levels):.1f}")
    return lines
