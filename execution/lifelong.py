"""Lifelong demo: every robot takes a new goal as soon as it reaches one, on and on.

Each robot cycles through its scenario tasks, one job per cell. When a robot
reaches its goal it is planned again from where it stands, with the battery it
has left, against the remaining paths of the robots still under way (prioritized
planning with those paths fixed), and the executor swaps the new path in
(Executor.replan). The others never stop. The planner inserts charging stops
by itself; every job must also end with enough battery left to reach a charger
afterwards, so a robot is never stranded between two jobs.

Preview on the grid simulator, without ROS or robots:
    python -m execution.lifelong --scenario scenarios/demo_ab.yaml --view
The same run on the robots: python3 -m robots.turtlebot_ros2 --lifelong ...
"""
import argparse
import math
import random
import signal
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

import yaml

from core.battery import BatteryModel
from core.grid_map import CHARGER, Cell, GridMap
from core.prioritized import plan_prioritized, plan_with_charging
from core.scenario import Robot, load_scenario, load_scenario_map
from execution.executor import Executor
from robots.base import RobotInterface


@dataclass(frozen=True)
class Job:
    robot: str
    goal: Cell
    start_tick: int
    end_tick: int
    charged: bool   # the plan for this job stopped at a charger
    battery: float  # level left on reaching the goal


def lifelong_battery(config: dict) -> BatteryModel:
    """The demo's simulated battery, counted in cells (lifelong.battery in config.yaml)."""
    return BatteryModel(**config["lifelong"]["battery"])


def charger_distances(grid: GridMap) -> dict[Cell, int]:
    """Moves from every reachable cell to the nearest charger."""
    chargers = grid.cells_with(CHARGER)
    if not chargers:
        raise ValueError("the lifelong mode needs a charger on the map")
    dist = {cell: 0 for cell in chargers}
    queue = deque(chargers)
    while queue:
        cell = queue.popleft()
        for n in grid.neighbors(cell):
            if n not in dist:
                dist[n] = dist[cell] + 1
                queue.append(n)
    return dist


def first_conflict(paths: dict[str, list[Cell]]) -> str | None:
    """The first violation of the planner's separation rule, or None.

    Paths start together at t = 0 and a robot stays on its last cell afterwards.
    Two robots may never be on the same cell at the same time, nor may one enter
    a cell in the step after another left it (which also rules out swapping cells).
    """
    def at(path, t):
        return path[min(max(t, 0), len(path) - 1)]

    names = list(paths)
    horizon = max(len(p) for p in paths.values())
    for t in range(horizon):
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                pa, pb = paths[a], paths[b]
                if at(pa, t) == at(pb, t):
                    return f"{a} and {b} both on {at(pa, t)} at t={t}"
                if t > 0 and (at(pa, t) == at(pb, t - 1) or at(pb, t) == at(pa, t - 1)):
                    return f"{a} and {b} one step apart on the same cell at t={t}"
    return None


class LifelongRun:
    """Executor plus job assignment. Call update() once per tick, then step the world."""

    def __init__(self, grid: GridMap, robots: list[Robot], backends: dict[str, RobotInterface],
                 battery: BatteryModel, config: dict, steps_per_tick: float = 1,
                 max_jobs: int | None = None):
        self.grid, self.battery = grid, battery
        self.names = [r.name for r in robots]
        self.cycles = {r.name: r.tasks for r in robots}
        self.max_jobs = max_jobs
        self._planner = config["planner"]
        self._rng = random.Random(config["experiments"]["seed"])
        self._to_charger = charger_distances(grid)
        for r in robots:
            for goal in r.tasks:  # from a full charge to the goal, with enough left to come back
                needed = battery.move_cost * self._to_charger[goal] + self._need_at(goal)
                if needed > battery.max:
                    raise ValueError(f"{r.name}: goal {goal} is too far from a charger for a battery "
                                     f"of {battery.max} (the trip there and back needs {needed})")
        self.next_task = {name: 0 for name in self.names}  # index into the robot's cycle
        self.goal: dict[str, Cell | None] = {name: None for name in self.names}
        self.job_start: dict[str, int] = {}
        self.job_charges: dict[str, bool] = {}
        self.jobs: list[Job] = []
        self.waiting = set(self.names)  # robots without a job, waiting to be planned
        self._failed: frozenset[str] = frozenset()  # waiting set of the last failed planning
        # Every robot starts as if it had just finished a job on its start cell.
        self.executor = Executor(grid, {r.name: [r.start] for r in robots}, backends,
                                 {r.name: r.battery for r in robots}, battery, steps_per_tick,
                                 idle_battery=True, fill_up=config["lifelong"]["fill_up"])

    def _need_at(self, goal: Cell) -> int:
        """Battery a job must leave on its goal: enough to drive on to the charger.

        That is all it takes when waiting is free (wait_cost 0, as in the demo's
        battery): a robot that stands still between jobs or for others keeps it.
        """
        return self.battery.move_cost * self._to_charger[goal] + self.battery.reserve

    @property
    def accepting(self) -> bool:
        return self.max_jobs is None or len(self.jobs) + self._in_progress() < self.max_jobs

    def _in_progress(self) -> int:
        return sum(1 for name in self.names if self.goal[name] is not None)

    def update(self) -> bool:
        """Process one tick; True once no more jobs will be given and every robot is done."""
        ex = self.executor
        ex.update()
        for name in self.names:
            if name in ex.finish_ticks and name not in self.waiting:
                self.jobs.append(Job(name, self.goal[name], self.job_start[name], ex.tick,
                                     self.job_charges[name], ex.levels[name][-1]))
                self.goal[name] = None
                self.waiting.add(name)
        if self.waiting and self.accepting and frozenset(self.waiting) != self._failed:
            self._assign()
        return not self.accepting and len(self.waiting) == len(self.names)

    def _assign(self) -> None:
        ex = self.executor
        order = [name for name in self.names if name in self.waiting]
        if self.max_jobs is not None:
            order = order[:self.max_jobs - len(self.jobs) - self._in_progress()]
        robots = []
        for name in order:
            cell = ex.remaining_path(name)[0]
            goal = self.cycles[name][self.next_task[name] % len(self.cycles[name])]
            level = max(0, math.floor(ex.levels[name][-1] + 1e-9))
            robots.append(Robot(name, cell, level, (goal,)))
        fixed = tuple(ex.remaining_path(name) for name in self.names if name not in self.waiting)
        result = plan_prioritized(self.grid, robots, self.battery, self._planner["t_max"],
                                  self._planner["priority"], self._planner["max_restarts"],
                                  self._rng, self._plan_job, fixed)
        if result.paths is None:
            if len(self.waiting) == len(self.names):
                raise RuntimeError(f"no plan for any robot ({result.failure}, {result.failed_robot})")
            self._failed = frozenset(self.waiting)  # retry once another robot is done
            return
        # Belt and braces before anything moves: the new paths, together with what the
        # other robots still have to drive, must keep every robot apart.
        combined = {name: ex.remaining_path(name) for name in self.names}
        combined.update(result.paths)
        clash = first_conflict(combined)
        if clash:
            raise RuntimeError(f"refusing a plan that would bring robots together: {clash}")
        ex.replan(result.paths)
        for robot in robots:
            name = robot.name
            self.goal[name] = robot.tasks[0]
            self.next_task[name] += 1
            self.job_start[name] = ex.tick
            self.job_charges[name] = any(self.grid.symbol(c) == CHARGER for c in result.paths[name][1:])
            self.waiting.discard(name)
        self._failed = frozenset()

    def _plan_job(self, grid, robot, battery, t_max, table):
        need = max(battery.min_final, self._need_at(robot.tasks[-1]))
        return plan_with_charging(grid, robot, replace(battery, min_final=need), t_max, table)

    def status(self, name: str) -> str:
        if name in self.waiting:
            return "idle" if not self.accepting else "planning"
        return self.executor.status(name)

    def lowest_battery(self) -> float:
        return min(min(levels) for levels in self.executor.levels.values())


def run_lifelong(run: LifelongRun, step: Callable[[], None],
                 on_tick: Callable[[LifelongRun], None] | None = None,
                 should_stop: Callable[[], bool] | None = None) -> bool:
    """Drive a LifelongRun until it is finished (forever if it has no max_jobs).

    Returns False if `should_stop` ended it early. The caller stops the robots.
    """
    while True:
        done = run.update()
        if on_tick:
            on_tick(run)
        if done:
            return True
        if should_stop and should_stop():
            return False
        step()
        run.executor.tick += 1


class StopOnSignal:
    """Turns Ctrl-C and SIGTERM into a flag, checked between ticks with `requested`.

    A GUI event loop (the live view) catches a KeyboardInterrupt raised while it
    is drawing, prints it and carries on, so the default Ctrl-C handling cannot
    be relied on to stop the robots.
    """

    def __enter__(self) -> "StopOnSignal":
        self.requested = False
        self._previous = {s: signal.signal(s, self._request) for s in (signal.SIGINT, signal.SIGTERM)}
        return self

    def _request(self, *_) -> None:
        self.requested = True

    def __exit__(self, *_) -> None:
        for s, handler in self._previous.items():
            signal.signal(s, handler)


def summary_lines(run: LifelongRun, seconds_per_tick: float) -> list[str]:
    lines = []
    for name in run.names:
        jobs = [j for j in run.jobs if j.robot == name]
        charged = sum(j.charged for j in jobs)
        levels = run.executor.levels[name]
        avg = (sum(j.end_tick - j.start_tick for j in jobs) / len(jobs) * seconds_per_tick) if jobs else 0
        lines.append(f"{name}: {len(jobs)} jobs ({charged} with a charging stop), "
                     f"{avg:.1f} s per job, battery now {levels[-1]:.1f}, lowest {min(levels):.1f}")
    return lines


def main():
    from robots.grid_sim import GridSim  # noqa: PLC0415  (only the preview needs it)

    parser = argparse.ArgumentParser(description="Lifelong demo on the grid simulator.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/demo_ab.yaml")
    parser.add_argument("--jobs", type=int, default=None, help="stop after this many jobs (default: never)")
    parser.add_argument("--view", action="store_true", help="show the live view, paced in real time")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = load_scenario_map(args.scenario, config_path, config)
    robots = load_scenario(args.scenario)
    sim = GridSim(grid, {r.name: r.start for r in robots}, 0.0, 0, random.Random(0))
    run = LifelongRun(grid, robots, sim.robots, lifelong_battery(config), config, max_jobs=args.jobs)
    tick_seconds = config["lifelong"]["sim_tick_seconds"]
    on_tick, view = None, None
    if args.view:
        from viz.live_view import LiveView  # noqa: PLC0415
        view = LiveView(grid, run.names, run.battery.max, config["lifelong"]["view_hz"])

        def on_tick(r):
            view.show(r, {name: (*sim.robots[name].get_cell(), None) for name in r.names},
                      r.executor.tick * tick_seconds)

        def step():
            sim.step()
            view.pause(tick_seconds)
    else:
        step = sim.step
    with StopOnSignal() as stop:  # Ctrl-C, or closing the window, ends the preview
        run_lifelong(run, step, on_tick, lambda: stop.requested or (view is not None and view.closed))
    print(f"{len(run.jobs)} jobs in {run.executor.tick} ticks, collisions: {sim.collisions}")
    for line in summary_lines(run, 1.0):
        print(line)


if __name__ == "__main__":
    main()
