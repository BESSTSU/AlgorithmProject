"""Lifelong demo: new goals as soon as one is reached, with the robots kept apart."""
import random
import signal
from dataclasses import replace

import matplotlib
import pytest

from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot, load_scenario
from execution.executor import Executor
from execution.lifelong import (LifelongRun, StopOnSignal, charger_distances, first_conflict,
                                lifelong_battery, run_lifelong)
from robots.grid_frame import GridFrame
from robots.grid_sim import GridSim
from tests.conftest import ROOT

DEMO = ROOT / "scenarios" / "demo_ab.yaml"


@pytest.fixture(scope="module")
def demo():
    return load_scenario(DEMO)


def run_demo(grid, robots, config, jobs, delay=(0.0, 0), seed=0, battery=None):
    sim = GridSim(grid, {r.name: r.start for r in robots}, *delay, random.Random(seed))
    run = LifelongRun(grid, robots, sim.robots, battery or lifelong_battery(config), config,
                      max_jobs=jobs)
    run_lifelong(run, sim.step)
    return sim, run


def test_charger_distances(arena_map):
    dist = charger_distances(arena_map)
    assert dist[(2, 5)] == 0
    assert (dist[(0, 0)], dist[(0, 1)], dist[(4, 4)], dist[(4, 3)]) == (7, 6, 5, 4)


def test_first_conflict():
    apart = {"A": [(0, 0), (0, 1)], "B": [(1, 0), (1, 1)]}
    assert first_conflict(apart) is None
    assert "both on" in first_conflict({"A": [(0, 0), (0, 1)], "B": [(0, 2), (0, 1)]})
    # B steps into the cell A has just left: forbidden by the clearance rule.
    assert "one step apart" in first_conflict({"A": [(0, 0), (0, 1)], "B": [(1, 0), (0, 0)]})
    # A parks on (0, 1); B arriving there long after is still a clash.
    assert first_conflict({"A": [(0, 1)], "B": [(0, 3), (0, 2), (0, 2), (0, 1)]}) is not None


def test_plan_avoids_fixed_paths(arena_map, battery, config):
    planner = config["planner"]
    other = [(2, 3), (2, 2), (2, 1), (1, 1), (0, 1)]  # coming down the corridor towards room A
    robot = Robot("R1", (2, 0), battery.max, ((4, 4),))
    result = plan_prioritized(arena_map, [robot], battery, planner["t_max"], "fixed", 0,
                              random.Random(0), fixed=(other,))
    assert result.paths is not None
    assert first_conflict({"R1": result.paths["R1"], "other": other}) is None


def test_remaining_path_of_a_moving_robot(battery):
    grid = GridMap(["...."])
    sim = GridSim(grid, {"A": (0, 0)}, 0.0, 0, random.Random(0))
    ex = Executor(grid, {"A": [(0, 0), (0, 0), (0, 1), (0, 2)]}, sim.robots, {"A": 20}, battery)
    # Standing on (0, 0): the whole planned stay there is still ahead.
    assert ex.remaining_path("A") == [(0, 0), (0, 0), (0, 1), (0, 2)]
    ex.update()  # nobody to wait for, so it sets off at once
    assert ex.remaining_path("A") == [(0, 0), (0, 1), (0, 2)]  # next cell at t = 1
    sim.step()
    ex.tick += 1
    ex.update()  # on (0, 1), already moving on
    assert ex.remaining_path("A") == [(0, 1), (0, 2)]


def test_demo_runs_jobs_in_cycle_order(arena_map, demo, config):
    sim, run = run_demo(arena_map, demo, config, jobs=20)
    assert len(run.jobs) == 20 and sim.collisions == 0
    for robot in demo:
        goals = [job.goal for job in run.jobs if job.robot == robot.name]
        expected = [robot.tasks[i % len(robot.tasks)] for i in range(len(goals))]
        assert goals == expected
        assert any(job.charged for job in run.jobs if job.robot == robot.name)
        assert sim.robots[robot.name].get_cell() == goals[-1]  # parked on its last goal
    # The small battery forces charging stops, yet it never runs out.
    assert run.lowest_battery() >= 0


@pytest.mark.parametrize("delay", [(0.2, 2), (0.5, 3), (0.8, 4)])
def test_demo_with_delays_never_collides(arena_map, demo, config, delay):
    for seed in range(10):
        sim, run = run_demo(arena_map, demo, config, jobs=60, delay=delay, seed=seed)
        assert len(run.jobs) == 60
        assert sim.collisions == 0
        assert run.lowest_battery() >= 0


def test_jobs_must_end_within_reach_of_a_charger(arena_map, demo, config):
    # (0, 0) is 7 cells from the charger: a 12-cell battery cannot go there and back.
    small = replace(lifelong_battery(config), max=12)
    sim = GridSim(arena_map, {r.name: r.start for r in demo}, 0.0, 0, random.Random(0))
    with pytest.raises(ValueError, match="too far from a charger"):
        LifelongRun(arena_map, demo, sim.robots, small, config)


def test_every_job_ends_with_the_way_to_the_charger(arena_map, demo, config):
    sim, run = run_demo(arena_map, demo, config, jobs=30)
    dist = charger_distances(arena_map)
    for job in run.jobs:
        assert job.battery >= run.battery.move_cost * dist[job.goal]


def test_filling_up_means_fewer_charging_stops(arena_map, demo, config):
    charged = {}
    for fill_up in (False, True):
        cfg = {**config, "lifelong": {**config["lifelong"], "fill_up": fill_up}}
        sim, run = run_demo(arena_map, demo, cfg, jobs=40)
        assert sim.collisions == 0 and run.lowest_battery() >= 0
        charged[fill_up] = sum(job.charged for job in run.jobs)
        if fill_up:  # charging goes all the way up
            levels = [lv for levels in run.executor.levels.values() for lv in levels]
            assert max(levels) == run.battery.max
    assert charged[True] < charged[False]


def test_ctrl_c_becomes_a_stop_request(arena_map, demo, config):
    sim = GridSim(arena_map, {r.name: r.start for r in demo}, 0.0, 0, random.Random(0))
    run = LifelongRun(arena_map, demo, sim.robots, lifelong_battery(config), config)

    def on_tick(r):
        if r.executor.tick == 30:
            signal.raise_signal(signal.SIGINT)  # as if Ctrl-C was pressed mid-run

    previous = signal.getsignal(signal.SIGINT)
    with StopOnSignal() as stop:
        finished = run_lifelong(run, sim.step, on_tick, lambda: stop.requested)
    assert not finished and run.executor.tick == 30
    assert signal.getsignal(signal.SIGINT) is previous


def test_grid_coordinates_of_cell_centers():
    frame = GridFrame(5, 0.45, x=0.3, y=-0.2, yaw=0.5)
    for cell in [(0, 0), (2, 4), (4, 5)]:
        assert frame.to_grid(frame.to_metric(cell)) == pytest.approx(cell)


def test_live_view_draws(arena_map, demo, config, tmp_path):
    matplotlib.use("Agg")
    from viz.live_view import LiveView
    sim, run = run_demo(arena_map, demo, config, jobs=3)
    view = LiveView(arena_map, run.names, run.battery.max, refresh_hz=1000)
    view.show(run, {name: (*sim.robots[name].get_cell(), 0.0) for name in run.names}, 75.0)
    view.fig.savefig(tmp_path / "view.png")
    assert (tmp_path / "view.png").stat().st_size > 0
