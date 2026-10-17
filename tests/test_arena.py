"""The real arena: thin walls between cells, the arena map, and its scenario S0."""
import random
from dataclasses import replace

import pytest

from core.baseline import plan_baseline
from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import load_scenario_map, random_scenario
from core.st_astar import FAIL_BATTERY, plan_path
from tests.conftest import ROOT
from tests.helpers import assert_valid_path, charging_times, find_conflicts, plan_single
from tests.test_execution import NO_DELAY, all_at_goal, run
from viz.animate import animate_paths

CORRIDOR_ROW = 2


@pytest.fixture(scope="module")
def run_battery(battery, config):
    """The battery model used when planning for CoppeliaSim and the real robots."""
    return replace(battery, reserve=config["execution"]["reserve"])


def plan(grid, robots, battery, config, priority="fixed"):
    planner = config["planner"]
    return plan_prioritized(grid, robots, battery, planner["t_max"], priority,
                            planner["max_restarts"], random.Random(config["experiments"]["seed"]))


# --- thin walls ----------------------------------------------------------------

def test_thin_wall_separates_neighbouring_cells(battery):
    open_grid = GridMap(["..", ".."])
    walled = GridMap(["..", "..", "walls: (0,0)-(1,0)"])
    assert (1, 0) in open_grid.neighbors((0, 0))
    assert walled.neighbors((0, 0)) == [(0, 1)] and (0, 0) not in walled.neighbors((1, 0))
    assert walled.edge_walls == [((0, 0), (1, 0))]
    assert (walled.height, walled.width) == (2, 2)  # the wall line is not a grid row
    # The planner goes around the wall: three moves instead of one.
    assert plan_single(open_grid, (0, 0), (1, 0), battery, t_max=20) == [(0, 0), (1, 0)]
    assert plan_single(walled, (0, 0), (1, 0), battery, t_max=20) == [(0, 0), (0, 1), (1, 1), (1, 0)]


def test_neighbours_are_never_diagonal(arena_map):
    for r in range(arena_map.height):
        for c in range(arena_map.width):
            if arena_map.is_passable((r, c)):
                for nr, nc in arena_map.neighbors((r, c)):
                    assert abs(nr - r) + abs(nc - c) == 1


@pytest.mark.parametrize("line", [
    "walls: (0,0)-(1,1)",    # not neighbours
    "walls: (0,0)-(0,1)",    # (0,1) is a wall cell
    "walls: (0,0)-(1,0) x",  # unreadable
    "walls: (0,0)-(5,0)",    # outside the map
])
def test_bad_wall_lists_are_rejected(line):
    with pytest.raises(ValueError):
        GridMap([".#", "..", line])


# --- the arena map -------------------------------------------------------------

def test_arena_layout(arena_map):
    assert (arena_map.height, arena_map.width) == (5, 6)
    passable = [(r, c) for r in range(5) for c in range(6) if arena_map.is_passable((r, c))]
    assert len(passable) == 15
    assert arena_map.cells_with("C") == [(2, 5)]
    assert arena_map.cells_with("A") == [(0, 0), (0, 1), (1, 0), (1, 1)]
    assert arena_map.cells_with("B") == [(3, 3), (3, 4), (4, 3), (4, 4)]
    assert [cell for cell in passable if arena_map.is_side_pocket(cell)] == [(1, 4)]


def test_arena_rooms_have_one_entrance_each(arena_map):
    # Top room: only (1,1) opens onto the corridor; (1,0) is walled off from (2,0).
    assert (2, 0) not in arena_map.neighbors((1, 0))
    assert (2, 1) in arena_map.neighbors((1, 1))
    # Bottom room: only (3,3) opens onto the corridor; (3,4) is walled off from (2,4).
    assert (2, 4) not in arena_map.neighbors((3, 4))
    assert (2, 3) in arena_map.neighbors((3, 3))
    assert arena_map.neighbors((2, 5)) == [(2, 4)]  # the charger is a dead end


def test_scenario_file_names_its_map(config):
    config_path = ROOT / "config.yaml"
    arena = load_scenario_map(ROOT / "scenarios" / "s0.yaml", config_path, config)
    challenge = load_scenario_map(ROOT / "scenarios" / "challenge_s0.yaml", config_path, config)
    assert (arena.height, arena.width) == (5, 6)
    assert (challenge.height, challenge.width) == (5, 11)


def test_animation_draws_the_arena(arena_map, battery, config, arena_s0, tmp_path):
    paths = plan(arena_map, arena_s0, battery, config).paths
    out = tmp_path / "arena.gif"
    animate_paths(arena_map, paths, out, config["viz"]["frame_ms"])
    assert out.stat().st_size > 0


# --- scenario S0 on the arena ---------------------------------------------------

def test_s0_r1_must_charge(arena_map, battery, run_battery, config, arena_s0):
    r1 = arena_s0[0]
    t_max = config["planner"]["t_max"]
    for model in (battery, run_battery):
        assert plan_path(arena_map, r1.start, r1.tasks, r1.battery, model, t_max,
                         allow_charging=False) is None
        path = plan_path(arena_map, r1.start, r1.tasks, r1.battery, model, t_max)
        assert charging_times(arena_map, path)
        levels = model.trace(arena_map, path, r1.battery)
        assert min(levels) >= model.reserve and levels[-1] >= model.min_final


def test_s0_r1_battery_range(arena_map, battery, run_battery, config, arena_s0):
    r1 = arena_s0[0]
    t_max = config["planner"]["t_max"]

    def needs_charging_and_can(level, model):
        return (plan_path(arena_map, r1.start, r1.tasks, level, model, t_max,
                          allow_charging=False) is None
                and plan_path(arena_map, r1.start, r1.tasks, level, model, t_max) is not None)

    # The task costs 22 from R1's start and the charger is 10 away.
    assert [b for b in range(5, 30) if needs_charging_and_can(b, battery)] == list(range(10, 22))
    assert [b for b in range(5, 30) if needs_charging_and_can(b, run_battery)] == list(range(15, 22))
    assert r1.battery in range(15, 22)


def test_s0_plan(arena_map, run_battery, config, arena_s0):
    paths = plan(arena_map, arena_s0, run_battery, config).paths
    assert paths is not None
    for robot in arena_s0:
        assert_valid_path(arena_map, paths[robot.name], robot.start, robot.tasks[-1])
        levels = run_battery.trace(arena_map, paths[robot.name], robot.battery)
        assert min(levels) >= run_battery.reserve and levels[-1] >= run_battery.min_final
    assert find_conflicts(paths) == []
    # R1: 5 moves to the charger, 3 charge steps, 5 moves to its task.
    assert len(paths["R1"]) - 1 == 13
    assert charging_times(arena_map, paths["R1"]) == [5, 6, 7]
    # R2 has to leave the corridor to let R1 through, then goes on to the top room.
    assert any(r != CORRIDOR_ROW for r, _ in paths["R2"][:6])
    assert len(paths["R2"]) - 1 == 10


def test_s0_other_methods_fail(arena_map, battery, run_battery, config, arena_s0):
    planner = config["planner"]
    baseline = plan_baseline(arena_map, arena_s0, battery, planner["t_max"], "fixed",
                             planner["max_restarts"], random.Random(0),
                             config["baseline"]["threshold"])
    # R1 starts above the rule's threshold, so the baseline never sends it to charge.
    assert (baseline.paths, baseline.failure, baseline.failed_robot) == (None, FAIL_BATTERY, "R1")
    # With R2 planned first, R1 cannot afford to give way.
    reversed_order = plan(arena_map, arena_s0[::-1], run_battery, config)
    assert (reversed_order.paths, reversed_order.failure) == (None, FAIL_BATTERY)
    assert plan(arena_map, arena_s0[::-1], run_battery, config, "battery_urgency").paths is not None


# --- execution on the arena ------------------------------------------------------

def test_s0_executes_as_planned_without_delay(arena_map, run_battery, config, arena_s0):
    paths = plan(arena_map, arena_s0, run_battery, config).paths
    sim, result = run(arena_map, arena_s0, paths, run_battery, NO_DELAY, random.Random(0))
    assert sim.collisions == 0 and all_at_goal(sim, arena_s0)
    assert result.finish_ticks == {name: len(path) - 1 for name, path in paths.items()}
    for robot in arena_s0:
        planned = run_battery.trace(arena_map, paths[robot.name], robot.battery)
        assert result.battery[robot.name] == planned


def test_random_arena_scenarios_never_collide(arena_map, run_battery, config):
    gen = config["scenario_gen"]
    delay = (config["sim_delay"]["prob"], config["sim_delay"]["max_extra_steps"])
    rng = random.Random(config["experiments"]["seed"])
    solved = 0
    while solved < 60:
        robots = random_scenario(arena_map, rng, gen["n_robots"], gen["battery_min"],
                                 run_battery.max, gen["tasks_min"], gen["tasks_max"])
        paths = plan(arena_map, robots, run_battery, config, "random_restart").paths
        if paths is None:
            continue
        solved += 1
        # Without delays the execution is the plan; with delays it still never collides.
        sim, result = run(arena_map, robots, paths, run_battery, NO_DELAY, rng)
        assert sim.collisions == 0 and all_at_goal(sim, robots)
        assert result.finish_ticks == {name: len(path) - 1 for name, path in paths.items()}, robots
        assert result.lowest_battery >= run_battery.reserve, robots
        for _ in range(5):
            sim, _ = run(arena_map, robots, paths, run_battery, delay, rng)
            assert sim.collisions == 0 and all_at_goal(sim, robots), robots
