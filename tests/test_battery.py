"""M3: battery, charging inside the search, and ordered task lists.

Scenario numbers below assume the default battery values of SPEC section 4.2.
"""
import random
from dataclasses import replace

import pytest

from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot
from core.st_astar import plan_path
from tests.helpers import assert_valid_path, charging_times, find_conflicts

CHARGER = (3, 10)
CORRIDOR_ROW = 3


def assert_battery_ok(grid, path, start_level, battery):
    levels = battery.trace(grid, path, start_level)
    assert min(levels) >= 0
    assert levels[-1] >= battery.min_final


def prioritized(grid, robots, battery, config, priority):
    planner = config["planner"]
    return plan_prioritized(
        grid, robots, battery, planner["t_max"], priority,
        planner["max_restarts"], random.Random(config["experiments"]["seed"]),
    ).paths


# --- battery model ---------------------------------------------------------

def test_battery_model_steps(challenge_map, battery):
    corridor = (3, 9)
    level = battery.max // 2
    assert battery.next_level(challenge_map, level, corridor, CHARGER) == level - battery.move_cost
    assert battery.next_level(challenge_map, level, corridor, corridor) == level - battery.wait_cost
    assert battery.next_level(challenge_map, level, CHARGER, CHARGER) == level + battery.charge_rate
    # Charging is capped, and a full battery on the charger stays full.
    assert battery.next_level(challenge_map, battery.max - 1, CHARGER, CHARGER) == battery.max
    assert battery.next_level(challenge_map, battery.max, CHARGER, CHARGER) == battery.max
    # With charging disabled, waiting on the charger costs like any other wait.
    assert battery.next_level(challenge_map, level, CHARGER, CHARGER,
                              allow_charging=False) == level - battery.wait_cost


def test_battery_never_negative(battery, config):
    grid = GridMap(["..."])
    model = replace(battery, min_final=0)
    need = 2 * model.move_cost
    t_max = config["planner"]["t_max"]
    assert plan_path(grid, (0, 0), [(0, 2)], need - 1, model, t_max) is None
    path = plan_path(grid, (0, 0), [(0, 2)], need, model, t_max)
    assert model.trace(grid, path, need) == [need, need - model.move_cost, 0]


def test_min_final_battery_required(battery, config):
    grid = GridMap(["..."])
    need = 2 * battery.move_cost + battery.min_final
    t_max = config["planner"]["t_max"]
    assert plan_path(grid, (0, 0), [(0, 2)], need - 1, battery, t_max) is None
    path = plan_path(grid, (0, 0), [(0, 2)], need, battery, t_max)
    assert battery.trace(grid, path, need)[-1] == battery.min_final


def test_reserve_is_kept_at_every_step(battery, config):
    grid = GridMap(["..."])
    model = replace(battery, min_final=0, reserve=5)
    need = 2 * model.move_cost + model.reserve
    t_max = config["planner"]["t_max"]
    assert plan_path(grid, (0, 0), [(0, 2)], need - 1, model, t_max) is None
    path = plan_path(grid, (0, 0), [(0, 2)], need, model, t_max)
    assert min(model.trace(grid, path, need)) == model.reserve


def test_reserve_narrows_the_s0_battery_range(challenge_map, battery, config, challenge_s0):
    # R1 needs 16 to reach the charger. With a reserve of 5 it must arrive with 5 left.
    r1 = challenge_s0[0]
    model = replace(battery, reserve=5)
    t_max = config["planner"]["t_max"]
    assert plan_path(challenge_map, r1.start, r1.tasks, 20, model, t_max) is None
    path = plan_path(challenge_map, r1.start, r1.tasks, 21, model, t_max)
    assert min(model.trace(challenge_map, path, 21)) == 5


# --- ordered tasks (index k) -------------------------------------------------

def test_tasks_visited_in_order(challenge_map, battery, config):
    t_max = config["planner"]["t_max"]
    path = plan_path(challenge_map, (3, 4), [(1, 6), (1, 1)], battery.max, battery, t_max)
    assert_valid_path(challenge_map, path, (3, 4), (1, 1))
    assert len(path) - 1 == 13 and path[4] == (1, 6)
    # Same cells in the other order give a different plan.
    path = plan_path(challenge_map, (3, 4), [(1, 1), (1, 6)], battery.max, battery, t_max)
    assert_valid_path(challenge_map, path, (3, 4), (1, 6))
    assert len(path) - 1 == 14 and path[5] == (1, 1)


def test_charging_planned_across_the_task_sequence(challenge_map, battery, config):
    # Start next to the charger with 30. Both tasks together need more than a full
    # battery, so charging first cannot work: the fastest plan does task 1, returns
    # to charge exactly enough (6 steps) for task 2, then finishes.
    t_max = config["planner"]["t_max"]
    start, tasks, level = (3, 9), [(0, 6), (0, 1)], 30
    path = plan_path(challenge_map, start, tasks, level, battery, t_max)
    assert_valid_path(challenge_map, path, start, tasks[-1])
    assert_battery_ok(challenge_map, path, level, battery)
    assert len(path) - 1 == 31
    assert path[6] == (0, 6)
    assert charging_times(challenge_map, path) == list(range(13, 19))
    assert plan_path(challenge_map, start, tasks, level, battery, t_max, allow_charging=False) is None


# --- scenario S0 (SPEC section 3) -------------------------------------------

def test_s0_r1_needs_to_charge(challenge_map, battery, config, challenge_s0):
    r1 = challenge_s0[0]
    t_max = config["planner"]["t_max"]
    assert plan_path(challenge_map, r1.start, r1.tasks, r1.battery, battery, t_max,
                     allow_charging=False) is None
    path = plan_path(challenge_map, r1.start, r1.tasks, r1.battery, battery, t_max)
    assert_valid_path(challenge_map, path, r1.start, r1.tasks[-1])
    assert_battery_ok(challenge_map, path, r1.battery, battery)
    assert charging_times(challenge_map, path)


@pytest.mark.parametrize("level", range(16, 24))
def test_s0_r1_battery_range(challenge_map, battery, config, challenge_s0, level):
    # Every start level in 16..23 has the S0 property: fails without charging, succeeds with it.
    r1 = challenge_s0[0]
    t_max = config["planner"]["t_max"]
    assert plan_path(challenge_map, r1.start, r1.tasks, level, battery, t_max,
                     allow_charging=False) is None
    assert plan_path(challenge_map, r1.start, r1.tasks, level, battery, t_max) is not None


def test_s0_r1_battery_range_edges(challenge_map, battery, config, challenge_s0):
    r1 = challenge_s0[0]
    t_max = config["planner"]["t_max"]
    # 15 cannot even reach the charger; 24 does not need it.
    assert plan_path(challenge_map, r1.start, r1.tasks, 15, battery, t_max) is None
    path = plan_path(challenge_map, r1.start, r1.tasks, 24, battery, t_max, allow_charging=False)
    assert len(path) - 1 == 7


def test_s0_plan(challenge_map, battery, config, challenge_s0):
    r1, r2 = challenge_s0
    paths = prioritized(challenge_map, challenge_s0, battery, config, "fixed")
    assert paths is not None
    for robot in challenge_s0:
        assert_valid_path(challenge_map, paths[robot.name], robot.start, robot.tasks[-1])
        assert_battery_ok(challenge_map, paths[robot.name], robot.battery, battery)
    assert find_conflicts(paths) == []
    # R1 detours to the charger, charges, then does its task.
    assert charging_times(challenge_map, paths["R1"])
    assert len(paths["R1"]) - 1 == 19
    # R2 gives way to R1 by leaving the corridor between them.
    assert any(r != CORRIDOR_ROW and 3 <= c <= 8 for r, c in paths["R2"])
    assert len(paths["R2"]) - 1 == 14


def test_battery_urgency_plans_low_battery_first(challenge_map, battery, config, challenge_s0):
    r2_first = challenge_s0[::-1]
    # If full-battery R2 gets priority, low-battery R1 cannot afford to give way.
    assert prioritized(challenge_map, r2_first, battery, config, "fixed") is None
    # battery_urgency reorders so R1 plans first.
    paths = prioritized(challenge_map, r2_first, battery, config, "battery_urgency")
    assert paths == prioritized(challenge_map, challenge_s0, battery, config, "fixed")


# --- charger as a reserved resource ------------------------------------------

def test_charger_holds_one_robot_at_a_time(battery, config):
    grid = GridMap([".....", "..C..", "....."])
    robots = [Robot("R1", (1, 1), 12, ((1, 4),)), Robot("R2", (1, 3), 12, ((1, 0),))]
    t_max = config["planner"]["t_max"]
    for robot in robots:  # both really need the charger
        assert plan_path(grid, robot.start, robot.tasks, robot.battery, battery, t_max,
                         allow_charging=False) is None

    paths = prioritized(grid, robots, battery, config, "fixed")
    assert paths is not None
    for robot in robots:
        assert_valid_path(grid, paths[robot.name], robot.start, robot.tasks[-1])
        assert_battery_ok(grid, paths[robot.name], robot.battery, battery)
    assert find_conflicts(paths) == []
    r1_charging = charging_times(grid, paths["R1"])
    r2_charging = charging_times(grid, paths["R2"])
    assert r1_charging and r2_charging
    # R2 only gets the charger after R1 has finished with it.
    assert min(r2_charging) > max(r1_charging)
