"""M4: rule-based baseline and how it differs from the main method.

Scenario numbers assume the default battery values (SPEC 4.2) and the default
baseline threshold of 0.3, so the rule triggers below 12.
"""
import random

from core.baseline import nearest_charger, plan_baseline
from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.reservation import ReservationTable
from core.scenario import Robot
from core.st_astar import FAIL_BATTERY, FAIL_HORIZON, FAIL_NO_PATH, failure_reason, plan_path
from experiments.compare import compare
from experiments.metrics import plan_metrics
from tests.helpers import assert_valid_path, charging_times, find_conflicts

CHARGER = (3, 10)


def plan_both(grid, robots, battery, config, priority="fixed"):
    """(main result, baseline result) under identical planner settings."""
    planner = config["planner"]
    seed = config["experiments"]["seed"]
    common = (grid, robots, battery, planner["t_max"], priority, planner["max_restarts"])
    return (plan_prioritized(*common, random.Random(seed)),
            plan_baseline(*common, random.Random(seed), config["baseline"]["threshold"]))


def assert_solution(grid, robots, battery, result):
    assert result.paths is not None, f"failed: {result.failure} ({result.failed_robot})"
    for robot in robots:
        path = result.paths[robot.name]
        assert_valid_path(grid, path, robot.start, robot.tasks[-1])
        levels = battery.trace(grid, path, robot.battery)
        assert min(levels) >= 0 and levels[-1] >= battery.min_final
    assert find_conflicts(result.paths) == []


# --- A* extensions used to chain segments -----------------------------------

def test_segment_uses_absolute_time(battery):
    grid = GridMap(["...", "..."])
    table = ReservationTable()
    # Another robot is on (0,1) only at t=5, then parks on (1,1).
    table.reserve_path([(1, 1)] * 5 + [(0, 1), (1, 1)])
    early = plan_path(grid, (0, 0), [(0, 2)], battery.max, battery, 20, table, t_start=0)
    assert early == [(0, 0), (0, 1), (0, 2)]
    # Starting at t=4, (0,1) is closed at t=5 and t=6 (clearance), so it has to wait twice.
    late = plan_path(grid, (0, 0), [(0, 2)], battery.max, battery, 20, table, t_start=4)
    assert late == [(0, 0), (0, 0), (0, 0), (0, 1), (0, 2)]


def test_segment_without_parking_ends_on_arrival(battery):
    grid = GridMap(["...", "..."])
    table = ReservationTable()
    table.reserve_path([(1, 1), (1, 1), (1, 1), (0, 1), (0, 2)])  # passes (0,1) at t=3
    parked = plan_path(grid, (0, 0), [(0, 1)], battery.max, battery, 20, table)
    assert len(parked) - 1 == 5
    segment = plan_path(grid, (0, 0), [(0, 1)], battery.max, battery, 20, table, park=False)
    assert segment == [(0, 0), (0, 1)]


def test_failure_reason(battery, config):
    t_max = config["planner"]["t_max"]
    open_grid = GridMap(["..."])
    assert plan_path(open_grid, (0, 0), [(0, 2)], 1, battery, t_max) is None
    assert failure_reason(open_grid, (0, 0), [(0, 2)], battery, t_max) == FAIL_BATTERY
    walled = GridMap(["..#.."])
    assert failure_reason(walled, (0, 0), [(0, 4)], battery, t_max) == FAIL_NO_PATH
    # Two steps are needed but the horizon allows one.
    assert plan_path(open_grid, (0, 0), [(0, 2)], battery.max, battery, 1) is None
    assert failure_reason(open_grid, (0, 0), [(0, 2)], battery, 1) == FAIL_HORIZON


# --- the charging rule -------------------------------------------------------

def test_nearest_charger_uses_walking_distance(battery, config):
    # From (0,2) the left charger is 2 cells away in a straight line but 6 steps
    # around the wall; the right charger is 5 steps away.
    grid = GridMap(["C#.....C", ".#......", "........"])
    assert nearest_charger(grid, (0, 2)) == (0, 7)
    assert nearest_charger(GridMap(["..#C"]), (0, 0)) is None
    # With 11 the robot reaches the right charger (10) but not the left one (12).
    robots = [Robot("R1", (0, 2), 11, ((2, 4),))]
    _, baseline = plan_both(grid, robots, battery, config)
    assert_solution(grid, robots, battery, baseline)
    assert (0, 7) in baseline.paths["R1"] and (0, 0) not in baseline.paths["R1"]


def test_rule_charges_to_full_below_threshold(challenge_map, battery, config):
    robots = [Robot("R1", (3, 8), 10, ((0, 6),))]
    main, baseline = plan_both(challenge_map, robots, battery, config)
    assert_solution(challenge_map, robots, battery, main)
    assert_solution(challenge_map, robots, battery, baseline)
    # Baseline: 2 steps to the charger, 7 to fill up from 6, then 7 to the task.
    path = baseline.paths["R1"]
    assert len(path) - 1 == 16
    assert len(charging_times(challenge_map, path)) == 7
    assert max(battery.trace(challenge_map, path, 10)) == battery.max
    # Main method: charges only the 4 steps the task needs.
    assert len(main.paths["R1"]) - 1 == 13
    assert len(charging_times(challenge_map, main.paths["R1"])) == 4


def test_rule_does_not_trigger_at_threshold(challenge_map, battery, config):
    # 12 is not below 30% of 40, so the baseline heads straight for the task and
    # arrives with 2, under the required final reserve.
    robots = [Robot("R1", (3, 8), 12, ((0, 6),))]
    main, baseline = plan_both(challenge_map, robots, battery, config)
    assert_solution(challenge_map, robots, battery, main)
    assert baseline.paths is None
    assert (baseline.failure, baseline.failed_robot) == (FAIL_BATTERY, "R1")


def test_rule_is_checked_before_each_task(battery, config):
    grid = GridMap(["..C..", "....."])
    # 13 passes the check before task 1; after one move it is 11, below the threshold.
    robots = [Robot("R1", (1, 0), 13, ((1, 1), (1, 4)))]
    main, baseline = plan_both(grid, robots, battery, config)
    assert_solution(grid, robots, battery, main)
    assert_solution(grid, robots, battery, baseline)
    path = baseline.paths["R1"]
    assert path[1] == (1, 1)                    # task 1 first, no charging before it
    charging = charging_times(grid, path)
    assert min(charging) > 1 and len(charging) == 7
    assert len(path) - 1 == 13
    assert len(main.paths["R1"]) - 1 == 8       # main charges 2 steps on the way


# --- charger contention ------------------------------------------------------

def test_waits_for_occupied_charger(battery, config):
    grid = GridMap([".....", "..C..", "....."])
    # R1 starts on the charger, R2 right next to it. Both are below the threshold.
    robots = [Robot("R1", (1, 2), 11, ((1, 4),)), Robot("R2", (0, 2), 11, ((1, 0),))]
    main, baseline = plan_both(grid, robots, battery, config)
    assert_solution(grid, robots, battery, main)
    assert_solution(grid, robots, battery, baseline)
    r1_charging = charging_times(grid, baseline.paths["R1"])
    r2_charging = charging_times(grid, baseline.paths["R2"])
    assert min(r2_charging) > max(r1_charging)  # R2 queues until R1 is full
    base_metrics, main_metrics = plan_metrics(grid, baseline.paths), plan_metrics(grid, main.paths)
    assert base_metrics["wait_steps"] == 7
    assert (main_metrics["makespan"], base_metrics["makespan"]) == (7, 18)


def test_robot_can_die_waiting_for_charger(battery, config):
    grid = GridMap([".....", "..C..", "....."])
    # R1 needs 7 steps to fill up. R2 cannot afford to wait that long.
    robots = [Robot("R1", (1, 1), 11, ((1, 4),)), Robot("R2", (1, 3), 11, ((1, 0),))]
    main, baseline = plan_both(grid, robots, battery, config)
    assert_solution(grid, robots, battery, main)
    assert baseline.paths is None
    assert (baseline.failure, baseline.failed_robot) == (FAIL_BATTERY, "R2")


# --- comparison with the main method ----------------------------------------

def test_same_plan_when_nobody_needs_charging(challenge_map, battery, config):
    robots = [Robot("R1", (3, 2), battery.max, ((3, 10),)),
              Robot("R2", (3, 8), battery.max, ((0, 1),))]
    main, baseline = plan_both(challenge_map, robots, battery, config)
    assert main.paths is not None and main.paths == baseline.paths


def test_no_path_failure_is_reported_by_both_methods(challenge_map, battery, config):
    # R1 parks in the single corridor, so R2 can never get past.
    robots = [Robot("R1", (3, 4), battery.max, ((3, 5),)),
              Robot("R2", (3, 8), battery.max, ((3, 0),))]
    for result in plan_both(challenge_map, robots, battery, config):
        assert result.paths is None
        assert (result.failure, result.failed_robot) == (FAIL_NO_PATH, "R2")


def test_s0_baseline_fails_where_main_succeeds(challenge_map, battery, config, challenge_s0):
    results = compare(challenge_map, challenge_s0, battery, config)
    main, baseline = results["main"], results["baseline"]
    assert_solution(challenge_map, challenge_s0, battery, main)
    assert plan_metrics(challenge_map, main.paths) == {
        "makespan": 19, "sum_of_costs": 33, "wait_steps": 2, "charge_steps": 4, "charge_visits": 1,
    }
    # R1 starts with 20, above the threshold, so the rule never sends it to charge.
    assert baseline.paths is None
    assert (baseline.failure, baseline.failed_robot) == (FAIL_BATTERY, "R1")


def test_s1_baseline_is_slower(challenge_map, battery, config, challenge_s1):
    results = compare(challenge_map, challenge_s1, battery, config)
    main, baseline = results["main"], results["baseline"]
    assert_solution(challenge_map, challenge_s1, battery, main)
    assert_solution(challenge_map, challenge_s1, battery, baseline)
    assert plan_metrics(challenge_map, main.paths) == {
        "makespan": 14, "sum_of_costs": 27, "wait_steps": 2, "charge_steps": 4, "charge_visits": 1,
    }
    assert plan_metrics(challenge_map, baseline.paths) == {
        "makespan": 17, "sum_of_costs": 33, "wait_steps": 5, "charge_steps": 7, "charge_visits": 1,
    }
