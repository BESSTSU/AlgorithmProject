import random

from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot
from tests.helpers import assert_valid_path, find_conflicts

CORRIDOR_ROW = 3


def run(grid, robots, battery, config, priority=None):
    planner = config["planner"]
    return plan_prioritized(
        grid, robots, battery, planner["t_max"], priority or planner["priority"],
        planner["max_restarts"], random.Random(config["experiments"]["seed"]),
    ).paths


def assert_solution(grid, robots, paths):
    assert paths is not None
    for robot in robots:
        assert_valid_path(grid, paths[robot.name], robot.start, robot.tasks[-1])
    assert find_conflicts(paths) == []


def test_crossing_robots_one_dodges(challenge_map, battery, config):
    # S0 layout with full batteries: R1 heads right to the charger cell, R2 heads left to room A.
    robots = [Robot("R1", (3, 2), battery.max, ((3, 10),)),
              Robot("R2", (3, 8), battery.max, ((0, 1),))]
    paths = run(challenge_map, robots, battery, config, priority="fixed")
    assert_solution(challenge_map, robots, paths)
    # R1 has priority and drives straight through.
    assert paths["R1"] == [(3, c) for c in range(2, 11)]
    # R2 must leave the corridor between the two robots to let R1 pass,
    # e.g. into the side pocket (4,6) or the door (2,6).
    assert any(r != CORRIDOR_ROW and 3 <= c <= 8 for r, c in paths["R2"])
    # 10 steps unobstructed; giving way costs exactly 4 more (out, wait for R1 to be
    # fully past, back in).
    assert len(paths["R2"]) - 1 == 14


def test_reverse_priority_is_also_collision_free(challenge_map, battery, config):
    robots = [Robot("R2", (3, 8), battery.max, ((0, 1),)),
              Robot("R1", (3, 2), battery.max, ((3, 10),))]
    paths = run(challenge_map, robots, battery, config, priority="fixed")
    assert_solution(challenge_map, robots, paths)
    assert len(paths["R2"]) - 1 == 10  # now R2 goes straight


def test_parked_robot_is_not_walked_over(battery, config):
    grid = GridMap(["...", "...", "..."])
    # R1 parks in the center at t=1. R2's straight route crosses the center.
    robots = [Robot("R1", (1, 0), battery.max, ((1, 1),)),
              Robot("R2", (1, 2), battery.max, ((1, 0),))]
    paths = run(grid, robots, battery, config, priority="fixed")
    assert_solution(grid, robots, paths)
    assert (1, 1) not in paths["R2"][1:]
    assert len(paths["R2"]) - 1 == 4  # around the parked robot


def test_parked_robot_blocking_corridor(challenge_map, battery, config):
    # R1 parks in the single corridor at (3,5); R2 must get from (3,8) to (3,0).
    robots = [Robot("R1", (3, 4), battery.max, ((3, 5),)),
              Robot("R2", (3, 8), battery.max, ((3, 0),))]
    # Fixed order: R1 blocks the corridor forever, so R2 has no path.
    assert run(challenge_map, robots, battery, config, priority="fixed") is None
    # R2 first: R1 hides in door (2,1) while R2 passes, then returns. Restarts find that order.
    paths = run(challenge_map, robots, battery, config, priority="random_restart")
    assert_solution(challenge_map, robots, paths)
