"""M5: Action Dependency Graph, executor, and the grid simulator with delays."""
import random

import pytest

from core.baseline import plan_baseline
from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot, random_scenario
from core.st_astar import plan_path
from execution.adg import ActionDependencyGraph
from execution.executor import execute
from robots.grid_sim import GridSim

NO_DELAY = (0.0, 0)


def plan(grid, robots, battery, config, rng=None):
    planner = config["planner"]
    rng = rng or random.Random(config["experiments"]["seed"])
    return plan_prioritized(grid, robots, battery, planner["t_max"], planner["priority"],
                            planner["max_restarts"], rng).paths


def run(grid, robots, paths, battery, delay, rng):
    """Execute `paths` in a fresh GridSim; return (sim, execution result)."""
    sim = GridSim(grid, {r.name: r.start for r in robots}, *delay, rng)
    result = execute(grid, paths, sim.robots, {r.name: r.battery for r in robots}, battery, sim.step)
    return sim, result


def all_at_goal(sim, robots):
    return all(sim.robots[r.name].get_cell() == r.tasks[-1] for r in robots)


# --- ADG ----------------------------------------------------------------------

def test_visits_drop_waits():
    adg = ActionDependencyGraph({"A": [(0, 0), (0, 0), (0, 1), (0, 1), (0, 1), (0, 2)]})
    assert [(v.cell, v.t_in, v.t_out) for v in adg.visits["A"]] == [
        ((0, 0), 0, 1), ((0, 1), 2, 4), ((0, 2), 5, 5),
    ]
    assert adg.n_moves("A") == 2


def test_follower_depends_on_the_leader_leaving():
    adg = ActionDependencyGraph({"lead": [(0, 1), (0, 2)], "follow": [(0, 0), (0, 1)]})
    assert adg.dependencies("lead", 0) == []
    assert adg.dependencies("follow", 0) == [("lead", 0)]
    assert not adg.can_start("follow", 0, {"lead": 0, "follow": 0})
    assert adg.can_start("follow", 0, {"lead": 1, "follow": 0})


def test_s0_dependencies(challenge_map, battery, config, s0):
    adg = ActionDependencyGraph(plan(challenge_map, s0, battery, config))
    # R2 passes (3,6) first, on its way into the door: R1 enters (3,6) only after R2 left it.
    assert adg.dependencies("R1", 3) == [("R2", 2)]
    # R2 comes back out of the door into (3,6) only after R1 has moved on from it.
    assert adg.dependencies("R2", 3) == [("R1", 4)]


def test_plan_entering_a_parked_robots_cell_is_rejected():
    with pytest.raises(ValueError):
        ActionDependencyGraph({"parked": [(0, 0)], "late": [(0, 1), (0, 0)]})


# --- grid simulator -----------------------------------------------------------

def test_move_takes_one_tick_without_delay(challenge_map):
    sim = GridSim(challenge_map, {"A": (3, 0)}, *NO_DELAY, random.Random(0))
    robot = sim.robots["A"]
    robot.go_to_cell((3, 1))
    assert not robot.is_arrived() and robot.get_cell() == (3, 0)
    sim.step()
    assert robot.is_arrived() and robot.get_cell() == (3, 1)


def test_delayed_move_takes_longer(challenge_map, config):
    max_extra = config["sim_delay"]["max_extra_steps"]
    sim = GridSim(challenge_map, {"A": (3, 0)}, 1.0, max_extra, random.Random(0))
    robot = sim.robots["A"]
    durations = set()
    for target in [(3, 1), (3, 0)] * 20:
        robot.go_to_cell(target)
        ticks = 0
        while not robot.is_arrived():
            sim.step()
            ticks += 1
        durations.add(ticks)
    assert durations == set(range(2, 2 + max_extra))


def test_collision_detection_is_strict(challenge_map):
    def collisions(starts, targets):
        sim = GridSim(challenge_map, starts, *NO_DELAY, random.Random(0))
        for name, target in targets.items():
            sim.robots[name].go_to_cell(target)
        sim.step()
        return sim.collisions

    assert collisions({"A": (3, 0), "B": (3, 2)}, {"A": (3, 1), "B": (3, 1)}) > 0  # same cell
    assert collisions({"A": (3, 0), "B": (3, 1)}, {"A": (3, 1), "B": (3, 0)}) > 0  # swap
    assert collisions({"A": (3, 0), "B": (3, 1)}, {"A": (3, 1), "B": (3, 2)}) > 0  # tailgating
    assert collisions({"A": (3, 0), "B": (3, 2)}, {"A": (3, 1), "B": (3, 3)}) == 0


def test_invalid_move_rejected(challenge_map):
    sim = GridSim(challenge_map, {"A": (3, 0)}, *NO_DELAY, random.Random(0))
    with pytest.raises(ValueError):
        sim.robots["A"].go_to_cell((2, 0))  # wall
    with pytest.raises(ValueError):
        sim.robots["A"].go_to_cell((3, 2))  # not adjacent


# --- executor -----------------------------------------------------------------

def test_s0_without_delay(challenge_map, battery, config, s0):
    paths = plan(challenge_map, s0, battery, config)
    sim, result = run(challenge_map, s0, paths, battery, NO_DELAY, random.Random(0))
    assert sim.collisions == 0 and all_at_goal(sim, s0)
    # Without delays the execution reproduces the plan, tick for tick.
    assert result.finish_ticks == {"R1": 19, "R2": 14}
    for robot in s0:
        planned = battery.trace(challenge_map, paths[robot.name], robot.battery)
        assert result.battery[robot.name] == planned


def test_s0_with_delays_never_collides(challenge_map, battery, config, s0):
    delay = (config["sim_delay"]["prob"], config["sim_delay"]["max_extra_steps"])
    paths = plan(challenge_map, s0, battery, config)
    rng = random.Random(config["experiments"]["seed"])
    for _ in range(200):
        sim, result = run(challenge_map, s0, paths, battery, delay, rng)
        assert sim.collisions == 0 and all_at_goal(sim, s0)
        assert result.makespan >= 19


def test_charges_longer_to_make_up_for_delays(challenge_map, battery, config):
    # Plan: reach the charger with 6, charge 4 steps to 26, then 7 moves to the task.
    robots = [Robot("R1", (3, 8), 10, ((0, 6),))]
    paths = plan(challenge_map, robots, battery, config)
    planned = battery.trace(challenge_map, paths["R1"], 10)
    assert max(planned) == 26
    # Every move is delayed, so the robot arrives at the charger with less than planned.
    always_delayed = (1.0, config["sim_delay"]["max_extra_steps"])
    sim, result = run(challenge_map, robots, paths, battery, always_delayed, random.Random(0))
    real = result.battery["R1"]
    charge_ticks = sum(1 for a, b in zip(real, real[1:]) if b > a)
    assert all_at_goal(sim, robots)
    assert max(real) >= 26 and charge_ticks > 4


def test_baseline_plan_executes_too(challenge_map, battery, config, s1):
    planner = config["planner"]
    rng = random.Random(config["experiments"]["seed"])
    paths = plan_baseline(challenge_map, s1, battery, planner["t_max"], planner["priority"],
                          planner["max_restarts"], rng, config["baseline"]["threshold"]).paths
    delay = (config["sim_delay"]["prob"], config["sim_delay"]["max_extra_steps"])
    for _ in range(50):
        sim, _ = run(challenge_map, s1, paths, battery, delay, rng)
        assert sim.collisions == 0 and all_at_goal(sim, s1)


def test_deadlock_is_reported(battery):
    # Four robots rotating around a 2x2 block: nobody can move first under the
    # executor's rule. The planner's clearance rule never produces such a plan,
    # so these paths are written by hand.
    grid = GridMap(["..", ".."])
    ring = [(0, 0), (0, 1), (1, 1), (1, 0)]
    paths = {f"R{i}": [ring[i], ring[(i + 1) % 4]] for i in range(4)}
    sim = GridSim(grid, {name: path[0] for name, path in paths.items()}, *NO_DELAY, random.Random(0))
    with pytest.raises(RuntimeError, match="deadlock"):
        execute(grid, paths, sim.robots, {name: battery.max for name in paths}, battery, sim.step)


# --- random scenarios -----------------------------------------------------------

def generate(grid, battery, config, rng):
    gen = config["scenario_gen"]
    return random_scenario(grid, rng, gen["n_robots"], gen["battery_min"], battery.max,
                           gen["tasks_min"], gen["tasks_max"])


@pytest.fixture(scope="module")
def solved_scenarios(challenge_map, battery, config):
    """100 random scenarios together with their plans, shared by the tests below."""
    rng = random.Random(config["experiments"]["seed"])
    t_max = config["planner"]["t_max"]
    found = []
    while len(found) < 100:
        robots = generate(challenge_map, battery, config, rng)
        if not all(plan_path(challenge_map, r.start, r.tasks, r.battery, battery, t_max)
                   for r in robots):
            continue  # impossible even for a robot on its own: skip the costly joint attempt
        paths = plan(challenge_map, robots, battery, config, rng)
        if paths is not None:
            found.append((robots, paths))
    return found


def test_random_scenario_is_valid_and_reproducible(challenge_map, battery, config):
    gen = config["scenario_gen"]
    rooms = set(challenge_map.room_cells())
    for seed in range(50):
        robots = generate(challenge_map, battery, config, random.Random(seed))
        assert robots == generate(challenge_map, battery, config, random.Random(seed))
        assert len(robots) == gen["n_robots"]
        assert len({r.start for r in robots}) == len(robots)
        assert len({r.tasks[-1] for r in robots}) == len(robots)
        for robot in robots:
            assert challenge_map.is_passable(robot.start)
            assert gen["battery_min"] <= robot.battery <= battery.max
            assert gen["tasks_min"] <= len(robot.tasks) <= gen["tasks_max"]
            assert set(robot.tasks) <= rooms and len(set(robot.tasks)) == len(robot.tasks)


def test_execution_without_delay_matches_the_plan(challenge_map, battery, solved_scenarios):
    # The planner's clearance rule is the executor's rule, so with no delays every
    # robot finishes exactly when planned and no battery goes below zero.
    for robots, paths in solved_scenarios:
        sim, result = run(challenge_map, robots, paths, battery, NO_DELAY, random.Random(0))
        assert sim.collisions == 0 and all_at_goal(sim, robots)
        assert result.finish_ticks == {name: len(path) - 1 for name, path in paths.items()}, robots
        assert result.lowest_battery >= 0, robots


def test_time_parked_on_a_charger_is_kept(challenge_map, battery, config):
    # R2 charges first and then has to wait for R1. The plan parks that wait on the
    # charger, where it is free. Leaving as soon as the battery is full would move
    # the wait into the corridor and run R2's battery below zero.
    robots = [Robot("R1", (3, 3), 31, ((1, 1), (0, 5))),
              Robot("R2", (3, 9), 37, ((1, 6), (0, 1), (0, 0)))]
    paths = plan(challenge_map, robots, battery, config)
    planned = battery.trace(challenge_map, paths["R2"], 37)
    assert planned[1:6] == [35, 40, 40, 40, 40]  # one charge step, then three free waits
    sim, result = run(challenge_map, robots, paths, battery, NO_DELAY, random.Random(0))
    assert result.finish_ticks == {name: len(path) - 1 for name, path in paths.items()}
    assert result.battery["R2"] == planned


def test_1000_random_rounds_with_delays(challenge_map, battery, config, solved_scenarios):
    """SPEC M5: 1,000 randomized executions, no collision, every robot reaches its goal.

    100 random solvable scenarios, each executed 10 times with different random delays.
    """
    delay = (config["sim_delay"]["prob"], config["sim_delay"]["max_extra_steps"])
    rng = random.Random(config["experiments"]["seed"])
    rounds = 0
    for robots, paths in solved_scenarios:
        for _ in range(10):
            sim, _ = run(challenge_map, robots, paths, battery, delay, rng)
            assert sim.collisions == 0, f"collision in {robots}"
            assert all_at_goal(sim, robots), f"goal missed in {robots}"
            rounds += 1
    assert rounds == 1000
