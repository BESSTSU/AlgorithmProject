"""Batch experiments: every method on the same seeded scenario set, one CSV row per run.

    python -m experiments.run_batch              # main map, results/results.csv
    python -m experiments.run_batch --scaling    # second map, 2..n robots, results/scaling.csv
    python -m experiments.run_batch --map maps/challenge_11x5.txt --out results/challenge_11x5/results.csv

Methods are the main planner and the rule-based baseline at each configured
threshold. The main experiment plans each with every configured priority
strategy and battery reserve; the scaling experiment varies the number of robots
instead. A successful plan is also executed in the grid simulator with random delays.
"""
import argparse
import csv
import random
import time
from dataclasses import dataclass, replace
from pathlib import Path

import yaml

from core.baseline import plan_baseline
from core.battery import BatteryModel
from core.grid_map import GridMap
from core.prioritized import plan_prioritized
from core.scenario import Robot, random_scenario
from core.st_astar import plan_path
from execution.executor import execute
from experiments.metrics import PLAN_METRICS, plan_metrics, summarize
from robots.grid_sim import GridSim

FIELDS = ["map", "n_robots", "scenario", "method", "threshold", "priority", "reserve", "success",
          "failure", "failed_robot", *PLAN_METRICS, "planner_ms", "delay_runs",
          "real_makespan_mean", "collisions", "battery_violation_rate", "lowest_battery"]


@dataclass(frozen=True)
class Variant:
    method: str              # "main" or "baseline"
    threshold: float | None  # baseline charging threshold
    priority: str            # prioritized-planning strategy
    reserve: int             # battery.reserve used for planning


def variants(config: dict, priorities: list[str], reserves: list[int]) -> list[Variant]:
    """The main method and every baseline threshold, for each priority and reserve."""
    thresholds = config["experiments"]["baseline_thresholds"]
    return [
        Variant(method, threshold, priority, reserve)
        for priority in priorities
        for reserve in reserves
        for method, threshold in [("main", None)] + [("baseline", t) for t in thresholds]
    ]


def feasible_scenarios(grid: GridMap, battery: BatteryModel, config: dict, rng: random.Random,
                       n_robots: int) -> tuple[list[list[Robot]], int]:
    """Random scenarios in which every robot can finish its own tasks when alone on the map.

    Returns the scenarios and the number of draws it took. Scenarios that are
    impossible from the start are dropped, so remaining failures come from the
    planning method or from robots getting in each other's way.
    """
    gen, t_max = config["scenario_gen"], config["planner"]["t_max"]
    scenarios, draws = [], 0
    while len(scenarios) < config["experiments"]["n_scenarios"]:
        robots = random_scenario(grid, rng, n_robots, gen["battery_min"], battery.max,
                                 gen["tasks_min"], gen["tasks_max"])
        draws += 1
        if all(plan_path(grid, r.start, r.tasks, r.battery, battery, t_max) for r in robots):
            scenarios.append(robots)
    return scenarios, draws


def run_variant(grid: GridMap, robots: list[Robot], battery: BatteryModel, config: dict,
                variant: Variant, scenario: int, map_name: str) -> dict:
    """Plan one scenario with one variant, execute it with delays, and return its CSV row."""
    planner, experiments, delay = config["planner"], config["experiments"], config["sim_delay"]
    model = replace(battery, reserve=variant.reserve)
    common = (grid, robots, model, planner["t_max"], variant.priority, planner["max_restarts"],
              random.Random(experiments["seed"]))
    started = time.perf_counter()
    if variant.method == "main":
        result = plan_prioritized(*common)
    else:
        result = plan_baseline(*common, variant.threshold)
    planner_ms = (time.perf_counter() - started) * 1000

    row = dict.fromkeys(FIELDS, "")
    row.update(map=map_name, n_robots=len(robots), scenario=scenario, method=variant.method,
               threshold="" if variant.threshold is None else variant.threshold,
               priority=variant.priority, reserve=variant.reserve,
               success=int(result.paths is not None), failure=result.failure or "",
               failed_robot=result.failed_robot or "", planner_ms=round(planner_ms, 2))
    if result.paths is None:
        return row
    row.update(plan_metrics(grid, result.paths))

    runs = experiments["delay_runs"]
    rng = random.Random(experiments["seed"] + scenario)
    makespans, collisions, violations, lowest = [], 0, 0, model.max
    for _ in range(runs):
        sim = GridSim(grid, {r.name: r.start for r in robots}, delay["prob"],
                      delay["max_extra_steps"], rng)
        executed = execute(grid, result.paths, sim.robots, {r.name: r.battery for r in robots},
                           model, sim.step)
        makespans.append(executed.makespan)
        collisions += sim.collisions
        violations += executed.lowest_battery < 0
        lowest = min(lowest, executed.lowest_battery)
    if runs:
        row.update(delay_runs=runs, real_makespan_mean=round(sum(makespans) / runs, 2),
                   collisions=collisions, battery_violation_rate=round(violations / runs, 3),
                   lowest_battery=lowest)
    return row


def run_batch(grid: GridMap, battery: BatteryModel, config: dict, n_robots: int,
              variant_list: list[Variant], map_name: str = "") -> tuple[list[dict], int]:
    """Every variant on the same seeded scenario set. Returns (rows, scenario draws)."""
    rng = random.Random(config["experiments"]["seed"])
    scenarios, draws = feasible_scenarios(grid, battery, config, rng, n_robots)
    rows = [
        run_variant(grid, robots, battery, config, variant, index, map_name)
        for index, robots in enumerate(scenarios)
        for variant in variant_list
    ]
    return rows, draws


def run_main_experiment(config: dict, root: Path,
                        map_file: str | None = None) -> tuple[list[dict], int]:
    """All priority strategies and battery reserves, on the main map unless another is given."""
    experiments = config["experiments"]
    map_file = map_file or config["map"]
    grid = GridMap.from_file(root / map_file)
    battery = BatteryModel(**config["battery"])
    variant_list = variants(config, experiments["priorities"], experiments["reserves"])
    return run_batch(grid, battery, config, config["scenario_gen"]["n_robots"], variant_list,
                     map_file)


def run_scaling_experiment(config: dict, root: Path) -> tuple[list[dict], int]:
    """Second map: growing number of robots, one priority strategy, the default reserve."""
    scaling = config["experiments"]["scaling"]
    grid = GridMap.from_file(root / scaling["map"])
    battery = BatteryModel(**config["battery"])
    variant_list = variants(config, [scaling["priority"]], [battery.reserve])
    rows, draws = [], 0
    for n_robots in scaling["n_robots"]:
        more_rows, more_draws = run_batch(grid, battery, config, n_robots, variant_list,
                                          scaling["map"])
        rows += more_rows
        draws += more_draws
    return rows, draws


def main():
    parser = argparse.ArgumentParser(description="Run all methods on a seeded scenario set.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scaling", action="store_true",
                        help="run the robot-count experiment on the second map")
    parser.add_argument("--map", help="map file for the main experiment (default: the config's map)")
    parser.add_argument("--out", help="CSV file (default results/results.csv or results/scaling.csv)")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if args.scaling:
        rows, draws = run_scaling_experiment(config, config_path.parent)
    else:
        rows, draws = run_main_experiment(config, config_path.parent, args.map)

    out = Path(args.out or ("results/scaling.csv" if args.scaling else "results/results.csv"))
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print(f"{len(rows)} rows from {draws} scenario draws -> {out}")
    header = ["robots", "method", "thr", "priority", "reserve", "solved", "battery", "no_path",
              "horizon", "makespan", "x main", "delay x", "bat<0", "collisions", "plan ms"]
    widths = [6, 9, 5, 16, 8, 8, 8, 8, 8, 9, 7, 8, 6, 11, 8]
    print("".join(f"{h:>{w}}" for h, w in zip(header, widths)))
    for s in summarize(rows):
        cells = [s["n_robots"], s["method"], s["threshold"], s["priority"], s["reserve"],
                 f"{s['solved']}/{s['runs']}", s["fail_battery"], s["fail_no_path"],
                 s["fail_horizon"], s["makespan_mean"], s["makespan_vs_main"], s["delay_slowdown"],
                 s["battery_violation_rate"], s["collisions"], s["planner_ms_mean"]]
        print("".join(f"{c:>{w}}" for c, w in zip(cells, widths)))


if __name__ == "__main__":
    main()
