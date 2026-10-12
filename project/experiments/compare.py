"""Compare the main method and the rule-based baseline on one scenario.

    python -m experiments.compare --scenario scenarios/s0.yaml
"""
import argparse
import random
from pathlib import Path

import yaml

from core.baseline import plan_baseline
from core.battery import BatteryModel
from core.grid_map import GridMap
from core.prioritized import PlanResult, plan_prioritized
from core.scenario import Robot, load_scenario
from experiments.metrics import plan_metrics


def compare(grid: GridMap, robots: list[Robot], battery: BatteryModel,
            config: dict) -> dict[str, PlanResult]:
    """Plan the same scenario with both methods under identical planner settings."""
    planner = config["planner"]
    seed = config["experiments"]["seed"]
    common = (grid, robots, battery, planner["t_max"], planner["priority"], planner["max_restarts"])
    return {
        "main": plan_prioritized(*common, random.Random(seed)),
        "baseline": plan_baseline(*common, random.Random(seed), config["baseline"]["threshold"]),
    }


def main():
    parser = argparse.ArgumentParser(description="Compare main method and baseline on one scenario.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/s0.yaml")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = GridMap.from_file(config_path.parent / config["map"])
    battery = BatteryModel(**config["battery"])
    robots = load_scenario(args.scenario)
    results = compare(grid, robots, battery, config)

    columns = ["makespan", "sum_of_costs", "wait_steps", "charge_steps", "charge_visits"]
    print(f"scenario: {args.scenario}   priority: {config['planner']['priority']}")
    print(f"{'method':<10}{'result':<24}" + "".join(f"{c:>15}" for c in columns))
    for method, result in results.items():
        if result.paths is None:
            outcome = f"failed: {result.failure} ({result.failed_robot})"
            values = ["-"] * len(columns)
        else:
            outcome = "success"
            metrics = plan_metrics(grid, result.paths)
            values = [metrics[c] for c in columns]
        print(f"{method:<10}{outcome:<24}" + "".join(f"{v:>15}" for v in values))
    for method, result in results.items():
        if result.paths is None:
            continue
        print(f"\n{method}:")
        for robot in robots:
            path = result.paths[robot.name]
            levels = battery.trace(grid, path, robot.battery)
            print(f"  {robot.name}: {len(path) - 1} steps, battery {levels[0]} -> {levels[-1]}"
                  f" (lowest {min(levels)})")


if __name__ == "__main__":
    main()
