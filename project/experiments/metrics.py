"""Metrics of a finished plan, and summaries of batch results (SPEC section 8)."""
from core.grid_map import CHARGER, Cell, GridMap

PLAN_METRICS = ["makespan", "sum_of_costs", "wait_steps", "charge_steps", "charge_visits"]


def plan_metrics(grid: GridMap, paths: dict[str, list[Cell]]) -> dict[str, int]:
    """Makespan, sum of costs, and wait / charge counts summed over all robots.

    A wait on a charger cell counts as a charge step, not a wait step. A charge
    visit is one unbroken run of charge steps.
    """
    wait_steps = charge_steps = charge_visits = 0
    for path in paths.values():
        was_charging = False
        for a, b in zip(path, path[1:]):
            charging = a == b and grid.symbol(a) == CHARGER
            if charging:
                charge_steps += 1
                charge_visits += not was_charging
            elif a == b:
                wait_steps += 1
            was_charging = charging
    finish_times = [len(path) - 1 for path in paths.values()]
    return {
        "makespan": max(finish_times),
        "sum_of_costs": sum(finish_times),
        "wait_steps": wait_steps,
        "charge_steps": charge_steps,
        "charge_visits": charge_visits,
    }


def _mean(values: list[float]) -> float | str:
    return round(sum(values) / len(values), 2) if values else ""


def summarize(rows: list[dict]) -> list[dict]:
    """One summary per variant (robots, method, threshold, priority, reserve), in first-seen order.

    Works on rows as produced by run_batch or as read back from its CSV.
    `makespan_vs_main` is the mean ratio to the main method with the same number
    of robots, priority and reserve, over the scenarios both solved, so methods
    that solve different scenario subsets are still compared like for like.
    """
    def key(row):
        return (str(row["n_robots"]), row["method"], str(row["threshold"]), row["priority"],
                str(row["reserve"]))

    def scenario_key(row):
        return str(row["n_robots"]), row["priority"], str(row["reserve"]), str(row["scenario"])

    groups: dict[tuple, list[dict]] = {}
    for row in rows:
        groups.setdefault(key(row), []).append(row)
    main_makespan = {
        scenario_key(row): float(row["makespan"])
        for row in rows if row["method"] == "main" and int(row["success"])
    }

    summaries = []
    for (n_robots, method, threshold, priority, reserve), group in groups.items():
        solved = [row for row in group if int(row["success"])]
        delayed = [row for row in solved if row["delay_runs"] != ""]
        ratios = [float(row["makespan"]) / main_makespan[scenario_key(row)]
                  for row in solved if scenario_key(row) in main_makespan]
        summaries.append({
            "n_robots": n_robots,
            "method": method,
            "threshold": threshold,
            "priority": priority,
            "reserve": reserve,
            "runs": len(group),
            "solved": len(solved),
            "success_rate": round(len(solved) / len(group), 3),
            "fail_battery": sum(row["failure"] == "battery" for row in group),
            "fail_no_path": sum(row["failure"] == "no_path" for row in group),
            "fail_horizon": sum(row["failure"] == "horizon" for row in group),
            "makespan_mean": _mean([float(row["makespan"]) for row in solved]),
            "sum_of_costs_mean": _mean([float(row["sum_of_costs"]) for row in solved]),
            "wait_steps_mean": _mean([float(row["wait_steps"]) for row in solved]),
            "charge_steps_mean": _mean([float(row["charge_steps"]) for row in solved]),
            "makespan_vs_main": _mean(ratios),
            "compared_with_main": len(ratios),
            "delay_slowdown": _mean([float(row["real_makespan_mean"]) / float(row["makespan"])
                                     for row in delayed if float(row["makespan"])]),
            "battery_violation_rate": _mean([float(row["battery_violation_rate"]) for row in delayed]),
            "collisions": sum(int(row["collisions"]) for row in delayed),
            "planner_ms_mean": _mean([float(row["planner_ms"]) for row in group]),
        })
    return summaries
