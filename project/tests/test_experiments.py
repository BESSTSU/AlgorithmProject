"""M6: batch runner, summaries, and charts."""
import copy
import csv
import random

import pytest

from core.st_astar import plan_path
from experiments.metrics import summarize
from experiments.plots import make_plots, make_scaling_plots
from experiments.run_batch import (FIELDS, feasible_scenarios, run_main_experiment,
                                   run_scaling_experiment, variants)
from tests.conftest import ROOT


@pytest.fixture(scope="module")
def small_config(config):
    """The real config with small experiments, so each batch runs in about a second."""
    small = copy.deepcopy(config)
    small["experiments"].update(n_scenarios=6, delay_runs=3, priorities=["fixed", "battery_urgency"],
                                baseline_thresholds=[0.3, 1.0], reserves=[0, 5])
    small["experiments"]["scaling"].update(n_robots=[2, 3], priority="fixed")
    return small


@pytest.fixture(scope="module")
def batch(small_config):
    rows, _ = run_main_experiment(small_config, ROOT)
    return rows


@pytest.fixture(scope="module")
def scaling_batch(small_config):
    rows, _ = run_scaling_experiment(small_config, ROOT)
    return rows


def test_variants_cover_every_combination(small_config):
    all_variants = variants(small_config, ["fixed", "battery_urgency"], [0, 5])
    assert len(all_variants) == len(set(all_variants)) == 2 * 2 * (1 + 2)
    assert {v.method for v in all_variants} == {"main", "baseline"}
    assert all((v.threshold is None) == (v.method == "main") for v in all_variants)


def test_scenarios_are_feasible_for_each_robot_alone(challenge_map, battery, small_config):
    rng = random.Random(small_config["experiments"]["seed"])
    scenarios, draws = feasible_scenarios(challenge_map, battery, small_config, rng, n_robots=2)
    assert len(scenarios) == 6 and draws >= 6
    t_max = small_config["planner"]["t_max"]
    for robots in scenarios:
        assert len(robots) == 2
        for robot in robots:
            assert plan_path(challenge_map, robot.start, robot.tasks, robot.battery, battery, t_max)


def test_every_method_runs_on_the_same_scenarios(batch):
    n_variants = 2 * 2 * (1 + 2)
    assert len(batch) == 6 * n_variants
    assert all(list(row) == FIELDS for row in batch)
    by_scenario = {}
    for row in batch:
        by_scenario.setdefault(row["scenario"], []).append(row)
    assert sorted(by_scenario) == list(range(6))
    assert all(len(rows) == n_variants for rows in by_scenario.values())


def test_rows_are_consistent(batch, scaling_batch):
    for row in batch + scaling_batch:
        if row["success"]:
            assert row["failure"] == "" and row["makespan"] > 0
            assert row["sum_of_costs"] >= row["makespan"]
            assert row["collisions"] == 0  # execution with delays never collides
            assert row["real_makespan_mean"] >= row["makespan"]
            assert 0 <= row["battery_violation_rate"] <= 1
        else:
            assert row["failure"] in ("battery", "no_path", "horizon")
            assert row["failed_robot"] and row["makespan"] == ""


def test_batch_is_reproducible(small_config, batch):
    again, _ = run_main_experiment(small_config, ROOT)
    timing = "planner_ms"  # wall-clock time, the only field allowed to differ
    assert [{k: v for k, v in row.items() if k != timing} for row in again] == \
           [{k: v for k, v in row.items() if k != timing} for row in batch]


def test_scaling_experiment_grows_the_robot_count(small_config, scaling_batch):
    scaling = small_config["experiments"]["scaling"]
    n_variants = 1 + 2  # one priority and one reserve: main plus two thresholds
    assert len(scaling_batch) == 2 * 6 * n_variants
    assert {row["n_robots"] for row in scaling_batch} == {2, 3}
    assert {row["map"] for row in scaling_batch} == {scaling["map"]}
    assert {row["priority"] for row in scaling_batch} == {"fixed"}
    summaries = summarize(scaling_batch)
    assert [(s["n_robots"], s["method"]) for s in summaries] == [
        ("2", "main"), ("2", "baseline"), ("2", "baseline"),
        ("3", "main"), ("3", "baseline"), ("3", "baseline"),
    ]


def test_summary_from_csv_matches_summary_from_rows(batch, tmp_path):
    path = tmp_path / "results.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(batch)
    with path.open(newline="", encoding="utf-8") as f:
        from_csv = summarize(list(csv.DictReader(f)))
    assert from_csv == summarize(batch)


def test_summary_counts():
    def row(scenario, method, success, makespan="", failure=""):
        return {"n_robots": 2, "scenario": scenario, "method": method,
                "threshold": "" if method == "main" else 0.3,
                "priority": "fixed", "reserve": 0, "success": success, "failure": failure,
                "makespan": makespan, "sum_of_costs": makespan, "wait_steps": 0, "charge_steps": 0,
                "planner_ms": 1.0, "delay_runs": 2 if success else "",
                "real_makespan_mean": makespan * 1.5 if success else "",
                "collisions": 0 if success else "",
                "battery_violation_rate": 0.5 if success else ""}

    rows = [row(0, "main", 1, 10), row(1, "main", 1, 20), row(2, "main", 0, failure="no_path"),
            row(0, "baseline", 1, 15), row(1, "baseline", 0, failure="battery"),
            row(2, "baseline", 0, failure="battery")]
    main, baseline = summarize(rows)
    assert (main["solved"], main["runs"], main["fail_no_path"]) == (2, 3, 1)
    assert main["makespan_mean"] == 15 and main["makespan_vs_main"] == 1
    assert (baseline["solved"], baseline["fail_battery"]) == (1, 2)
    # Compared only on scenario 0, the one both solved: 15 against 10.
    assert (baseline["makespan_vs_main"], baseline["compared_with_main"]) == (1.5, 1)
    assert baseline["delay_slowdown"] == 1.5 and baseline["battery_violation_rate"] == 0.5


def test_plots_are_written(batch, scaling_batch, small_config, tmp_path):
    files = make_plots(batch, small_config, tmp_path) + make_scaling_plots(scaling_batch, tmp_path)
    assert {f.name for f in files} == {
        "summary.csv", "success_by_priority.png", "outcomes.png", "makespan.png",
        "battery_violations.png", "scaling_summary.csv", "scaling_solved.png", "scaling_makespan.png",
    }
    assert all(f.stat().st_size > 0 for f in files)
