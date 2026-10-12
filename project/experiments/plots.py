"""Charts and a summary table from the batch results.

    python -m experiments.plots              # from results/results.csv
    python -m experiments.plots --scaling    # from results/scaling.csv

Writes PNG charts and a summary CSV next to the results file. The summary holds
every number the charts show.
"""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import yaml
from matplotlib.patches import Patch, PathPatch
from matplotlib.path import Path as MplPath

from experiments.metrics import summarize

# Chart chrome and palette (light surface). Series colors are used in this fixed order;
# in every chart the main method is the first color and each baseline keeps its own.
SURFACE, INK, INK_2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
LOW_HIGH = ["#86b6ef", "#2a78d6"]  # two steps of one hue for an ordered pair
DPI = 160
BAR_PX, GAP_PX, RADIUS_PX = 22 * DPI / 100, 2 * DPI / 100, 4 * DPI / 100

PRIORITY_LABELS = {"fixed": "Fixed order", "battery_urgency": "Lowest battery first",
                   "random_restart": "Random restarts"}
OUTCOMES = [("solved", "Solved"), ("fail_battery", "Battery ran short"),
            ("fail_no_path", "No path"), ("fail_horizon", "Over the time limit")]


def method_label(summary: dict) -> str:
    if summary["method"] == "main":
        return "Planned charging"
    return f"Rule: below {float(summary['threshold']):.0%}"


def _figure(title: str, subtitle: str, height: float = 4.2):
    plt.rcParams["font.family"] = ["Segoe UI", "DejaVu Sans"]
    fig, ax = plt.subplots(figsize=(7.6, height), dpi=DPI, facecolor=SURFACE)
    fig.subplots_adjust(left=0.09, right=0.97, top=1 - 1.25 / height, bottom=0.55 / height)
    ax.set_facecolor(SURFACE)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(AXIS)
    ax.tick_params(colors=MUTED, length=0, labelsize=9)
    ax.set_axisbelow(True)
    fig.text(0.03, 1 - 0.32 / height, title, fontsize=13, fontweight="bold", color=INK)
    fig.text(0.03, 1 - 0.60 / height, subtitle, fontsize=9, color=INK_2)
    return fig, ax


def _legend(fig, labels: list[str], colors: list[str]) -> None:
    handles = [Patch(facecolor=color, edgecolor="none", label=label)
               for label, color in zip(labels, colors)]
    height = fig.get_figheight()
    legend = fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.025, 1 - 0.70 / height),
                        ncols=len(labels), frameon=False, fontsize=9, handlelength=1.0,
                        handleheight=1.0, columnspacing=1.4)
    for text in legend.get_texts():
        text.set_color(INK_2)


def _columns(ax, groups: list[str], series: list[list[float]], colors: list[str],
             y_max: float, label, y_format: str = "{:g}") -> None:
    """Grouped columns with rounded tops. `label(group, series)` says whether to print the value."""
    ax.set_xlim(-0.5, len(groups) - 0.5)
    ax.set_ylim(0, y_max)
    ax.set_xticks(range(len(groups)), groups)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.yaxis.set_major_formatter(lambda value, _: y_format.format(value))
    box = ax.get_window_extent()
    px_x, px_y = box.width / len(groups), box.height / y_max  # pixels per data unit
    width, step = BAR_PX / px_x, (BAR_PX + GAP_PX) / px_x
    rx = RADIUS_PX / px_x
    for i, values in enumerate(series):
        offset = (i - (len(series) - 1) / 2) * step
        for g, value in enumerate(values):
            x0, x1 = g + offset - width / 2, g + offset + width / 2
            if value > 0:
                ry = min(RADIUS_PX / px_y, value)
                path = MplPath(
                    [(x0, 0), (x0, value - ry), (x0, value), (x0 + rx, value), (x1 - rx, value),
                     (x1, value), (x1, value - ry), (x1, 0), (x0, 0)],
                    [MplPath.MOVETO, MplPath.LINETO, MplPath.CURVE3, MplPath.CURVE3, MplPath.LINETO,
                     MplPath.CURVE3, MplPath.CURVE3, MplPath.LINETO, MplPath.CLOSEPOLY])
                ax.add_patch(PathPatch(path, facecolor=colors[i], edgecolor="none"))
            if label(g, i):
                ax.text(g + offset, value + 5 / px_y, y_format.format(value), ha="center",
                        va="bottom", fontsize=8.5, color=INK_2)


def _pick(summaries: list[dict], **where) -> list[dict]:
    return [s for s in summaries if all(str(s[k]) == str(v) for k, v in where.items())]


def _write_summary(summaries: list[dict], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)


def _save(fig, out: Path) -> None:
    fig.savefig(out, dpi=DPI, facecolor=SURFACE)
    plt.close(fig)


# --- main experiment ----------------------------------------------------------

def plot_success_by_priority(summaries: list[dict], reserve, n: int, out: Path) -> None:
    priorities = list(dict.fromkeys(s["priority"] for s in summaries))
    by_priority = [_pick(summaries, priority=p, reserve=reserve) for p in priorities]
    groups = [method_label(s) for s in by_priority[0]]
    series = [[s["solved"] for s in rows] for rows in by_priority]
    fig, ax = _figure("Scenarios solved, by method and priority strategy",
                      f"Out of {n} random two-robot scenarios that each robot could finish alone. "
                      f"Battery reserve {reserve}.")
    _legend(fig, [PRIORITY_LABELS.get(p, p) for p in priorities], SERIES)
    # Label the best strategy of each method; the axis and the summary carry the rest.
    best = [max(range(len(series)), key=lambda i: series[i][g]) for g in range(len(groups))]
    _columns(ax, groups, series, SERIES, n, lambda g, i: i == best[g])
    _save(fig, out)


def plot_outcomes(summaries: list[dict], priority, reserve, n: int, out: Path) -> None:
    rows = _pick(summaries, priority=priority, reserve=reserve)
    shown = [(key, name) for key, name in OUTCOMES if any(s[key] for s in rows)]
    fig, ax = _figure("What happened to each scenario",
                      f"{n} scenarios per method. Priority: {PRIORITY_LABELS.get(priority, priority)}. "
                      f"Battery reserve {reserve}.", height=2.4 + 0.3 * len(rows))
    _legend(fig, [name for _, name in shown], SERIES)
    fig.subplots_adjust(left=0.2)
    ax.set_xlim(0, n)
    ax.set_ylim(len(rows) - 0.5, -0.5)
    ax.set_yticks(range(len(rows)), [method_label(s) for s in rows])
    ax.spines["bottom"].set_visible(False)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    box = ax.get_window_extent()
    thickness, px_x = BAR_PX / (box.height / len(rows)), box.width / n
    for y, summary in enumerate(rows):
        left = 0
        for (key, _), color in zip(shown, SERIES):
            count = summary[key]
            if not count:
                continue
            # A surface-colored edge leaves a gap between touching segments.
            ax.barh(y, count, left=left, height=thickness, color=color, edgecolor=SURFACE,
                    linewidth=GAP_PX * 72 / DPI)
            if count * px_x >= 26 * DPI / 100:  # print the count only where it fits
                ax.text(left + count / 2, y, str(count), ha="center", va="center", fontsize=8.5,
                        color="white" if color == SERIES[0] else INK)
            left += count
    _save(fig, out)


def plot_makespan(rows: list[dict], summaries: list[dict], priority, reserve, out: Path) -> None:
    """Mean makespan of the main method and each baseline over the scenarios both solved."""
    def chosen(row, method):
        return (row["method"] == method and row["priority"] == priority
                and str(row["reserve"]) == str(reserve) and int(row["success"]))

    main = {r["scenario"]: float(r["makespan"]) for r in rows if chosen(r, "main")}
    groups, main_means, base_means = [], [], []
    for summary in _pick(summaries, method="baseline", priority=priority, reserve=reserve):
        shared = [(main[r["scenario"]], float(r["makespan"])) for r in rows
                  if chosen(r, "baseline") and str(r["threshold"]) == summary["threshold"]
                  and r["scenario"] in main]
        if not shared:
            continue
        groups.append(f"{method_label(summary)}\n{len(shared)} shared scenarios")
        main_means.append(round(sum(a for a, _ in shared) / len(shared), 1))
        base_means.append(round(sum(b for _, b in shared) / len(shared), 1))
    fig, ax = _figure("Makespan where both methods succeed",
                      "Mean time steps until the last robot finishes, over the scenarios solved by "
                      f"both. Priority: {PRIORITY_LABELS.get(priority, priority)}.")
    _legend(fig, ["Planned charging", "Rule-based baseline"], SERIES)
    fig.subplots_adjust(bottom=0.75 / fig.get_figheight())
    top = max(main_means + base_means)
    _columns(ax, groups, [main_means, base_means], SERIES, top * 1.18, lambda g, i: True)
    _save(fig, out)


def plot_battery_violations(summaries: list[dict], priority, n_runs, out: Path) -> None:
    reserves = sorted({s["reserve"] for s in summaries}, key=float)[:2]
    by_reserve = [_pick(summaries, priority=priority, reserve=r) for r in reserves]
    solved = [" / ".join(str(rows[g]["solved"]) for rows in by_reserve)
              for g in range(len(by_reserve[0]))]
    groups = [f"{method_label(s)}\n{solved[g]} solved" for g, s in enumerate(by_reserve[0])]
    series = [[100 * (s["battery_violation_rate"] or 0) for s in rows] for rows in by_reserve]
    fig, ax = _figure("Delayed runs in which a battery dropped below zero",
                      f"Each solved plan executed {n_runs} times with random delays. "
                      f"Priority: {PRIORITY_LABELS.get(priority, priority)}.")
    _legend(fig, [f"Planned with reserve {r}" for r in reserves], LOW_HIGH)
    fig.subplots_adjust(bottom=0.75 / fig.get_figheight())
    top = max(max(values) for values in series)
    _columns(ax, groups, series, LOW_HIGH, max(10, top * 1.25), lambda g, i: True, "{:.0f}%")
    _save(fig, out)


def make_plots(rows: list[dict], config: dict, out_dir: Path) -> list[Path]:
    """Write summary.csv and the main experiment's charts into out_dir; return the files."""
    summaries = summarize(rows)
    priority, reserve = config["planner"]["priority"], config["battery"]["reserve"]
    n = len({row["scenario"] for row in rows})
    out_dir.mkdir(parents=True, exist_ok=True)
    files = [out_dir / name for name in ("summary.csv", "success_by_priority.png", "outcomes.png",
                                         "makespan.png", "battery_violations.png")]
    _write_summary(summaries, files[0])
    plot_success_by_priority(summaries, reserve, n, files[1])
    plot_outcomes(summaries, priority, reserve, n, files[2])
    plot_makespan(rows, summaries, priority, reserve, files[3])
    plot_battery_violations(summaries, priority, config["experiments"]["delay_runs"], files[4])
    return files


# --- scaling experiment -------------------------------------------------------

def plot_scaling_solved(summaries: list[dict], n: int, map_name: str, out: Path) -> None:
    counts = list(dict.fromkeys(s["n_robots"] for s in summaries))
    methods = _pick(summaries, n_robots=counts[0])
    series = [[_pick(summaries, n_robots=c, method=m["method"], threshold=m["threshold"])[0]["solved"]
               for c in counts] for m in methods]
    fig, ax = _figure("Scenarios solved as robots are added",
                      f"Out of {n} random scenarios per robot count on {map_name}. "
                      f"Priority: {PRIORITY_LABELS.get(methods[0]['priority'], methods[0]['priority'])}.")
    _legend(fig, [method_label(m) for m in methods], SERIES)
    _columns(ax, [f"{c} robots" for c in counts], series, SERIES, n, lambda g, i: i == 0)
    _save(fig, out)


def plot_scaling_makespan(summaries: list[dict], map_name: str, out: Path) -> None:
    counts = list(dict.fromkeys(s["n_robots"] for s in summaries))
    baselines = _pick(summaries, n_robots=counts[0], method="baseline")
    series = [[_pick(summaries, n_robots=c, method="baseline", threshold=b["threshold"])[0]
               ["makespan_vs_main"] or 0 for c in counts] for b in baselines]
    fig, ax = _figure("Baseline makespan relative to planned charging",
                      f"On the scenarios both solved ({map_name}). The line marks 1.00×, equal to "
                      "planned charging. No bar: none solved by both.")
    _legend(fig, [method_label(b) for b in baselines], SERIES[1:])
    top = max(max(values) for values in series)
    # Label only the strictest rule, the strongest baseline; the axis carries the others.
    strictest = len(series) - 1
    _columns(ax, [f"{c} robots" for c in counts], series, SERIES[1:], max(2.0, top * 1.2),
             lambda g, i: i == strictest and series[i][g] > 0, "{:.2f}×")
    ax.axhline(1.0, color=MUTED, linewidth=0.8)
    _save(fig, out)


def make_scaling_plots(rows: list[dict], out_dir: Path) -> list[Path]:
    """Write scaling_summary.csv and the scaling charts into out_dir; return the files."""
    summaries = summarize(rows)
    n = len({row["scenario"] for row in rows})
    map_name = Path(rows[0]["map"]).stem
    out_dir.mkdir(parents=True, exist_ok=True)
    files = [out_dir / name for name in ("scaling_summary.csv", "scaling_solved.png",
                                         "scaling_makespan.png")]
    _write_summary(summaries, files[0])
    plot_scaling_solved(summaries, n, map_name, files[1])
    plot_scaling_makespan(summaries, map_name, files[2])
    return files


def main():
    parser = argparse.ArgumentParser(description="Draw charts from the batch results.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scaling", action="store_true", help="draw the robot-count experiment")
    parser.add_argument("--results", help="CSV file (default results/results.csv or results/scaling.csv)")
    args = parser.parse_args()

    results = Path(args.results or ("results/scaling.csv" if args.scaling else "results/results.csv"))
    with results.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if args.scaling:
        files = make_scaling_plots(rows, results.parent)
    else:
        config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
        files = make_plots(rows, config, results.parent)
    for path in files:
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
