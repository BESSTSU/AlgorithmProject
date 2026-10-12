"""Grid animation of planned paths with matplotlib.

Demo (scenario S0):
    python -m viz.animate --scenario scenarios/s0.yaml [--method baseline]
"""
import argparse
import random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import to_rgb
from matplotlib.patches import Circle

from core.baseline import plan_baseline
from core.battery import BatteryModel
from core.grid_map import CHARGER, DOOR, FREE, WALL, Cell, GridMap
from core.prioritized import plan_prioritized
from core.scenario import load_scenario

CELL_COLORS = {WALL: "#2f4f4f", FREE: "#d9ece8", DOOR: "#f6d9a0", CHARGER: "#d9ece8"}
ROOM_COLOR = "#eef3f3"
DODGE_COLOR = CELL_COLORS[DOOR]
ROBOT_COLORS = ["#f2a93b", "#1f6f6b", "#c0504d", "#4f81bd", "#8064a2"]


def _background(grid: GridMap) -> np.ndarray:
    img = np.zeros((grid.height, grid.width, 3))
    for r in range(grid.height):
        for c in range(grid.width):
            color = CELL_COLORS.get(grid.symbol((r, c)), ROOM_COLOR)
            if grid.is_side_pocket((r, c)):
                color = DODGE_COLOR
            img[r, c] = to_rgb(color)
    return img


def animate_paths(grid: GridMap, paths: dict[str, list[Cell]], out_file, frame_ms: int,
                  batteries: dict[str, list[int]] | None = None) -> None:
    """Save a GIF showing every robot moving along its path, one frame per time step.

    A robot whose path has ended stays on its last cell. If `batteries` gives each
    robot's level per time step, the levels are shown in the title.
    """
    makespan = max(len(p) for p in paths.values()) - 1
    fig, ax = plt.subplots(figsize=(grid.width * 0.6, grid.height * 0.6 + 0.6))
    ax.imshow(_background(grid), interpolation="nearest")
    ax.set_xticks(np.arange(-0.5, grid.width), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.height), minor=True)
    ax.grid(which="minor", color="white", linewidth=1)
    ax.set_xticks(range(grid.width))
    ax.set_yticks(range(grid.height))
    ax.tick_params(which="both", length=0, labelsize=7)

    # Label each room once at its first cell, and mark chargers.
    labeled = set()
    for r, row in enumerate(grid.rows):
        for c, ch in enumerate(row):
            if ch == CHARGER:
                ax.text(c, r, "C", ha="center", va="center", fontweight="bold", color="#c07a00")
            elif ch.isupper() and ch != DOOR and ch not in labeled:
                labeled.add(ch)
                ax.text(c - 0.4, r - 0.4, ch, ha="left", va="top", fontweight="bold", color="#555")

    markers = {}
    for i, (name, path) in enumerate(paths.items()):
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        rows, cols = zip(*path)
        ax.plot(cols, rows, color=color, linewidth=2, alpha=0.35)
        gr, gc = path[-1]
        ax.add_patch(Circle((gc, gr), 0.38, fill=False, edgecolor=color, linewidth=2))
        body = ax.add_patch(Circle((path[0][1], path[0][0]), 0.33, color=color, zorder=3))
        label = ax.text(path[0][1], path[0][0], name, ha="center", va="center",
                        fontsize=7, color="white", fontweight="bold", zorder=4)
        markers[name] = (body, label)
    title = ax.set_title("t = 0")

    def update(t):
        for name, path in paths.items():
            r, c = path[min(t, len(path) - 1)]
            body, label = markers[name]
            body.center = (c, r)
            label.set_position((c, r))
        text = f"t = {t}"
        if batteries:
            levels = [f"{name}: {b[min(t, len(b) - 1)]}" for name, b in batteries.items()]
            text += "    battery  " + "   ".join(levels)
        title.set_text(text)
        return []

    anim = FuncAnimation(fig, update, frames=makespan + 1, interval=frame_ms)
    anim.save(out_file, writer=PillowWriter(fps=1000 / frame_ms))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plan a scenario with prioritized planning and save a GIF.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/s0.yaml")
    parser.add_argument("--method", choices=["main", "baseline"], default="main")
    parser.add_argument("--out", default="outputs/plan.gif")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = GridMap.from_file(config_path.parent / config["map"])
    battery = BatteryModel(**config["battery"])
    robots = load_scenario(args.scenario)
    planner = config["planner"]
    common = (grid, robots, battery, planner["t_max"], planner["priority"],
              planner["max_restarts"], random.Random(config["experiments"]["seed"]))
    if args.method == "main":
        result = plan_prioritized(*common)
    else:
        result = plan_baseline(*common, config["baseline"]["threshold"])
    if result.paths is None:
        raise SystemExit(f"planning failed: {result.failure} ({result.failed_robot})")
    paths = result.paths
    batteries = {r.name: battery.trace(grid, paths[r.name], r.battery) for r in robots}
    for name, path in paths.items():
        print(f"{name} ({len(path) - 1} steps): {path}")
        print(f"   battery: {batteries[name]}")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    animate_paths(grid, paths, out, config["viz"]["frame_ms"], batteries)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
