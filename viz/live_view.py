"""Live window for the lifelong demo: the arena, each robot with its goal and planned
path, and a side panel with one card per robot (simulated battery, state, jobs done).

Every robot has one color throughout: its disc on the map, its goal star, its path
and its card in the panel. Used by execution/lifelong.py (grid_sim preview) and
robots/turtlebot_ros2.py (--lifelong --view). Only matplotlib, no ROS.
"""
import math
import time

import matplotlib.pyplot as plt
from matplotlib import patheffects
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

from core.grid_map import CHARGER, GridMap
from viz.animate import ROBOT_COLORS, draw_map

STATUS_TEXT = {"moving": "driving", "charging": "CHARGING", "waiting": "waiting for the other robot",
               "planning": "planning", "idle": "finished", "done": "finished"}
LOW = 0.25  # battery fraction shown as low


class LiveView:
    def __init__(self, grid: GridMap, names: list[str], battery_max: float, refresh_hz: float):
        self._grid, self._names, self._battery_max = grid, names, battery_max
        self._period = 1.0 / refresh_hz
        self._last_draw = -math.inf
        plt.ion()
        self.fig, (self._ax, self._panel) = plt.subplots(
            1, 2, figsize=(12, 5.6), gridspec_kw={"width_ratios": [grid.width, 4.6]})
        self.fig.canvas.manager.set_window_title("MRPP lifelong demo")
        draw_map(self._ax, grid)
        self._title = self._ax.set_title("lifelong demo   00:00", fontsize=13, fontweight="bold")
        self._ax.legend(handles=[
            Line2D([], [], marker="*", markersize=14, color="#777", linestyle="none", label="goal"),
            Line2D([], [], color="#777", linewidth=3, alpha=0.5, label="planned path"),
            Line2D([], [], marker="$C$", markersize=9, color="#c07a00", linestyle="none", label="charger"),
        ], loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3, frameon=False, fontsize=9)

        card_h = 2.2
        self._panel.set_xlim(0, 1)
        self._panel.set_ylim(len(names) * card_h + 0.9, 0)
        self._panel.axis("off")
        self._jobs_text = self._panel.text(0.02, 0.45, "", fontsize=13, fontweight="bold", va="center")

        self._artists = {}
        for i, name in enumerate(names):
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            # On the map: path, goal star, the robot disc with its heading and a battery tag.
            path, = self._ax.plot([], [], color=color, linewidth=4, alpha=0.45, zorder=2)
            goal, = self._ax.plot([], [], marker="*", markersize=24, color=color,
                                  markeredgecolor="white", linestyle="none", zorder=3)
            body = self._ax.add_patch(Circle((0, 0), 0.3, color=color, zorder=4, visible=False))
            heading, = self._ax.plot([], [], color="white", linewidth=2.5, zorder=5)
            label = self._ax.text(0, 0, name, ha="center", va="center", fontsize=9,
                                  color="white", fontweight="bold", zorder=6)
            tag = self._ax.text(0, 0, "", ha="center", va="top", fontsize=8, color="white",
                                fontweight="bold", zorder=6,
                                bbox=dict(boxstyle="round,pad=0.15", facecolor=color, edgecolor="none"))

            # In the panel: one card in the robot's color.
            top = 0.9 + i * card_h
            self._panel.add_patch(FancyBboxPatch((0.01, top + 0.05), 0.98, card_h - 0.25,
                                                 boxstyle="round,pad=0.01,rounding_size=0.03",
                                                 facecolor=color, alpha=0.12, edgecolor=color, linewidth=2))
            # A marker stays round although the panel's axes are not to scale.
            self._panel.plot([0.09], [top + 0.5], marker="o", markersize=28, color=color)
            self._panel.text(0.09, top + 0.5, name, ha="center", va="center", fontsize=10,
                             color="white", fontweight="bold")
            self._panel.text(0.2, top + 0.5, f"robot {name}", va="center", fontsize=12,
                             fontweight="bold", color=color)
            bar_y, bar_h = top + 0.85, 0.42
            self._panel.add_patch(Rectangle((0.05, bar_y), 0.9, bar_h, facecolor="white",
                                            edgecolor=color, linewidth=2))
            bar = self._panel.add_patch(Rectangle((0.05, bar_y), 0, bar_h, facecolor=color))
            level = self._panel.text(0.5, bar_y + bar_h / 2, "", ha="center", va="center",
                                     fontsize=11, fontweight="bold",  # white halo: readable on any fill
                                     path_effects=[patheffects.withStroke(linewidth=3, foreground="white")])
            state = self._panel.text(0.05, bar_y + bar_h + 0.15, "", va="top", fontsize=10)
            self._artists[name] = dict(path=path, goal=goal, body=body, heading=heading, label=label,
                                       tag=tag, bar=bar, level=level, state=state, color=color)
        self.fig.tight_layout(rect=(0, 0, 1, 0.97))

    def show(self, run, poses: dict[str, tuple[float, float, float | None]], elapsed: float) -> None:
        """Redraw at most `refresh_hz` times a second.

        `run` is a LifelongRun. `poses` gives each robot's (row, col, heading) in grid
        coordinates: row and col may be fractional, and heading is in radians from the
        direction of increasing column, counterclockwise as seen on the floor (None
        if unknown).
        """
        now = time.monotonic()
        if now - self._last_draw < self._period or self.closed:
            self.fig.canvas.flush_events()
            return
        self._last_draw = now
        ex = run.executor
        for name in self._names:
            a = self._artists[name]
            row, col, heading = poses[name]
            a["body"].set_visible(True)
            a["body"].center = (col, row)
            a["label"].set_position((col, row))
            if heading is None:
                a["heading"].set_data([], [])
            else:  # on screen rows grow downwards
                a["heading"].set_data([col, col + 0.3 * math.cos(heading)],
                                      [row, row - 0.3 * math.sin(heading)])
            goal = run.goal[name]
            rest = ex.remaining_path(name) if goal is not None else []
            a["path"].set_data([c for _, c in rest], [r for r, _ in rest])
            a["goal"].set_data(([goal[1]], [goal[0]]) if goal else ([], []))

            level = ex.levels[name][-1]
            fraction = max(0.0, min(1.0, level / self._battery_max))
            low = fraction <= LOW
            a["tag"].set_position((col, row + 0.34))
            a["tag"].set_text(f"{level:.0f}")
            a["bar"].set_width(0.9 * fraction)
            a["bar"].set_facecolor("#c0504d" if low else a["color"])
            a["level"].set_text(f"battery {level:.1f} / {self._battery_max:g}" + ("   LOW" if low else ""))

            status = run.status(name)
            lines = [STATUS_TEXT[status] + (f"  ->  goal {goal}" if goal else "")]
            if status != "charging" and any(self._grid.symbol(c) == CHARGER for c in rest[1:]):
                lines.append("will stop at the charger first")
            done = sum(1 for job in run.jobs if job.robot == name)
            lines.append(f"jobs done: {done}")
            a["state"].set_text("\n".join(lines))
        self._jobs_text.set_text(f"jobs done in total: {len(run.jobs)}")
        minutes, seconds = divmod(int(elapsed), 60)
        self._title.set_text(f"lifelong demo   {minutes:02d}:{seconds:02d}")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    @property
    def closed(self) -> bool:
        """True once the window has been closed; the demos stop the robots then."""
        return not plt.fignum_exists(self.fig.number)

    def pause(self, seconds: float) -> None:
        """Wait while keeping the window responsive."""
        if not self.closed:
            plt.pause(seconds)
        else:
            time.sleep(seconds)
