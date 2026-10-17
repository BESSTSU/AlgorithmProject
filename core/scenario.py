"""Scenario definitions: which robots exist, where they start and what they must do."""
import random
from dataclasses import dataclass
from pathlib import Path

import yaml

from core.grid_map import Cell, GridMap


@dataclass(frozen=True)
class Robot:
    name: str
    start: Cell
    battery: int             # battery level at t = 0
    tasks: tuple[Cell, ...]  # cells to visit in order; the robot parks on the last one


def load_scenario(path) -> list[Robot]:
    """Read robots from a YAML scenario file, in robot-number order."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return [
        Robot(r["name"], tuple(r["start"]), r["battery"], tuple(tuple(c) for c in r["tasks"]))
        for r in data["robots"]
    ]


def load_scenario_map(scenario_path, config_path, config: dict) -> GridMap:
    """The map a scenario runs on: the one its file names, else the configured default.

    Map paths are relative to the folder of the config file.
    """
    data = yaml.safe_load(Path(scenario_path).read_text(encoding="utf-8"))
    return GridMap.from_file(Path(config_path).parent / data.get("map", config["map"]))


def random_scenario(grid: GridMap, rng: random.Random, n_robots: int, battery_min: int,
                    battery_max: int, tasks_min: int, tasks_max: int) -> list[Robot]:
    """Random robots R1..Rn for the map.

    Start cells are distinct passable cells, start batteries are uniform in
    [battery_min, battery_max], and each robot gets tasks_min..tasks_max distinct
    room cells as tasks. Final task cells differ between robots, so every robot
    has its own cell to park on.
    """
    passable = [(r, c) for r in range(grid.height) for c in range(grid.width)
                if grid.is_passable((r, c))]
    rooms = grid.room_cells()
    starts = rng.sample(passable, n_robots)
    finals = rng.sample(rooms, n_robots)
    robots = []
    for i in range(n_robots):
        others = [cell for cell in rooms if cell != finals[i]]
        earlier = rng.sample(others, rng.randint(tasks_min, tasks_max) - 1)
        battery = rng.randint(battery_min, battery_max)
        robots.append(Robot(f"R{i + 1}", starts[i], battery, (*earlier, finals[i])))
    return robots
