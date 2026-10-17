from pathlib import Path

import pytest
import yaml

from core.battery import BatteryModel
from core.grid_map import GridMap
from core.scenario import load_scenario

ROOT = Path(__file__).resolve().parent.parent


def pytest_addoption(parser):
    parser.addoption("--coppelia", action="store_true",
                     help="also run tests that drive a running CoppeliaSim (they rebuild its scene)")
    parser.addoption("--ros", action="store_true",
                     help="also run tests that execute a scenario over ROS 2 in real time")


def pytest_collection_modifyitems(config, items):
    for option in ("coppelia", "ros"):
        if config.getoption(f"--{option}"):
            continue
        skip = pytest.mark.skip(reason=f"pass --{option} to run")
        for item in items:
            if item.get_closest_marker(option):
                item.add_marker(skip)


@pytest.fixture(scope="session")
def config():
    return yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="session")
def arena_map(config):
    """The real arena: the project's main map."""
    return GridMap.from_file(ROOT / config["map"])


@pytest.fixture(scope="session")
def challenge_map():
    """The 11x5 simulation map most planner tests were written for."""
    return GridMap.from_file(ROOT / "maps" / "challenge_11x5.txt")


@pytest.fixture(scope="session")
def battery(config):
    return BatteryModel(**config["battery"])


@pytest.fixture(scope="session")
def arena_s0():
    """Reference scenario S0 on the arena, as [R1, R2]."""
    return load_scenario(ROOT / "scenarios" / "s0.yaml")


@pytest.fixture(scope="session")
def challenge_s0():
    """The original S0 on the 11x5 map, as [R1, R2]."""
    return load_scenario(ROOT / "scenarios" / "challenge_s0.yaml")


@pytest.fixture(scope="session")
def challenge_s1():
    """Extra 11x5 scenario as [R1, R2]: both methods succeed."""
    return load_scenario(ROOT / "scenarios" / "challenge_s1.yaml")
