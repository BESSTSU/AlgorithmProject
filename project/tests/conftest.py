from pathlib import Path

import pytest
import yaml

from core.battery import BatteryModel
from core.grid_map import GridMap
from core.scenario import load_scenario

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="session")
def config():
    return yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))


@pytest.fixture(scope="session")
def challenge_map(config):
    return GridMap.from_file(ROOT / config["map"])


@pytest.fixture(scope="session")
def battery(config):
    return BatteryModel(**config["battery"])


@pytest.fixture(scope="session")
def s0():
    """Reference scenario S0 as [R1, R2]."""
    return load_scenario(ROOT / "scenarios" / "s0.yaml")


@pytest.fixture(scope="session")
def s1():
    """Extra scenario S1 as [R1, R2]: both methods succeed."""
    return load_scenario(ROOT / "scenarios" / "s1.yaml")
