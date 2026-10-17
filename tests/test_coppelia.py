"""CoppeliaSim backend: one test that needs CoppeliaSim running (pass --coppelia)."""
import pytest

from robots.coppelia import run_scenario


@pytest.mark.coppelia
def test_s0_runs_in_coppeliasim(arena_map, config, arena_s0):
    """SPEC M7: scenario S0 completes in CoppeliaSim with the executor used for grid_sim."""
    # run_scenario raises if planning fails or a robot does not end on its goal.
    result, paths, dt, closest = run_scenario(arena_map, arena_s0, config, realtime=False)
    step_seconds = config["execution"]["step_seconds"]
    for robot in arena_s0:
        planned_seconds = (len(paths[robot.name]) - 1) * step_seconds
        real_seconds = result.finish_ticks[robot.name] * dt
        # Turning makes real moves slower than a plan step, never faster overall.
        assert planned_seconds <= real_seconds <= 1.5 * planned_seconds
    assert result.lowest_battery >= 0
    assert closest > config["cell_size"] / 2  # robots never came near touching
