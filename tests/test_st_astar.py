import pytest

from core.grid_map import GridMap
from tests.helpers import assert_valid_path, plan_single
from viz.animate import animate_paths


def test_straight_corridor(challenge_map, battery, config):
    path = plan_single(challenge_map, (3, 0), (3, 9), battery, config["planner"]["t_max"])
    assert path == [(3, c) for c in range(10)]


@pytest.mark.parametrize(
    "start, goal, steps",
    [
        ((3, 2), (0, 6), 7),    # S0 R1 route without charging
        ((3, 8), (0, 1), 10),   # S0 R2 route
        ((0, 1), (0, 6), 11),   # room A -> room B: Manhattan is 5, walls force a detour
    ],
)
def test_optimal_path_avoids_walls(challenge_map, battery, config, start, goal, steps):
    path = plan_single(challenge_map, start, goal, battery, config["planner"]["t_max"])
    assert_valid_path(challenge_map, path, start, goal)
    assert len(path) - 1 == steps


def test_room_to_room_goes_through_both_doors(challenge_map, battery, config):
    path = plan_single(challenge_map, (0, 1), (0, 6), battery, config["planner"]["t_max"])
    assert (2, 1) in path and (2, 6) in path


def test_start_equals_goal(challenge_map, battery, config):
    path = plan_single(challenge_map, (3, 4), (3, 4), battery, config["planner"]["t_max"])
    assert path == [(3, 4)]


def test_horizon_too_short_returns_none(challenge_map, battery):
    # (0,1) -> (0,6) needs exactly 11 steps.
    assert plan_single(challenge_map, (0, 1), (0, 6), battery, t_max=10) is None
    assert len(plan_single(challenge_map, (0, 1), (0, 6), battery, t_max=11)) == 12


def test_unreachable_goal_returns_none(battery, config):
    grid = GridMap(["..#..", "..#.."])
    assert plan_single(grid, (0, 0), (0, 4), battery, config["planner"]["t_max"]) is None


def test_blocked_start_or_goal_rejected(challenge_map, battery, config):
    with pytest.raises(ValueError):
        plan_single(challenge_map, (2, 0), (3, 0), battery, config["planner"]["t_max"])
    with pytest.raises(ValueError):
        plan_single(challenge_map, (3, 0), (2, 0), battery, config["planner"]["t_max"])


def test_animation_writes_gif(challenge_map, battery, config, tmp_path):
    path = plan_single(challenge_map, (3, 2), (0, 6), battery, config["planner"]["t_max"])
    out = tmp_path / "anim.gif"
    animate_paths(challenge_map, {"R1": path}, out, config["viz"]["frame_ms"])
    assert out.stat().st_size > 0
