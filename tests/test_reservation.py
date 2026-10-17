import math

from core.grid_map import GridMap
from core.reservation import ReservationTable
from tests.helpers import find_conflicts, plan_single


def test_reservations_keep_one_step_of_clearance():
    table = ReservationTable()
    table.reserve_path([(0, 0), (0, 1), (0, 2)])
    # The robot is on (0,1) at t=1 only; the cell is closed one step before and after.
    assert [table.is_free((0, 1), t) for t in range(5)] == [False, False, False, True, True]
    assert table.last_reserved((0, 1)) == 2
    # It parks on (0,2) from t=2 for good, so the cell closes at t=1.
    assert table.is_free((0, 2), 0)
    assert not table.is_free((0, 2), 1)
    assert not table.is_free((0, 2), 150)
    assert table.last_reserved((0, 2)) == math.inf
    assert table.last_reserved((5, 5)) == -1
    assert table.last_time() == 3


def test_start_cells_of_unplanned_robots_are_closed_at_first():
    table = ReservationTable()
    table.reserve_start((0, 0))
    assert not table.is_free((0, 0), 0) and not table.is_free((0, 0), 1)
    assert table.is_free((0, 0), 2)
    assert table.last_reserved((0, 0)) == 1
    table.release_start((0, 0))
    assert table.is_free((0, 0), 1)


def test_clearance_blocks_swap(battery):
    grid = GridMap([".."])
    # Without reservations the robot simply steps left.
    assert plan_single(grid, (0, 1), (0, 0), battery, t_max=20) == [(0, 1), (0, 0)]
    # Another robot moves (0,0) -> (0,1) at the same time: the two would swap cells.
    table = ReservationTable()
    table.reserve_path([(0, 0), (0, 1)])
    assert plan_single(grid, (0, 1), (0, 0), battery, t_max=20, reservations=table) is None


def test_no_tailgating(battery):
    grid = GridMap(["...."])
    table = ReservationTable()
    table.reserve_path([(0, 1), (0, 2), (0, 3)])  # leader, one cell ahead
    path = plan_single(grid, (0, 0), (0, 1), battery, t_max=20, reservations=table)
    # The leader is on (0,1) at t=0, so the follower may enter it at t=2, not t=1.
    assert path == [(0, 0), (0, 0), (0, 1)]


def test_waits_until_goal_is_no_longer_reserved(battery):
    grid = GridMap(["...", "..."])
    table = ReservationTable()
    other = [(1, 1), (1, 1), (1, 1), (0, 1), (0, 2)]  # passes (0,1) at t=3, parks on (0,2)
    table.reserve_path(other)
    path = plan_single(grid, (0, 0), (0, 1), battery, t_max=20, reservations=table)
    # The goal is closed at t=2..4, so the robot can only settle on it at t=5.
    assert len(path) - 1 == 5
    assert (0, 1) not in path[2:5]
    assert find_conflicts({"other": other, "me": path}) == []


def test_conflict_checker_detects_collisions():
    # Sanity check of the test helper itself.
    assert find_conflicts({"a": [(0, 0), (0, 1)], "b": [(0, 2), (0, 1)]})  # same cell
    assert find_conflicts({"a": [(0, 0), (0, 1)], "b": [(0, 1), (0, 0)]})  # swap
    assert find_conflicts({"a": [(0, 0)], "b": [(0, 2), (0, 1), (0, 0)]})  # runs over parked a
    assert not find_conflicts({"a": [(0, 0), (0, 1)], "b": [(1, 0), (1, 1)]})
