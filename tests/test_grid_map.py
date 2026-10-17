import pytest

from core.grid_map import GridMap


def test_challenge_map_size(challenge_map):
    assert (challenge_map.height, challenge_map.width) == (5, 11)


def test_passable_cells(challenge_map):
    assert challenge_map.is_passable((3, 0))   # corridor
    assert challenge_map.is_passable((0, 1))   # room A
    assert challenge_map.is_passable((2, 6))   # door
    assert challenge_map.is_passable((3, 10))  # charger
    assert not challenge_map.is_passable((2, 0))   # wall
    assert not challenge_map.is_passable((-1, 0))  # out of bounds
    assert not challenge_map.is_passable((3, 11))  # out of bounds


def test_neighbors(challenge_map):
    assert set(challenge_map.neighbors((3, 6))) == {(2, 6), (4, 6), (3, 5), (3, 7)}
    assert challenge_map.neighbors((3, 10)) == [(3, 9)]  # charger is a dead end
    assert challenge_map.neighbors((4, 6)) == [(3, 6)]   # side pocket


def test_special_cells(challenge_map):
    assert challenge_map.cells_with("C") == [(3, 10)]
    assert challenge_map.cells_with("D") == [(2, 1), (2, 6)]
    assert len(challenge_map.cells_with("A")) == 6
    assert len(challenge_map.cells_with("B")) == 6


def test_ragged_rows_rejected():
    with pytest.raises(ValueError):
        GridMap(["...", ".."])


def test_unknown_symbol_rejected():
    with pytest.raises(ValueError):
        GridMap(["..x"])
