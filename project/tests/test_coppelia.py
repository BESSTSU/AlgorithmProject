"""Pure helpers of the CoppeliaSim backend. These do not need CoppeliaSim running."""
import pytest

from robots.coppelia import cell_to_world, wheel_speeds, world_to_cell


def test_cell_world_round_trip(challenge_map, config):
    cs = config["cell_size"]
    for r in range(challenge_map.height):
        for c in range(challenge_map.width):
            assert world_to_cell(challenge_map, cell_to_world(challenge_map, (r, c), cs), cs) == (r, c)


def test_world_axes(challenge_map, config):
    cs = config["cell_size"]
    # Bottom-left cell sits at the origin corner; row 0 is at the top (largest y).
    assert cell_to_world(challenge_map, (challenge_map.height - 1, 0), cs) == (cs / 2, cs / 2)
    x_left, y_top = cell_to_world(challenge_map, (0, 0), cs)
    x_right, y_bottom = cell_to_world(challenge_map, (challenge_map.height - 1, challenge_map.width - 1), cs)
    assert x_right > x_left and y_top > y_bottom


def test_wheel_speeds(config):
    model = config["coppelia"]["model"]
    radius, separation = model["wheel_radius"], model["wheel_separation"]
    v = config["robot"]["cruise_lin_vel"]
    w = config["robot"]["cruise_ang_vel"]
    # Straight: both wheels roll at v / r.
    assert wheel_speeds(v, 0.0, radius, separation) == pytest.approx((v / radius, v / radius))
    # Turning left in place: wheels spin opposite, right wheel forward.
    left, right = wheel_speeds(0.0, w, radius, separation)
    assert left == pytest.approx(-right) and right > 0
    assert (right - left) * radius / separation == pytest.approx(w)
