"""Backend-independent motion pieces: grid frame and the cell controller."""
import argparse
import math

import pytest

from robots.calibrate_grid import parse_sample
from robots.cell_controller import CellController, wheel_speeds
from robots.grid_frame import GridFrame, fit_grid_frame


# --- grid frame ----------------------------------------------------------------

def all_cells(grid):
    return [(r, c) for r in range(grid.height) for c in range(grid.width)]


def test_frame_at_the_origin(challenge_map, config):
    cs = config["cell_size"]
    frame = GridFrame(challenge_map.height, cs)
    # The bottom-left cell touches the origin; row 0 is the top row (largest y).
    assert frame.to_metric((challenge_map.height - 1, 0)) == pytest.approx((cs / 2, cs / 2))
    assert frame.to_metric((0, 0))[1] > frame.to_metric((challenge_map.height - 1, 0))[1]
    assert frame.to_metric((0, 3))[0] > frame.to_metric((0, 0))[0]
    assert all(frame.to_cell(frame.to_metric(cell)) == cell for cell in all_cells(challenge_map))


def test_frame_with_offset_and_rotation(challenge_map, config):
    cs = config["cell_size"]
    # Rotated a quarter turn: columns now run along +y and rows along +x.
    frame = GridFrame(challenge_map.height, cs, x=1.0, y=2.0, yaw=math.pi / 2)
    assert frame.to_metric((challenge_map.height - 1, 0)) == pytest.approx((1.0 - cs / 2, 2.0 + cs / 2))
    assert frame.to_metric((challenge_map.height - 1, 1)) == pytest.approx((1.0 - cs / 2, 2.0 + 1.5 * cs))
    for yaw in (0.3, -1.2, math.pi):
        frame = GridFrame(challenge_map.height, cs, x=-0.7, y=3.1, yaw=yaw)
        assert all(frame.to_cell(frame.to_metric(cell)) == cell for cell in all_cells(challenge_map))


# --- cell controller -----------------------------------------------------------

def drive(controller, pose, dt, max_seconds=60.0):
    """Integrate an ideal differential-drive robot under the controller; return the trace."""
    x, y, yaw = pose
    trace = []
    for _ in range(round(max_seconds / dt)):
        v, w = controller.command(x, y, yaw)
        if controller.arrived:
            break
        trace.append((x, y, yaw, v, w))
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
    return (x, y, yaw), trace


@pytest.fixture(params=["coppelia", "ros2"])
def setup(request, config):
    """Controller settings of each backend with that backend's control period."""
    backend = config[request.param]
    dt = 0.05 if request.param == "coppelia" else 1.0 / backend["control_hz"]
    limits = config["robot"]
    controller = CellController(backend["control"], limits["cruise_lin_vel"], limits["cruise_ang_vel"])
    return controller, backend["control"], limits, dt, config["cell_size"]


@pytest.mark.parametrize("direction", [(1, 0), (0, 1), (-1, 0), (0, -1)])
def test_reaches_the_neighbouring_cell(setup, direction):
    controller, control, limits, dt, cell = setup
    target = (direction[0] * cell, direction[1] * cell)
    controller.go_to(target)
    assert not controller.arrived
    (x, y, _), trace = drive(controller, (0.0, 0.0, 0.0), dt)
    assert controller.arrived
    assert math.hypot(target[0] - x, target[1] - y) < control["pos_tol"]
    assert controller.command(x, y, 0.0) == (0.0, 0.0)
    # Speeds stay within the cruise limits and the robot never reverses.
    assert all(0.0 <= v <= limits["cruise_lin_vel"] + 1e-9 for *_, v, _ in trace)
    assert all(abs(w) <= limits["cruise_ang_vel"] + 1e-9 for *_, w in trace)
    # It keeps to the straight line between the two cell centers.
    off_line = max(abs(px * direction[1] - py * direction[0]) for px, py, *_ in trace)
    assert off_line < control["pos_tol"]


def test_turns_on_the_spot_before_driving(setup):
    controller, control, _, dt, cell = setup
    controller.go_to((0.0, cell))  # a quarter turn to the left of the heading
    _, trace = drive(controller, (0.0, 0.0, 0.0), dt)
    first_move = next(i for i, (*_, v, _) in enumerate(trace) if v > 0)
    assert first_move > 0
    assert all(w > 0 for *_, w in trace[:first_move])                       # turning left
    assert abs(trace[first_move][2] - math.pi / 2) < control["ang_tol"] + 1e-9  # then faces the target


def test_stop_cancels_the_move(setup):
    controller, *_ = setup
    controller.go_to((1.0, 0.0))
    controller.stop()
    assert controller.arrived and controller.command(0.0, 0.0, 0.0) == (0.0, 0.0)


def test_wheel_speeds(config):
    model = config["coppelia"]["model"]
    radius, separation = model["wheel_radius"], model["wheel_separation"]
    v, w = config["robot"]["cruise_lin_vel"], config["robot"]["cruise_ang_vel"]
    # Straight: both wheels roll at v / r.
    assert wheel_speeds(v, 0.0, radius, separation) == pytest.approx((v / radius, v / radius))
    # Turning left in place: wheels spin opposite, right wheel forward.
    left, right = wheel_speeds(0.0, w, radius, separation)
    assert left == pytest.approx(-right) and right > 0
    assert (right - left) * radius / separation == pytest.approx(w)


# --- fitting the grid into the SLAM map ------------------------------------------

def test_fit_recovers_a_known_grid_placement(arena_map, config):
    true = GridFrame(arena_map.height, config["cell_size"], x=-0.31, y=1.27, yaw=0.4)
    samples = [(cell, true.to_metric(cell)) for cell in [(2, 0), (2, 5), (0, 0), (4, 4)]]
    fitted, errors = fit_grid_frame(arena_map.height, config["cell_size"], samples)
    assert (fitted.x, fitted.y, fitted.yaw) == pytest.approx((true.x, true.y, true.yaw))
    assert max(errors) < 1e-9
    # Two cells are enough when they are measured exactly.
    fitted, _ = fit_grid_frame(arena_map.height, config["cell_size"], samples[:2])
    assert (fitted.x, fitted.y, fitted.yaw) == pytest.approx((true.x, true.y, true.yaw))


def test_fit_reports_how_far_each_measurement_is_off(arena_map, config):
    true = GridFrame(arena_map.height, config["cell_size"], x=0.5, y=-0.2, yaw=-0.1)
    cells = [(2, 0), (2, 5), (0, 0), (4, 4)]
    samples = [(cell, true.to_metric(cell)) for cell in cells]
    x, y = samples[3][1]
    samples[3] = (cells[3], (x + 0.10, y))  # one robot was standing 10 cm off
    fitted, errors = fit_grid_frame(arena_map.height, config["cell_size"], samples)
    assert max(errors) == pytest.approx(errors[3]) and 0.03 < errors[3] < 0.10
    # The fit still puts every cell center inside the right cell.
    assert all(fitted.to_cell(true.to_metric(cell)) == cell for cell in cells)


def test_fit_needs_two_different_cells(arena_map, config):
    with pytest.raises(ValueError):
        fit_grid_frame(arena_map.height, config["cell_size"], [((2, 0), (0.0, 0.0))])
    with pytest.raises(ValueError):
        fit_grid_frame(arena_map.height, config["cell_size"],
                       [((2, 0), (0.0, 0.0)), ((2, 0), (0.01, 0.0))])


def test_calibration_sample_format():
    assert parse_sample("2,5:1.76,-0.03") == ((2, 5), (1.76, -0.03))
    with pytest.raises(argparse.ArgumentTypeError):
        parse_sample("2,5 1.76,-0.03")
