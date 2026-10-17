"""ROS 2 backend, rehearsed against simulated robots. Needs rclpy, so it runs in WSL/Ubuntu only."""
import math
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy")

from robots.grid_frame import GridFrame  # noqa: E402
from core.scenario import load_scenario  # noqa: E402
from robots.turtlebot_ros2 import (initial_pose, localize, run_lifelong_on_robots,  # noqa: E402
                                   run_scenario)  # (needs rclpy)
from tests.conftest import ROOT  # noqa: E402


def test_initial_pose_message(arena_map, config):
    settings = config["ros2"]["initial_pose"]
    # A grid rotated a quarter turn and shifted: columns run along +y of the map.
    frame = GridFrame(arena_map.height, config["cell_size"], x=1.0, y=2.0, yaw=math.pi / 2)
    message = initial_pose(frame, (2, 0), settings, "map")
    x, y = frame.to_metric((2, 0))
    assert message.header.frame_id == "map"
    assert (message.pose.pose.position.x, message.pose.pose.position.y) == pytest.approx((x, y))
    q = message.pose.pose.orientation
    yaw = 2 * math.atan2(q.z, q.w)
    assert yaw == pytest.approx(math.pi / 2 + settings["heading"])
    covariance = list(message.pose.covariance)
    assert covariance[0] == covariance[7] == pytest.approx(settings["xy_std"] ** 2)
    assert covariance[35] == pytest.approx(settings["yaw_std"] ** 2)
    assert sum(1 for value in covariance if value) == 3


@pytest.mark.ros
def test_localize_with_split_fakes(arena_map, config, arena_s0):
    """--only splits the robots across processes, and a fake takes the initial pose like AMCL."""
    fakes = [subprocess.Popen([sys.executable, "-m", "robots.fake_turtlebot",
                               "--config", str(ROOT / "config.yaml"),
                               "--scenario", str(ROOT / "scenarios" / "s0.yaml"),
                               "--only", namespace], cwd=ROOT)
             for namespace in config["ros2"]["namespaces"][:len(arena_s0)]]
    try:
        found = localize(arena_map, arena_s0, config)
    finally:
        for fake in fakes:
            fake.terminate()
            fake.wait(timeout=10)
    for robot in arena_s0:
        cell, distance = found[robot.name]
        assert cell == robot.start
        assert distance < 0.01


def start_fakes(scenario):
    return subprocess.Popen([sys.executable, "-m", "robots.fake_turtlebot",
                             "--config", str(ROOT / "config.yaml"), "--scenario", str(scenario)], cwd=ROOT)


@pytest.mark.ros
def test_lifelong_over_ros(arena_map, config):
    scenario = ROOT / "scenarios" / "demo_ab.yaml"
    fake = start_fakes(scenario)
    try:
        run, _ = run_lifelong_on_robots(arena_map, load_scenario(scenario), config, max_jobs=3, view=False)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    assert len(run.jobs) == 3
    assert run.lowest_battery() >= 0


@pytest.mark.ros
def test_ctrl_c_stops_the_lifelong_demo_with_the_view_open():
    """Regression: with the live view open, Ctrl-C used to be swallowed by the GUI
    event loop and the demo kept running. Needs a display (WSLg)."""
    scenario = ROOT / "scenarios" / "demo_ab.yaml"
    fake = start_fakes(scenario)
    demo = subprocess.Popen([sys.executable, "-m", "robots.turtlebot_ros2", "--config", str(ROOT / "config.yaml"),
                             "--scenario", str(scenario), "--lifelong", "--view"],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(15)  # well into the run, window drawing
        demo.send_signal(signal.SIGINT)
        out, _ = demo.communicate(timeout=10)
    finally:
        demo.kill()
        fake.terminate()
        fake.wait(timeout=10)
    assert demo.returncode == 0
    assert "stop requested" in out and "R1:" in out


@pytest.mark.ros
def test_watchdog_stops_the_robot_when_commands_stop(config):
    """The watchdog forwards commands and sends a stop once they have stopped for `timeout`."""
    import rclpy
    from geometry_msgs.msg import Twist, Vector3

    timeout = config["ros2"]["watchdog"]["timeout"]
    topic = config["ros2"]["watchdog"]["topic"]
    watchdog = subprocess.Popen([sys.executable, str(ROOT / "robots" / "cmd_watchdog.py"),
                                 "--namespace", "wd_test", "--input", topic, "--timeout", str(timeout)])
    rclpy.init()
    node = rclpy.create_node("watchdog_test")
    out = node.create_publisher(Twist, f"/wd_test/{topic}", 10)
    seen = []  # (time, angular z) of what reaches cmd_vel
    node.create_subscription(Twist, "/wd_test/cmd_vel",
                             lambda m: seen.append((time.monotonic(), m.angular.z)), 10)

    def spin(seconds, publish=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish:
                out.publish(Twist(angular=Vector3(z=0.3)))
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        deadline = time.monotonic() + 10
        while out.get_subscription_count() == 0 and time.monotonic() < deadline:
            spin(0.1)
        spin(1.5, publish=True)           # driving
        cut = time.monotonic()
        spin(2 * timeout + 1.0)           # the laptop goes silent
    finally:
        node.destroy_node()
        rclpy.shutdown()
        watchdog.send_signal(signal.SIGINT)
        watchdog.wait(timeout=10)
    assert any(z == pytest.approx(0.3) for t, z in seen if t < cut)  # commands were forwarded
    stops = [t for t, z in seen if t >= cut and z == 0.0]
    assert stops, "no stop after the commands ended"
    assert stops[0] - cut <= timeout + 0.3
    assert seen[-1][1] == 0.0          # and nothing but stops since
    assert len(stops) <= 3             # then silent, so a teleop can still drive


@pytest.mark.ros
def test_safety_stop(arena_map, config, arena_s0):
    # The start cells of S0 are 1.8 m apart: a 2 m limit must stop the run at once.
    strict = {**config, "ros2": {**config["ros2"], "safety_distance": 2.0}}
    fake = start_fakes(ROOT / "scenarios" / "s0.yaml")
    try:
        with pytest.raises(RuntimeError, match="safety stop"):
            run_scenario(arena_map, arena_s0, strict)
    finally:
        fake.terminate()
        fake.wait(timeout=10)


@pytest.mark.ros
def test_s0_runs_over_ros(arena_map, config, arena_s0):
    """Scenario S0 over real ROS topics and TF, with the executor used everywhere else.

    The robots are robots/fake_turtlebot.py in a separate process, so commands and
    poses really travel through DDS.
    """
    fake = subprocess.Popen([sys.executable, "-m", "robots.fake_turtlebot",
                             "--config", str(ROOT / "config.yaml"),
                             "--scenario", str(ROOT / "scenarios" / "s0.yaml")], cwd=ROOT)
    try:
        # run_scenario raises if a robot is off its start cell or misses its goal.
        result, paths, period, closest = run_scenario(arena_map, arena_s0, config)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    step_seconds = config["execution"]["step_seconds"]
    for robot in arena_s0:
        planned_seconds = (len(paths[robot.name]) - 1) * step_seconds
        real_seconds = result.finish_ticks[robot.name] * period
        assert planned_seconds <= real_seconds <= 1.5 * planned_seconds
    assert result.lowest_battery >= 0
    assert closest > config["cell_size"] / 2  # robots never came near touching
