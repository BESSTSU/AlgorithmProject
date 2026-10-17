"""TurtleBot3 backend over ROS 2 (Humble).

Poses come from TF, map -> <namespace>/base_footprint, as produced by AMCL plus
the robot's odometry. Commands go out as geometry_msgs/Twist on
/<namespace>/<ros2.watchdog.topic>, to the watchdog on each robot that forwards
them to cmd_vel (robots/cmd_watchdog.py, started with robots/robot_up.py). The
same ADG executor that drives grid_sim and CoppeliaSim drives the robots.

    python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml --localize   # pre-flight, no motion
    python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml              # run the plan

Run it with the real robots localized on the shared map (ros2/mrpp_bringup), or
against robots/fake_turtlebot.py to rehearse without hardware.
"""
import argparse
import math
import signal
import sys
import threading
import time
from itertools import combinations
from pathlib import Path

import rclpy
import rclpy.time
import tf2_ros
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from core.grid_map import Cell, GridMap
from core.scenario import Robot, load_scenario, load_scenario_map
from execution.executor import ExecutionResult, execute, plan_for_execution, summary_lines
from execution.lifelong import LifelongRun, StopOnSignal, lifelong_battery, run_lifelong
from execution.lifelong import summary_lines as lifelong_summary
from robots.base import RobotInterface
from robots.cell_controller import CellController
from robots.grid_frame import GridFrame


def initial_pose(frame: GridFrame, cell: Cell, settings: dict,
                 map_frame: str) -> PoseWithCovarianceStamped:
    """AMCL initial pose for a robot standing on the center of `cell`.

    `settings` is `ros2.initial_pose` from the config: the heading the robots are
    placed with, relative to the direction of increasing column, and how
    uncertain that placement is.
    """
    message = PoseWithCovarianceStamped()
    message.header.frame_id = map_frame
    x, y = frame.to_metric(cell)
    yaw = frame.yaw + settings["heading"]
    message.pose.pose.position.x, message.pose.pose.position.y = x, y
    message.pose.pose.orientation.z = math.sin(yaw / 2)
    message.pose.pose.orientation.w = math.cos(yaw / 2)
    covariance = [0.0] * 36  # row-major 6x6 over (x, y, z, roll, pitch, yaw)
    covariance[0] = covariance[7] = settings["xy_std"] ** 2
    covariance[35] = settings["yaw_std"] ** 2
    message.pose.covariance = covariance
    return message


class TurtleBotRobot(RobotInterface):
    """One robot on the ROS graph, driven cell by cell with the shared CellController."""

    def __init__(self, world: "TurtleBotWorld", name: str, namespace: str):
        self.name = name
        self.xy = None  # last known position in the map frame
        self._world = world
        self._base_frame = f"{namespace}/{world.ros_cfg['base_frame']}"
        # Through the watchdog on the robot (robots/cmd_watchdog.py), which stops it if
        # these commands stop arriving.
        self._publisher = world.node.create_publisher(
            Twist, f"/{namespace}/{world.ros_cfg['watchdog']['topic']}", 10)
        limits = world.robot_cfg
        self._controller = CellController(world.ros_cfg["control"], limits["cruise_lin_vel"],
                                          limits["cruise_ang_vel"])
        self._last_stamp = None    # stamp of the newest transform seen
        self._last_change = 0.0    # local time at which that stamp first appeared

    def pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) in the map frame, or None if no new transform arrived recently.

        Freshness is judged by when a new stamp last showed up on this machine, not
        by comparing the stamp with the local clock, so it does not depend on the
        robots' clocks being in sync with this one.
        """
        try:
            transform = self._world.tf_buffer.lookup_transform(
                self._world.ros_cfg["map_frame"], self._base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
        now = time.monotonic()
        if stamp != self._last_stamp:
            self._last_stamp, self._last_change = stamp, now
        elif now - self._last_change > self._world.ros_cfg["pose_timeout"]:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.xy = (t.x, t.y)
        return t.x, t.y, yaw

    def get_cell(self) -> Cell:
        pose = self.pose()
        if pose is None:
            raise RuntimeError(f"no pose for {self.name} ({self._base_frame})")
        return self._world.frame.to_cell(pose[:2])

    def go_to_cell(self, cell: Cell) -> None:
        self._controller.go_to(self._world.frame.to_metric(cell))

    def is_arrived(self) -> bool:
        return self._controller.arrived

    def stop(self) -> None:
        self._controller.stop()
        self._publish(0.0, 0.0)

    def update(self) -> None:
        """One control tick: read the pose and publish a velocity command."""
        pose = self.pose()
        if pose is None:  # no fresh pose: stand still until localization is back
            self._publish(0.0, 0.0)
            return
        self._publish(*self._controller.command(*pose))

    def _publish(self, v: float, w: float) -> None:
        limits = self._world.robot_cfg
        command = Twist()
        command.linear.x = max(-limits["max_lin_vel"], min(limits["max_lin_vel"], v))
        command.angular.z = max(-limits["max_ang_vel"], min(limits["max_ang_vel"], w))
        self._publisher.publish(command)


class TurtleBotWorld:
    """The ROS node, the TF listener and one TurtleBotRobot per scenario robot."""

    def __init__(self, grid: GridMap, config: dict, names: list[str]):
        self.ros_cfg, self.robot_cfg = config["ros2"], config["robot"]
        namespaces = self.ros_cfg["namespaces"]
        if len(names) > len(namespaces):
            raise ValueError(f"{len(names)} robots but only {len(namespaces)} namespaces in config")
        origin = self.ros_cfg["grid_origin"]
        self.frame = GridFrame(grid.height, config["cell_size"], origin["x"], origin["y"], origin["yaw"])
        self.period = 1.0 / self.ros_cfg["control_hz"]

        # rclpy must not react to Ctrl-C itself: it would invalidate the context
        # before shutdown() can publish the stop commands, leaving the robots
        # driving on their last command.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("mrpp_executor")
        self.tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        # The node spins in its own thread, so poses keep arriving while the
        # control loop sleeps.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self.namespaces = dict(zip(names, namespaces))
        self.robots = {name: TurtleBotRobot(self, name, namespace)
                       for name, namespace in self.namespaces.items()}
        self._next_tick = None

    def send_initial_poses(self, robots: list[Robot]) -> None:
        """Tell each robot's AMCL that the robot stands on its scenario start cell.

        Raises RuntimeError if an AMCL is not listening within `pose_wait` seconds.
        """
        publishers = {
            robot.name: self.node.create_publisher(
                PoseWithCovarianceStamped, f"/{self.namespaces[robot.name]}/initialpose", 10)
            for robot in robots
        }
        deadline = time.monotonic() + self.ros_cfg["pose_wait"]
        waiting = list(publishers)
        while waiting and time.monotonic() < deadline:
            time.sleep(self.period)
            waiting = [name for name, pub in publishers.items() if pub.get_subscription_count() == 0]
        if waiting:
            topics = [f"/{self.namespaces[name]}/initialpose" for name in waiting]
            raise RuntimeError(f"nothing listens on {topics}; is the localization launch running?")
        for robot in robots:
            message = initial_pose(self.frame, robot.start, self.ros_cfg["initial_pose"],
                                   self.ros_cfg["map_frame"])
            message.header.stamp = self.node.get_clock().now().to_msg()
            publishers[robot.name].publish(message)

    def wait_for_poses(self) -> None:
        """Block until every robot has a pose. Raises RuntimeError after `pose_wait` seconds."""
        deadline = time.monotonic() + self.ros_cfg["pose_wait"]
        missing = list(self.robots)
        while missing and time.monotonic() < deadline:
            time.sleep(self.period)
            missing = [name for name, robot in self.robots.items() if robot.pose() is None]
        if missing:
            raise RuntimeError(f"no pose in frame '{self.ros_cfg['map_frame']}' for {missing}")

    def step(self) -> None:
        """Run every robot's controller once, then wait out the rest of the control period.

        Raises RuntimeError if two robots are closer than `safety_distance`: the plan
        never brings them that close, so localization or control has gone wrong. The
        caller's cleanup then stops every robot.
        """
        for robot in self.robots.values():
            robot.update()
        limit = self.ros_cfg["safety_distance"]
        for a, b in combinations(self.robots.values(), 2):
            if a.xy and b.xy and math.dist(a.xy, b.xy) < limit:
                raise RuntimeError(f"safety stop: {a.name} and {b.name} are "
                                   f"{math.dist(a.xy, b.xy):.2f} m apart (limit {limit} m)")
        now = time.monotonic()
        self._next_tick = max(now, (self._next_tick or now) + self.period)
        time.sleep(self._next_tick - now)

    def shutdown(self) -> None:
        """Stop every robot and leave the ROS graph. Safe to call after any failure."""
        for _ in range(3):  # repeated in case a message is dropped
            for robot in self.robots.values():
                robot.stop()
            time.sleep(self.period)
        self._executor.shutdown()
        self._spin_thread.join()
        self._listener.unregister()
        self.node.destroy_node()
        rclpy.shutdown()


def run_scenario(grid: GridMap, robots: list[Robot],
                 config: dict) -> tuple[ExecutionResult, dict[str, list[Cell]], float, float]:
    """Plan a scenario and execute it on the robots reachable over ROS.

    Returns (execution result, planned paths, seconds per tick, closest approach
    between two robot centers in meters). Refuses to move if a robot is not
    localized on its start cell. The robots are always stopped on the way out.
    """
    execution = config["execution"]
    paths, battery = plan_for_execution(grid, robots, config)
    world = TurtleBotWorld(grid, config, [robot.name for robot in robots])
    ticks_per_step = round(execution["step_seconds"] / world.period)
    tick, closest = 0, math.inf

    def step():
        nonlocal tick, closest
        world.step()
        tick += 1
        for a, b in combinations(world.robots.values(), 2):
            closest = min(closest, math.dist(a.xy, b.xy))
        if tick % ticks_per_step == 0:  # one status line per plan step of time
            status = "   ".join(f"{name} {world.frame.to_cell(robot.xy)} bat {robot.battery:5.1f}"
                                for name, robot in world.robots.items())
            print(f"{tick * world.period:6.1f} s   {status}", flush=True)

    try:
        world.wait_for_poses()
        for robot in robots:
            cell = world.robots[robot.name].get_cell()
            if cell != robot.start:
                raise RuntimeError(f"{robot.name} is on {cell}, but the scenario starts it on {robot.start}")
        result = execute(grid, paths, world.robots, {r.name: r.battery for r in robots}, battery,
                         step, world.period / execution["step_seconds"])
        for robot in robots:
            cell = world.robots[robot.name].get_cell()
            if cell != robot.tasks[-1]:
                raise RuntimeError(f"{robot.name} ended on {cell}, not on its goal {robot.tasks[-1]}")
    finally:
        world.shutdown()
    return result, paths, world.period, closest


def run_lifelong_on_robots(grid: GridMap, robots: list[Robot], config: dict,
                           max_jobs: int | None, view: bool) -> tuple[LifelongRun, float]:
    """The lifelong demo on the robots: new goals as soon as one is reached, until
    Ctrl-C, the live view's window is closed, or `max_jobs`. Returns the run and the
    seconds per tick.

    Refuses to move if a robot is not localized on its start cell. The robots are
    always stopped on the way out.
    """
    world = TurtleBotWorld(grid, config, [robot.name for robot in robots])
    steps_per_tick = world.period / config["execution"]["step_seconds"]
    ticks_per_step = round(1 / steps_per_tick)
    run = LifelongRun(grid, robots, world.robots, lifelong_battery(config), config,
                      steps_per_tick, max_jobs)
    live = None
    if view:
        from viz.live_view import LiveView  # noqa: PLC0415  (matplotlib only when asked for)
        live = LiveView(grid, run.names, run.battery.max, config["lifelong"]["view_hz"])
    reported, closest = 0, math.inf

    def on_tick(r):
        nonlocal reported, closest
        for a, b in combinations(world.robots.values(), 2):
            if a.xy and b.xy:
                closest = min(closest, math.dist(a.xy, b.xy))
        for job in r.jobs[reported:]:
            stop = " (charged on the way)" if job.charged else ""
            print(f"{job.end_tick * world.period:7.1f} s   {job.robot} reached {job.goal}{stop}", flush=True)
        reported = len(r.jobs)
        if r.executor.tick % ticks_per_step == 0:
            status = "   ".join(f"{name} {world.frame.to_cell(world.robots[name].xy)} "
                                f"bat {r.executor.levels[name][-1]:4.1f} {r.status(name)}"
                                for name in r.names)
            print(f"{r.executor.tick * world.period:7.1f} s   {status}", flush=True)
        if live:
            poses = {}
            for name in r.names:
                pose = world.robots[name].pose()
                xy = pose[:2] if pose else world.robots[name].xy
                heading = pose[2] - world.frame.yaw if pose else None
                poses[name] = (*world.frame.to_grid(xy), heading)
            live.show(r, poses, r.executor.tick * world.period)

    try:
        world.wait_for_poses()
        for robot in robots:
            cell = world.robots[robot.name].get_cell()
            if cell != robot.start:
                raise RuntimeError(f"{robot.name} is on {cell}, but the scenario starts it on {robot.start}")
        # Ctrl-C becomes a flag checked every tick: raised as an exception it could
        # land inside the view's event loop, which would swallow it.
        with StopOnSignal() as stop:
            finished = run_lifelong(run, world.step, on_tick,
                                    lambda: stop.requested or (live is not None and live.closed))
        if not finished:
            print("stop requested: stopping the robots", flush=True)
    except KeyboardInterrupt:  # Ctrl-C while waiting for the poses
        print("Ctrl-C: stopping the robots")
    finally:
        world.shutdown()
        print(f"closest approach between robot centers: {closest:.3f} m")
    return run, world.period


def localize(grid: GridMap, robots: list[Robot], config: dict) -> dict[str, tuple[Cell, float]]:
    """Pre-flight check that moves nothing: send AMCL the scenario's start poses and
    report where each robot is then localized.

    Returns each robot's cell and its distance in meters from the center of its
    start cell.
    """
    world = TurtleBotWorld(grid, config, [robot.name for robot in robots])
    try:
        world.send_initial_poses(robots)
        world.wait_for_poses()
        time.sleep(world.ros_cfg["initial_pose"]["settle"])  # let the particle filter converge
        found = {}
        for robot in robots:
            x, y, _ = world.robots[robot.name].pose()
            found[robot.name] = (world.frame.to_cell((x, y)),
                                 math.dist((x, y), world.frame.to_metric(robot.start)))
    finally:
        world.shutdown()
    return found


def main():
    parser = argparse.ArgumentParser(description="Plan a scenario and drive it on robots over ROS 2.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/s0.yaml")
    parser.add_argument("--localize", action="store_true",
                        help="only send AMCL the start poses and report where the robots are; no motion")
    parser.add_argument("--lifelong", action="store_true",
                        help="demo: cycle through each robot's tasks as jobs, a new one as soon as one is done")
    parser.add_argument("--jobs", type=int, default=None,
                        help="with --lifelong: stop after this many jobs (default: until Ctrl-C)")
    parser.add_argument("--view", action="store_true", help="with --lifelong: show the live view")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = load_scenario_map(args.scenario, config_path, config)
    robots = load_scenario(args.scenario)
    # Ctrl-C and termination must both unwind through run_scenario's cleanup, which
    # stops the robots. SIGINT is set explicitly because a shell may start this
    # process with it ignored.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    if args.localize:
        found = localize(grid, robots, config)
        for robot in robots:
            cell, distance = found[robot.name]
            verdict = "OK" if cell == robot.start else "NOT on its start cell"
            print(f"{robot.name}: localized on {cell}, {distance * 100:.1f} cm from the center"
                  f" of start cell {robot.start} -> {verdict}")
        sys.exit(0 if all(found[r.name][0] == r.start for r in robots) else 1)
    if args.lifelong:
        run, period = run_lifelong_on_robots(grid, robots, config, args.jobs, args.view)
        for line in lifelong_summary(run, period):
            print(line)
        return
    result, paths, period, closest = run_scenario(grid, robots, config)
    for line in summary_lines(robots, result, paths, period, config["execution"]["step_seconds"]):
        print(line)
    print(f"closest approach between robot centers: {closest:.3f} m")


if __name__ == "__main__":
    main()
