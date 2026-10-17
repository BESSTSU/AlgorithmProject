"""Stand-in for the real robots on the ROS graph: no hardware and no AMCL needed.

For every robot of a scenario it listens to the commands the laptop sends
(/<namespace>/<ros2.watchdog.topic>, what a real robot's watchdog receives) and publishes
the transform map -> <namespace>/base_footprint of an ideal differential-drive
robot that starts on its scenario cell. That is everything robots/turtlebot_ros2.py
uses, so a scenario can be rehearsed over real ROS topics and TF:

    python3 -m robots.fake_turtlebot --scenario scenarios/s0.yaml    # terminal 1
    python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml    # terminal 2

With --only, just the listed robots are simulated and the rest are real, e.g. one
real robot localized by AMCL as tb3_0 plus an imaginary second robot:

    python3 -m robots.fake_turtlebot --scenario scenarios/s0.yaml --only tb3_1
"""
import argparse
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from core.scenario import load_scenario, load_scenario_map
from robots.grid_frame import GridFrame


class FakeTurtleBots(Node):
    def __init__(self, poses: dict[str, list[float]], config: dict):
        """`poses` maps each namespace to its start [x, y, yaw] in the map frame."""
        super().__init__("fake_turtlebots")
        self._ros, self._limits = config["ros2"], config["robot"]
        self._poses = poses
        self._commands = {namespace: (0.0, 0.0) for namespace in poses}
        self._broadcaster = TransformBroadcaster(self)
        for namespace in poses:  # the commands the laptop sends a real robot's watchdog
            self.create_subscription(Twist, f"/{namespace}/{self._ros['watchdog']['topic']}",
                                     lambda msg, ns=namespace: self._on_command(ns, msg), 10)
            # Stands in for AMCL here, so `turtlebot_ros2 --localize` finds a listener.
            self.create_subscription(PoseWithCovarianceStamped, f"/{namespace}/initialpose",
                                     lambda msg, ns=namespace: self._on_initial_pose(ns, msg), 10)
        self._dt = 1.0 / self._ros["fake_hz"]
        self.create_timer(self._dt, self._tick)

    def _on_command(self, namespace: str, msg: Twist) -> None:
        v_max, w_max = self._limits["max_lin_vel"], self._limits["max_ang_vel"]
        self._commands[namespace] = (max(-v_max, min(v_max, msg.linear.x)),
                                     max(-w_max, min(w_max, msg.angular.z)))

    def _on_initial_pose(self, namespace: str, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self._poses[namespace][:] = [p.position.x, p.position.y, yaw]

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for namespace, pose in self._poses.items():
            v, w = self._commands[namespace]
            pose[0] += v * math.cos(pose[2]) * self._dt
            pose[1] += v * math.sin(pose[2]) * self._dt
            pose[2] += w * self._dt
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._ros["map_frame"]
            transform.child_frame_id = f"{namespace}/{self._ros['base_frame']}"
            transform.transform.translation.x = pose[0]
            transform.transform.translation.y = pose[1]
            transform.transform.rotation.z = math.sin(pose[2] / 2)
            transform.transform.rotation.w = math.cos(pose[2] / 2)
            transforms.append(transform)
        self._broadcaster.sendTransform(transforms)


def main():
    parser = argparse.ArgumentParser(description="Simulate the scenario's robots on the ROS graph.")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--scenario", default="scenarios/s0.yaml")
    parser.add_argument("--only", nargs="+", metavar="NAMESPACE",
                        help="simulate only these robots; the others are real")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = load_scenario_map(args.scenario, config_path, config)
    origin = config["ros2"]["grid_origin"]
    frame = GridFrame(grid.height, config["cell_size"], origin["x"], origin["y"], origin["yaw"])
    robots = load_scenario(args.scenario)
    poses = {namespace: [*frame.to_metric(robot.start), origin["yaw"]]
             for robot, namespace in zip(robots, config["ros2"]["namespaces"])}
    if args.only:
        unknown = set(args.only) - set(poses)
        if unknown:
            parser.error(f"not a robot of the scenario: {', '.join(sorted(unknown))}")
        poses = {namespace: pose for namespace, pose in poses.items() if namespace in args.only}

    rclpy.init()
    node = FakeTurtleBots(poses, config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C or terminated
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
