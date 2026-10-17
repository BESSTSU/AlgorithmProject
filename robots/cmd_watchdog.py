"""Command watchdog that runs ON the robot (its Raspberry Pi), next to the bringup.

The laptop sends velocity commands to /<namespace>/<input topic> instead of
/<namespace>/cmd_vel. This node forwards each one to cmd_vel at once, and if no
command arrives for `timeout` seconds it sends a stop. So the robot halts by
itself when the Wi-Fi drops or the laptop's program hangs: turtlebot3_node has
no such timeout and would keep driving on the last command it got.

After a stop it stays silent until commands come back, so a teleop that writes
to cmd_vel directly still works.

    python3 cmd_watchdog.py --namespace tb3_0 --input cmd_vel_remote --timeout 0.5

Started by robots/robot_up.py; values come from ros2.watchdog in config.yaml.
Needs only rclpy, which the robot has with ROS 2.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdWatchdog(Node):
    def __init__(self, namespace: str, input_topic: str, timeout: float):
        super().__init__("cmd_watchdog", namespace=namespace)
        self._timeout = timeout
        self._last = None      # local time the last command arrived
        self._stopped = True
        self._out = self.create_publisher(Twist, "cmd_vel", 10)
        # Depth 1: after a dropout, an old queued command is worth nothing.
        self.create_subscription(Twist, input_topic, self._on_command, QoSProfile(depth=1))
        self.create_timer(timeout / 5, self._check)
        self.get_logger().info(f"forwarding {input_topic} to cmd_vel, stop after {timeout} s without one")

    def _on_command(self, msg: Twist) -> None:
        self._out.publish(msg)
        self._last = time.monotonic()
        if self._stopped:
            self.get_logger().info("commands arriving")
        self._stopped = False

    def _check(self) -> None:
        if self._stopped or time.monotonic() - self._last <= self._timeout:
            return
        self.stop()
        self.get_logger().warn(f"no command for {self._timeout} s: robot stopped")

    def stop(self) -> None:
        for _ in range(3):  # cheap insurance, the subscriber is on this machine
            self._out.publish(Twist())
        self._stopped = True


def main():
    parser = argparse.ArgumentParser(description="Stop the robot when the laptop's commands stop.")
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--input", required=True, help="topic the laptop publishes to, in the namespace")
    parser.add_argument("--timeout", type=float, required=True, help="seconds without a command before stopping")
    args = parser.parse_args()

    rclpy.init()
    node = CmdWatchdog(args.namespace, args.input, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
