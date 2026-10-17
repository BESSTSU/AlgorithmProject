"""Map server and AMCL per robot, all localizing in the same `map` frame.

Two ways to run it:

  On the laptop, for every robot at once (one shared map server on /map):
    ros2 launch mrpp_bringup localization.launch.py map:=/path/to/arena.yaml

  On each robot, for that robot only (per_robot:=true): the robot gets its own
  map server on /<namespace>/map, so its AMCL needs nothing from the network,
  and scans never cross the Wi-Fi. robots/robot_up.py starts it this way.
    ros2 launch ~/mrpp/launch/localization.launch.py map:=~/mrpp/arena_6x5.yaml \
        namespaces:=tb3_0 per_robot:=true

Each robot has to be brought up in its own namespace (tb3_0, tb3_1, ...), which
gives it the topic <namespace>/scan and the frames <namespace>/odom and
<namespace>/base_footprint. Every AMCL publishes map -> <namespace>/odom, so all
robots end up in a single TF tree under one `map` frame.

AMCL starts without a pose. Send the initial poses from the scenario with
    python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml --localize
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Next to this file in the source tree, in the install space, and in ~/mrpp on a robot.
DEFAULT_PARAMS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "amcl.yaml")


def _amcl(namespace: str, params_file: str, map_topic: str) -> Node:
    return Node(package="nav2_amcl", executable="amcl", name="amcl", namespace=namespace,
                output="screen",
                parameters=[params_file, {
                    "global_frame_id": "map",
                    "odom_frame_id": f"{namespace}/odom",
                    "base_frame_id": f"{namespace}/base_footprint",
                    "scan_topic": "scan",  # relative: /<namespace>/scan
                }],
                remappings=[("map", map_topic)])


def _map_server(namespace: str, map_file: str) -> Node:
    return Node(package="nav2_map_server", executable="map_server", name="map_server",
                namespace=namespace, output="screen",
                parameters=[{"yaml_filename": map_file, "topic_name": "map", "frame_id": "map"}])


def _lifecycle(namespace: str, name: str, nodes: list[str]) -> Node:
    # bond_timeout 0 turns off the bond: when a heartbeat came late (seen on a robot
    # whose Wi-Fi was dropping), the manager reset AMCL, which then lost its pose.
    # A dead AMCL is still caught: its pose goes stale and the laptop holds the robot.
    return Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name=name,
                namespace=namespace, output="screen",
                parameters=[{"autostart": True, "node_names": nodes, "bond_timeout": 0.0}])


def _nodes(context):
    map_file = os.path.expanduser(LaunchConfiguration("map").perform(context))
    params_file = os.path.expanduser(LaunchConfiguration("params").perform(context))
    per_robot = LaunchConfiguration("per_robot").perform(context).lower() == "true"
    namespaces = [ns.strip() for ns in LaunchConfiguration("namespaces").perform(context).split(",")]
    for path in (map_file, params_file):
        if not os.path.isfile(path):
            raise RuntimeError(f"file not found: {path}")

    if per_robot:  # everything inside the robot's namespace: /<ns>/map, /<ns>/amcl
        nodes = []
        for namespace in filter(None, namespaces):
            nodes += [_map_server(namespace, map_file),
                      _amcl(namespace, params_file, "map"),  # relative: /<ns>/map
                      _lifecycle(namespace, "lifecycle_manager_localization", ["map_server", "amcl"])]
        return nodes

    nodes = [_map_server("", map_file), _lifecycle("", "lifecycle_manager_map", ["map_server"])]
    for namespace in filter(None, namespaces):
        nodes += [_amcl(namespace, params_file, "/map"),  # the one shared map
                  _lifecycle(namespace, "lifecycle_manager_localization", ["amcl"])]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Full path to the map YAML"),
        DeclareLaunchArgument("namespaces", default_value="tb3_0,tb3_1",
                              description="Comma-separated robot namespaces; match ros2.namespaces in config.yaml"),
        DeclareLaunchArgument("params", default_value=DEFAULT_PARAMS, description="AMCL parameter file"),
        DeclareLaunchArgument("per_robot", default_value="false",
                              description="true: map server and AMCL inside each namespace (run on the robot)"),
        OpaqueFunction(function=_nodes),
    ])
