"""Start or stop one robot's software over SSH, from the laptop.

    python -m robots.robot_up turtlebest1@10.106.173.215 tb3_0           # start
    python -m robots.robot_up turtlebest1@10.106.173.215 tb3_0 --down    # stop
    python -m robots.robot_up turtlebest1@10.106.173.215 tb3_0 --status  # last log lines

Start copies to ~/mrpp on the robot the command watchdog, the localization launch
file with its AMCL parameters, and the layout map, then runs there, detached from
the SSH session:
  - the bringup, in the namespace;
  - the command watchdog (robots/cmd_watchdog.py), which stops the robot when the
    laptop's commands stop arriving;
  - map server and AMCL for this robot (ros2/mrpp_bringup, per_robot:=true), so
    scans never cross the Wi-Fi and the laptop's clock does not matter.
A Wi-Fi dropout does not kill them. Logs go to ~/mrpp/{bringup,watchdog,localization}.log
on the robot. Needs SSH key login to the robot, and nav2_amcl, nav2_map_server and
nav2_lifecycle_manager installed on it.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
REMOTE_DIR = "~/mrpp"
BRINGUP = "ros2 launch turtlebot3_bringup robot.launch.py"
LOGS = ("bringup", "watchdog", "localization")


def ssh(target: str, command: str, check: bool = True) -> str:
    # bash -i so that ~/.bashrc sets up ROS, ROS_DOMAIN_ID, TURTLEBOT3_MODEL and LDS_MODEL.
    result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", target,
                             f"bash -ic {quote(command)}"], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise SystemExit(f"ssh {target} failed: {result.stderr.strip()}")
    return result.stdout


def quote(text: str) -> str:
    return "'" + text.replace("'", "'\"'\"'") + "'"


def scp(target: str, sources: list[Path], remote_dir: str) -> None:
    result = subprocess.run(["scp", "-o", "BatchMode=yes", *map(str, sources), f"{target}:{remote_dir}/"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f"scp to {target} failed: {result.stderr.strip()}")


def stop_command() -> str:
    """Shell command that stops everything robot_up started on the robot.

    SIGINT lets the watchdog send its last stop and the launches shut their nodes
    down; whatever is left is killed after 3 s. The [x]yz patterns keep pkill from
    matching the shell that runs this very command.
    """
    launches = "[r]os2 launch (turtlebot3_bringup|.*localization.launch.py)"
    nodes = ("[t]urtlebot3_ros|[l]d08_driver|[r]obot_state_publisher|[c]md_watchdog.py|"
             "nav2_amcl/[a]mcl|[m]ap_server/map_server|[l]ifecycle_manager/lifecycle_manager")
    return (f"pkill -INT -f '{launches}'; pkill -INT -f '[c]md_watchdog.py'; sleep 3; "
            f"pkill -f '{launches}'; pkill -f '{nodes}'; true")


def up(target: str, namespace: str, config: dict) -> None:
    watchdog = config["ros2"]["watchdog"]
    map_yaml = ROOT / (config["ros2"]["layout_map"]["out"] + ".yaml")
    bringup_pkg = ROOT / "ros2" / "mrpp_bringup"
    ssh(target, f"mkdir -p {REMOTE_DIR}/launch {REMOTE_DIR}/config {REMOTE_DIR}/maps")
    scp(target, [Path(__file__).with_name("cmd_watchdog.py")], REMOTE_DIR)
    scp(target, [bringup_pkg / "launch" / "localization.launch.py"], f"{REMOTE_DIR}/launch")
    scp(target, [bringup_pkg / "config" / "amcl.yaml"], f"{REMOTE_DIR}/config")
    scp(target, [map_yaml, map_yaml.with_suffix(".pgm")], f"{REMOTE_DIR}/maps")
    ssh(target, stop_command(), check=False)  # never two of anything at once

    # Detached from the SSH session, with unbuffered logs so status shows them live.
    detach = ("RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1 "
              "setsid nohup {} > " + REMOTE_DIR + "/{}.log 2>&1 < /dev/null &")
    ssh(target, " ".join([  # each part ends with "&"
        detach.format(f"{BRINGUP} namespace:={namespace}", "bringup"),
        detach.format(f"python3 {REMOTE_DIR}/cmd_watchdog.py --namespace {namespace} "
                      f"--input {watchdog['topic']} --timeout {watchdog['timeout']}", "watchdog"),
        detach.format(f"ros2 launch {REMOTE_DIR}/launch/localization.launch.py "
                      f"map:={REMOTE_DIR}/maps/{map_yaml.name} namespaces:={namespace} per_robot:=true",
                      "localization"),
    ]))
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        time.sleep(2)
        logs = ssh(target, " ".join(f"cat {REMOTE_DIR}/{name}.log;" for name in LOGS), check=False)
        if ("diff_drive_controller]: Run!" in logs and "forwarding" in logs
                and "Managed nodes are active" in logs):
            print(f"{target} ({namespace}): bringup, watchdog and localization running")
            return
    print(status(target))
    raise SystemExit(f"{target}: not everything came up within 40 s (logs above)")


def status(target: str) -> str:
    tails = " echo ---; ".join(f"tail -n 4 {REMOTE_DIR}/{name}.log;" for name in LOGS)
    return ssh(target, f"{tails} echo ---; "
                       f"pgrep -fa '[t]urtlebot3_ros|[c]md_watchdog|nav2_amcl/[a]mcl' || echo 'nothing running'",
               check=False)


def main():
    parser = argparse.ArgumentParser(description="Start or stop a robot's bringup, watchdog and localization.")
    parser.add_argument("target", help="user@host of the robot")
    parser.add_argument("namespace", help="tb3_0, tb3_1, ... as in ros2.namespaces")
    parser.add_argument("--config", default="config.yaml")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--down", action="store_true", help="stop everything")
    action.add_argument("--status", action="store_true", help="print the last log lines")
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    if args.namespace not in config["ros2"]["namespaces"]:
        parser.error(f"{args.namespace} is not in ros2.namespaces")
    if args.down:
        ssh(args.target, stop_command(), check=False)
        print(f"{args.target} ({args.namespace}): stopped")
    elif args.status:
        print(status(args.target))
    else:
        up(args.target, args.namespace, config)


if __name__ == "__main__":
    sys.exit(main())
