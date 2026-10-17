#!/usr/bin/env bash
# TurtleBot3 Burger SBC setup (ROS 2 Humble, Ubuntu 22.04), following the ROBOTIS e-manual
# "Quick Start Guide > SBC Setup". Safe to re-run: every step skips work already done.
# Nothing here starts the robot (no bringup, no motor commands).
#
# Usage on the Raspberry Pi:
#   bash tb3_sbc_setup.sh [LDS-01|LDS-02|LDS-03]   # install everything
#   bash tb3_sbc_setup.sh --check                   # report status only, no sudo
#
# Differences from the e-manual: ROS_DOMAIN_ID is 70 (matches the project's WSL side),
# TURTLEBOT3_MODEL=burger is added to ~/.bashrc, and all sudo steps run before the long build.
set -euo pipefail

DOMAIN_ID=70
WS=~/turtlebot3_ws

check() {
  echo "== hardware =="
  tr -d '\0' < /proc/device-tree/model; echo
  free -h | sed -n '1,3p'
  echo "== os =="
  lsb_release -ds 2>/dev/null || grep PRETTY /etc/os-release
  echo "== ros =="
  if [ -f /opt/ros/humble/setup.bash ]; then echo "ROS 2 Humble: installed"; else echo "ROS 2 Humble: MISSING"; fi
  if [ -f $WS/install/setup.bash ]; then
    set +u; source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; set -u
    echo "turtlebot3 packages:"; ros2 pkg list 2>/dev/null | grep -E 'turtlebot3|ld08|coin_d4|hls_lfcd' | sed 's/^/  /'
  else
    echo "turtlebot3_ws: NOT BUILT"
  fi
  echo "== ~/.bashrc =="
  grep -E 'ros/humble|turtlebot3_ws|ROS_DOMAIN_ID|TURTLEBOT3_MODEL|LDS_MODEL' ~/.bashrc || echo "(no ROS lines)"
  echo "== system =="
  [ -f /etc/udev/rules.d/99-turtlebot3-cdc.rules ] && echo "udev rule: ok" || echo "udev rule: MISSING"
  systemctl is-enabled systemd-networkd-wait-online.service 2>/dev/null | sed 's/^/wait-online: /' || true
  systemctl is-enabled sleep.target 2>/dev/null | sed 's/^/sleep.target: /' || true
  grep -h Periodic /etc/apt/apt.conf.d/20auto-upgrades 2>/dev/null || true
  echo "== devices =="
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "(no ttyACM/ttyUSB: OpenCR or LiDAR not plugged in?)"
  lsusb 2>/dev/null || true
}

if [ "${1:-}" = "--check" ]; then check; exit 0; fi

LDS="${1:-}"
case "$LDS" in ""|LDS-01|LDS-02|LDS-03) ;; *) echo "LDS model must be LDS-01, LDS-02 or LDS-03"; exit 1;; esac

# apt that never stops to ask questions and waits for the boot-time apt lock
apt_get() {
  sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a \
    apt-get -y -o DPkg::Lock::Timeout=900 "$@"
}
add_line() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }

echo ">>> sudo password (asked once, kept alive during setup)"
sudo -v
while true; do sudo -n true; sleep 60; kill -0 "$$" 2>/dev/null || exit; done 2>/dev/null &

echo ">>> [1/7] system settings"
printf 'APT::Periodic::Update-Package-Lists "0";\nAPT::Periodic::Unattended-Upgrade "0";\n' \
  | sudo tee /etc/apt/apt.conf.d/20auto-upgrades > /dev/null
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

echo ">>> [2/7] swap (only if RAM < 3.5 GB and no swap yet)"
mem_kb=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
if [ "$mem_kb" -lt 3500000 ] && [ -z "$(swapon --show)" ]; then
  sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
  sudo mkswap /swapfile && sudo swapon /swapfile
  grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
else
  echo "skip"
fi

echo ">>> [3/7] ROS 2 Humble (ros-base)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  apt_get update
  apt_get install software-properties-common curl locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository -y universe
  ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.jammy_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  apt_get update
  apt_get upgrade
  apt_get install ros-humble-ros-base ros-dev-tools
else
  echo "skip (already installed)"
fi

echo ">>> [4/7] TurtleBot3 dependencies"
apt_get install git python3-argcomplete python3-colcon-common-extensions libboost-system-dev \
  build-essential libudev-dev ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs \
  ros-humble-dynamixel-sdk ros-humble-xacro

echo ">>> [5/7] clone sources"
mkdir -p $WS/src && cd $WS/src
[ -d turtlebot3 ] || git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
[ -d ld08_driver ] || git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
[ -d coin_d4_driver ] || git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver
rm -rf turtlebot3/turtlebot3_cartographer turtlebot3/turtlebot3_navigation2

echo ">>> [6/7] OpenCR udev rule + ~/.bashrc"
sudo cp $WS/src/turtlebot3/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
add_line 'source /opt/ros/humble/setup.bash'
add_line 'source ~/turtlebot3_ws/install/setup.bash'
sed -i '/^export ROS_DOMAIN_ID=/d' ~/.bashrc
add_line "export ROS_DOMAIN_ID=${DOMAIN_ID} #TURTLEBOT3"
add_line 'export TURTLEBOT3_MODEL=burger'
if [ -n "$LDS" ]; then
  sed -i '/^export LDS_MODEL=/d' ~/.bashrc
  add_line "export LDS_MODEL=${LDS}"
fi

echo ">>> [7/7] colcon build (no sudo needed; may take over an hour, keep wall power on)"
cd $WS
set +u; source /opt/ros/humble/setup.bash; set -u
colcon build --symlink-install --parallel-workers 1

echo ">>> DONE"
check
[ -n "$LDS" ] || echo "NOTE: LDS_MODEL not set yet. Re-run with LDS-01 or LDS-02 once the LiDAR model is known."
