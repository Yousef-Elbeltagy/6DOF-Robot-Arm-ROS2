#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
fi

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-tk \
  python3-yaml \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool \
  gazebo \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-moveit-planners-ompl \
  ros-humble-pilz-industrial-motion-planner

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep is unavailable after installation." >&2
  exit 1
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo
cat <<'EOF'
Base dependencies installed.

Next:
  cd ~/robot_arm_ws
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
  colcon build --symlink-install
EOF
