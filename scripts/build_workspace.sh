#!/usr/bin/env bash
set -euo pipefail

WS="${ROBOT_ARM_WS:-$HOME/robot_arm_ws}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ ! -d "$WS/src" ]]; then
  echo "Workspace source directory not found: $WS/src" >&2
  exit 1
fi

cd "$WS"

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install

cat <<EOF

Build completed successfully.
Source the workspace with:
  source "$WS/install/setup.bash"

Then run:
  bash "$WS/src/6DOF-Robot-Arm-ROS2/scripts/check_system.sh"
EOF
