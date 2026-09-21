#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WS="${ROBOT_ARM_WS:-$REPO_ROOT}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# ROS environment setup scripts are not guaranteed to be nounset-safe.
# Source them before enabling `set -u`.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [[ ! -d "$WS/src" ]]; then
  echo "Workspace source directory not found: $WS/src" >&2
  exit 1
fi

cd "$WS"

echo "Building workspace: $WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install

cat <<EOF

Build completed successfully.
Source the workspace with:
  source "$WS/install/setup.bash"

Then run:
  bash "$WS/scripts/check_system.sh"

To build a different workspace explicitly, set ROBOT_ARM_WS first, for example:
  ROBOT_ARM_WS="$HOME/robot_arm_ws" bash "$SCRIPT_DIR/build_workspace.sh"
EOF
