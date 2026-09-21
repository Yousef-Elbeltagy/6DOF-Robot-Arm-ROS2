#!/usr/bin/env bash
set -u

ok=0
warn=0
fail=0

pass() { printf '[OK]   %s\n' "$1"; ok=$((ok+1)); }
warning() { printf '[WARN] %s\n' "$1"; warn=$((warn+1)); }
failure() { printf '[FAIL] %s\n' "$1"; fail=$((fail+1)); }

printf '\n6DOF Robot Arm - Environment Check\n'
printf '%s\n\n' '-----------------------------------'

if [ -f /etc/os-release ]; then
  . /etc/os-release
  if [ "${VERSION_ID:-}" = "22.04" ]; then
    pass "Ubuntu 22.04 detected"
  else
    warning "Validated baseline is Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}"
  fi
else
  warning "Could not determine operating system"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  pass "ROS 2 Humble installation found"
else
  failure "/opt/ros/humble/setup.bash not found"
fi

if command -v ros2 >/dev/null 2>&1; then
  pass "ros2 command available"
else
  warning "ros2 command not currently in PATH; source /opt/ros/humble/setup.bash"
fi

if command -v gazebo >/dev/null 2>&1 || command -v gzserver >/dev/null 2>&1; then
  pass "Gazebo Classic executable found"
else
  failure "Gazebo Classic executable not found"
fi

if command -v colcon >/dev/null 2>&1; then
  pass "colcon found"
else
  failure "colcon not found"
fi

if command -v python3 >/dev/null 2>&1; then
  pass "Python 3 found"
else
  failure "Python 3 not found"
fi

if python3 - <<'PY' >/dev/null 2>&1
import tkinter
import yaml
PY
then
  pass "Python Tkinter and YAML modules available"
else
  failure "Python Tkinter and/or YAML module missing"
fi

WS="${ROBOT_ARM_WS:-$HOME/robot_arm_ws}"

if [ -d "$WS/src" ]; then
  pass "Workspace source directory found: $WS/src"
else
  failure "Workspace source directory not found: $WS/src"
fi

if [ -f "$WS/install/setup.bash" ]; then
  pass "Built workspace setup found"
else
  warning "Workspace has not been built yet or install/setup.bash is missing"
fi

required_packages=(
  robot_arm_with_gripper_urdf
  robot_arm_moveit_config
  robot_arm_python
  pymoveit2
  table_harness_urdf_v3
  pick_table_urdf
  large_jig_urdf
  medium_jig_urdf
  small_jig_urdf
  IFRA_LinkAttacher
)

for pkg in "${required_packages[@]}"; do
  if [ -d "$WS/src/$pkg" ]; then
    pass "Package present: $pkg"
  else
    failure "Missing package: $pkg"
  fi
done

if [ -f "$HOME/robot_arm_saved_poses.json" ]; then
  pass "Saved-pose configuration found in home directory"
elif [ -f "$WS/config/robot_arm_saved_poses.json" ]; then
  warning "Saved poses exist in repository but not in home directory"
else
  failure "Saved-pose configuration not found"
fi

if [ -f "$HOME/robot_arm_tcp_config.json" ]; then
  pass "TCP configuration found in home directory"
elif [ -f "$WS/config/robot_arm_tcp_config.json" ]; then
  warning "TCP config exists in repository but not in home directory"
else
  failure "TCP configuration not found"
fi

if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1; then
    pass "MoveIt 2 package available"
  else
    failure "MoveIt 2 package not found"
  fi

  if ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
    pass "gazebo_ros package available"
  else
    failure "gazebo_ros package not found"
  fi

  if ros2 pkg prefix controller_manager >/dev/null 2>&1; then
    pass "ros2_control controller_manager available"
  else
    failure "ros2_control controller_manager not found"
  fi
fi

printf '\nSummary: %d OK, %d warnings, %d failures\n' "$ok" "$warn" "$fail"

if [ "$fail" -gt 0 ]; then
  printf 'Fix failures before attempting a full project launch.\n'
  exit 1
fi

printf 'Environment is ready for the next reproduction step.\n'
exit 0
