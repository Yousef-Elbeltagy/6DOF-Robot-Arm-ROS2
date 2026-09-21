#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source "$HOME/robot_arm_ws/install/setup.bash"

export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:$HOME/robot_arm_ws/install/robot_arm_with_gripper_urdf/share"

gnome-terminal --title="GAZEBO" -- bash -c '
source /opt/ros/humble/setup.bash
source "$HOME/robot_arm_ws/install/setup.bash"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:$HOME/robot_arm_ws/install/robot_arm_with_gripper_urdf/share"
ros2 launch robot_arm_with_gripper_urdf gazebo.launch.py 2>&1 | tee "$HOME/gazebo_crash_capture.log"
exec bash
'

echo "Waiting for Gazebo controllers..."

until ros2 action list 2>/dev/null \
    | grep -qx "/arm_controller/follow_joint_trajectory"
do
    sleep 2
done

until ros2 action list 2>/dev/null \
    | grep -qx "/gripper_controller/follow_joint_trajectory"
do
    sleep 2
done

echo "Gazebo and controllers are ready."

gnome-terminal --title="MOVE GROUP" -- bash -c '
source /opt/ros/humble/setup.bash
source "$HOME/robot_arm_ws/install/setup.bash"
ros2 launch robot_arm_moveit_config move_group.launch.py
exec bash
'

echo "Waiting for MoveIt..."

until ros2 node list 2>/dev/null | grep -qx "/move_group"
do
    sleep 2
done

sleep 3
echo "MoveIt is ready."

gnome-terminal --title="RVIZ" -- bash -c '
source /opt/ros/humble/setup.bash
source "$HOME/robot_arm_ws/install/setup.bash"
ros2 launch robot_arm_moveit_config moveit_rviz.launch.py
exec bash
'

sleep 5

gnome-terminal --title="ROBOT CONTROLLER" -- bash -c '
source /opt/ros/humble/setup.bash
source "$HOME/robot_arm_ws/install/setup.bash"
ros2 run robot_arm_python robot_gui 2>&1 | tee "$HOME/robot_gui_runtime.log"
exec bash
'
