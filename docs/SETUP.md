# Setup Guide

This project was developed and validated with the following baseline:

```text
Ubuntu 22.04 LTS
ROS 2 Humble
Gazebo Classic 11
MoveIt 2
RViz 2
ros2_control
ros2_controllers
OMPL
Pilz Industrial Motion Planner
Python 3 / Tkinter
colcon / ament
```

## 1. Install ROS 2 Humble

Follow the official ROS 2 Humble Ubuntu installation instructions, then install the desktop package:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

Source ROS:

```bash
source /opt/ros/humble/setup.bash
```

## 2. Install development tools

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-tk \
  python3-yaml \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool
```

Initialize rosdep if needed:

```bash
sudo rosdep init
rosdep update
```

## 3. Install control and simulation packages

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control
```

## 4. Install MoveIt 2

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-planners-ompl \
  ros-humble-pilz-industrial-motion-planner
```

## 5. Clone the repository

```bash
mkdir -p ~/robot_arm_ws/src
cd ~/robot_arm_ws/src

git clone https://github.com/Yousef-Elbeltagy/6DOF-Robot-Arm-ROS2.git
```

If the repository is structured with the ROS packages directly under the repository root, move/copy those packages into `~/robot_arm_ws/src` or use the repository itself as the `src` content.

## 6. Install package dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_arm_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble
```

## 7. Build

```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/robot_arm_ws/install/setup.bash
```

## 8. Main project components

The original workspace includes packages for:

```text
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
```

## 9. Run the project

The original development system used a convenience startup script named:

```text
start_robot.sh
```

Once that script is included in the repository, it can be run with:

```bash
bash ~/start_robot.sh
```

Alternatively, the individual ROS 2 launch files can be started manually from the appropriate packages.

## 10. Useful checks

Controllers:

```bash
ros2 control list_controllers
```

MoveIt sequence action:

```bash
ros2 action info /sequence_move_group
```

Gripper transform:

```bash
timeout 3 ros2 run tf2_ros tf2_echo link_6 gripper_base
```

## Notes

- The project was developed on Gazebo Classic 11 rather than modern Gz Sim.
- The simulated jig attach/detach system uses a modified IFRA LinkAttacher implementation.
- The robot is a development/simulation project and not a certified industrial safety system.
