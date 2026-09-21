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

The repository itself is the colcon workspace and already contains a `src/` directory:

```bash
cd ~
git clone https://github.com/Yousef-Elbeltagy/6DOF-Robot-Arm-ROS2.git robot_arm_ws
cd ~/robot_arm_ws
```

Expected layout:

```text
~/robot_arm_ws/
├── src/
├── scripts/
├── config/
├── docs/
└── README.md
```

Do not clone the complete repository into `~/robot_arm_ws/src`.

## 6. Install package dependencies

Recommended:

```bash
cd ~/robot_arm_ws
bash scripts/setup_dependencies.sh
```

Or manually:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_arm_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 7. Build

Recommended:

```bash
cd ~/robot_arm_ws
bash scripts/build_workspace.sh
```

Or manually:

```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/robot_arm_ws/install/setup.bash
```

## 8. Restore GUI configuration

```bash
cp ~/robot_arm_ws/config/robot_arm_saved_poses.json ~/robot_arm_saved_poses.json
cp ~/robot_arm_ws/config/robot_arm_tcp_config.json ~/robot_arm_tcp_config.json
```

## 9. Verify the environment

```bash
cd ~/robot_arm_ws
bash scripts/check_system.sh
```

The public repository was validated on the development machine with all environment checks passing.

## 10. Run the project

```bash
cd ~/robot_arm_ws
bash scripts/start_robot.sh
```

The launcher starts Gazebo Classic, waits for the arm and gripper controllers, starts MoveIt, opens RViz, and launches the custom Tkinter GUI.

The public repository was build-tested and launch-tested after publication.

## 11. Main project components

Important packages include:

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

## 12. Useful checks

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
- `warehouse_ros_mongo` is not required for the supported planning/simulation workflow.
- Native SolidWorks and neutral STEP CAD packages are available under `cad/`.
- The robot is a development/simulation project and not a certified industrial safety system.
