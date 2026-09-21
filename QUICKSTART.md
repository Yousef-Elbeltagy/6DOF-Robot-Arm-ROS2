# Quick Start

For the complete walkthrough, use [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md). This page is the shortest path from a clean Ubuntu 22.04 machine to the simulated robot.

## 1. Install the baseline

Required platform:

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Classic 11
MoveIt 2
ros2_control
Python 3 / Tkinter
```

Install the packages listed in [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md).

## 2. Clone as the workspace source tree

```bash
mkdir -p ~/robot_arm_ws
cd ~/robot_arm_ws
git clone https://github.com/Yousef-Elbeltagy/6DOF-Robot-Arm-ROS2.git src
```

## 3. Install dependencies and build

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_arm_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## 4. Restore GUI configuration

```bash
cp ~/robot_arm_ws/config/robot_arm_saved_poses.json ~/robot_arm_saved_poses.json
cp ~/robot_arm_ws/config/robot_arm_tcp_config.json ~/robot_arm_tcp_config.json
```

## 5. Verify the environment

```bash
bash ~/robot_arm_ws/scripts/check_system.sh
```

## 6. Start the complete simulation

```bash
bash ~/robot_arm_ws/scripts/start_robot.sh
```

The launcher brings up Gazebo, MoveIt, RViz, the arm/gripper controllers, and the custom robot GUI.

## Where to go next

- Reproduce everything: [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md)
- Continue development: [docs/CONTINUE_DEVELOPMENT.md](docs/CONTINUE_DEVELOPMENT.md)
- Troubleshooting: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)
- Software architecture: [docs/SOFTWARE_ARCHITECTURE.md](docs/SOFTWARE_ARCHITECTURE.md)
- Sequence programming: [docs/SEQUENCE_PROGRAMMING.md](docs/SEQUENCE_PROGRAMMING.md)
- Automatic jig placement: [docs/AUTOMATIC_JIG_PLACEMENT.md](docs/AUTOMATIC_JIG_PLACEMENT.md)
- Contributing: [CONTRIBUTING.md](CONTRIBUTING.md)

## Project status

This repository reproduces the simulation-validated engineering prototype. Physical actuator integration, calibration, machine safety, and production commissioning remain future work.
