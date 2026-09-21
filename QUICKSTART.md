# Quick Start

For the complete walkthrough, use [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md). This page is the shortest validated path from a clean Ubuntu 22.04 machine to the simulated robot.

## Supported baseline

```text
Ubuntu 22.04 LTS
ROS 2 Humble
Gazebo Classic 11
MoveIt 2
RViz 2
ros2_control / ros2_controllers
Python 3 / Tkinter
colcon / ament
```

## 1. Clone the repository as the workspace

The repository already contains its ROS packages under `src/`, so clone the repository itself as `~/robot_arm_ws`:

```bash
cd ~
git clone https://github.com/Yousef-Elbeltagy/6DOF-Robot-Arm-ROS2.git robot_arm_ws
cd ~/robot_arm_ws
```

Do **not** clone the whole repository into `~/robot_arm_ws/src`; that would create a nested `src/src` layout.

## 2. Install dependencies

After installing ROS 2 Humble, Gazebo Classic, MoveIt 2, ros2_control, and the development tools listed in [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md), run:

```bash
cd ~/robot_arm_ws
bash scripts/setup_dependencies.sh
```

## 3. Build

```bash
cd ~/robot_arm_ws
bash scripts/build_workspace.sh
```

A successful build should finish all project packages without errors.

## 4. Restore GUI configuration

The GUI reads the saved-pose and TCP configuration from the user's home directory:

```bash
cp ~/robot_arm_ws/config/robot_arm_saved_poses.json ~/robot_arm_saved_poses.json
cp ~/robot_arm_ws/config/robot_arm_tcp_config.json ~/robot_arm_tcp_config.json
```

## 5. Verify the environment

```bash
cd ~/robot_arm_ws
bash scripts/check_system.sh
```

The validated project machine reported all checks passing before the public launch test.

## 6. Start the complete simulation

```bash
cd ~/robot_arm_ws
bash scripts/start_robot.sh
```

The launcher brings up Gazebo Classic, MoveIt 2, RViz 2, the arm and gripper controllers, and the custom robot GUI.

## 7. First checks after launch

Verify that:

- the robot appears correctly in Gazebo and RViz,
- `/move_group` is running,
- the arm and gripper controllers are active,
- the GUI opens and reports a connected robot,
- small jogs and gripper commands execute correctly.

Then test a saved pose, a short Sequence Controller program, and the Automatic Jig Placement interface.

## Where to go next

- Full reproduction guide: [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md)
- Setup details: [docs/SETUP.md](docs/SETUP.md)
- Continue development: [docs/CONTINUE_DEVELOPMENT.md](docs/CONTINUE_DEVELOPMENT.md)
- Troubleshooting: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)
- Software architecture: [docs/SOFTWARE_ARCHITECTURE.md](docs/SOFTWARE_ARCHITECTURE.md)
- Sequence programming: [docs/SEQUENCE_PROGRAMMING.md](docs/SEQUENCE_PROGRAMMING.md)
- Automatic jig placement: [docs/AUTOMATIC_JIG_PLACEMENT.md](docs/AUTOMATIC_JIG_PLACEMENT.md)
- Native and neutral CAD: [cad/README.md](cad/README.md)
- Contributing: [CONTRIBUTING.md](CONTRIBUTING.md)

## Project status

This repository reproduces the **simulation-validated engineering prototype**. Physical actuator integration, calibration, machine safety, certification, and production commissioning remain future work.
