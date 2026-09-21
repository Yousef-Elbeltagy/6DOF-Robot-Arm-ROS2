# Reproduce the Project

This guide is for someone who wants to clone the repository, build the ROS 2 workspace, launch the simulation, and reach the same software baseline used during development.

## Supported baseline

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic 11
- MoveIt 2
- RViz 2
- ros2_control / ros2_controllers
- OMPL
- Pilz Industrial Motion Planner
- Python 3 + Tkinter
- colcon / ament

The project was developed against Gazebo Classic rather than modern Gz Sim.

## 1. Install ROS 2 and development tools

Follow the official ROS 2 Humble installation procedure for Ubuntu 22.04, then install the main packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-tk \
  python3-yaml \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-moveit \
  ros-humble-moveit-planners-ompl \
  ros-humble-pilz-industrial-motion-planner
```

Initialize rosdep if needed:

```bash
sudo rosdep init
rosdep update
```

## 2. Create the workspace

```bash
mkdir -p ~/robot_arm_ws
cd ~/robot_arm_ws
git clone https://github.com/Yousef-Elbeltagy/6DOF-Robot-Arm-ROS2.git src
```

The repository is already organized so that its ROS packages live under `src/`.

## 3. Install package dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_arm_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 4. Build

```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Restore runtime configuration

The GUI expects these files in the user's home directory:

```text
~/robot_arm_saved_poses.json
~/robot_arm_tcp_config.json
```

Copy the repository versions:

```bash
cp ~/robot_arm_ws/config/robot_arm_saved_poses.json ~/robot_arm_saved_poses.json
cp ~/robot_arm_ws/config/robot_arm_tcp_config.json ~/robot_arm_tcp_config.json
```

The validated TCP translation is approximately:

```text
[0, 0, 0.4193463143]
```

Do not replace that value with the shorter `link_6 -> gripper_base` offset; they represent different things.

## 6. Check the environment

Run:

```bash
bash ~/robot_arm_ws/scripts/check_system.sh
```

Fix any reported missing dependency before launching the full system.

## 7. Launch the project

The convenience launcher starts Gazebo, waits for both arm and gripper controllers, starts MoveIt, opens RViz, then launches the Tkinter GUI:

```bash
bash ~/robot_arm_ws/scripts/start_robot.sh
```

Expected major components:

```text
Gazebo Classic
MoveIt / move_group
RViz 2
arm_controller
gripper_controller
Robot GUI
```

## 8. Verify a successful launch

Controllers:

```bash
ros2 control list_controllers
```

Expected active controllers include the six-axis arm trajectory controller and the gripper trajectory controller.

MoveIt:

```bash
ros2 node list | grep move_group
```

There should normally be one active `/move_group` instance.

Sequence action:

```bash
ros2 action info /sequence_move_group
```

Arm trajectory action:

```bash
ros2 action info /arm_controller/follow_joint_trajectory
```

Gripper trajectory action:

```bash
ros2 action info /gripper_controller/follow_joint_trajectory
```

## 9. First functional test

Use the GUI in this order:

1. Confirm the status reports that the robot is connected.
2. Test a very small Cartesian jog.
3. Test gripper open / close.
4. Load one saved pose and move to it.
5. Open the Sequence Controller and run a short two-waypoint sequence.
6. Open Automatic Jig Placement, search for a free target, scan for a matching jig, and validate the displayed target/jig information before starting motion.

## 10. Repository packages

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

## 11. What is reproduced

This repository reproduces the simulation and control prototype, including:

- six-axis robot + gripper URDF,
- Gazebo workcell,
- MoveIt planning,
- ros2_control execution,
- manual GUI control,
- saved poses,
- sequence programming,
- digital I/O logic,
- runtime IK,
- automatic jig selection and placement,
- simulated attach/detach,
- full-table automation logic.

It does not reproduce a commissioned physical industrial robot. Hardware integration, calibration, safety engineering, and certification remain future work.

## 12. If something fails

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) before changing coordinates, IK logic, controller timing, or attach/detach behavior.
