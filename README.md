# 6DOF Robot Arm — ROS 2

A custom **6-DOF robotic arm** designed in SolidWorks and developed as a robotics/mechatronics internship project, with a complete simulation and control stack built around **ROS 2 Humble**, **Gazebo Classic 11**, **MoveIt 2**, **RViz 2**, and a custom **Python/Tkinter GUI**.

The project focuses on flexible robotic manipulation for a **wire-harness jig-placement application**, combining mechanical design, robot modeling, motion planning, runtime inverse kinematics, gripper control, sequence execution, and automatic pick-and-place behavior.

## Highlights

- 6-DOF articulated robot arm
- SolidWorks mechanical design and CAD-to-URDF workflow
- ROS 2 Humble integration
- Gazebo Classic 11 simulation
- MoveIt 2 motion planning and inverse kinematics
- RViz 2 visualization
- ros2_control trajectory execution
- Custom Python/Tkinter operator GUI
- Joint and Cartesian jogging
- WORLD / FLANGE / TOOL jogging frames
- Configurable TCP
- Saved robot poses
- PTP / LIN sequence programming
- Digital I/O logic
- Gripper trajectory control
- Automatic jig detection and placement
- Runtime IK with closest-solution selection
- Dynamic target discovery from TF
- Automatic full-table execution
- Immediate STOP / RESUME behavior
- Gazebo link attachment/detachment for simulated grasping

## Project Motivation

Traditional wire-harness assembly boards are often product-specific, which makes changeovers expensive, space-intensive, and inflexible.

This project explores a reusable workboard concept in which a robot automatically places modular jigs at required locations. The long-term idea is to combine the robot with machine vision so that new harness layouts can be configured with far less manual rework.

## System Architecture

```text
SolidWorks CAD
      |
      v
URDF + STL meshes
      |
      +---------------------------+
      |                           |
      v                           v
Gazebo Classic               MoveIt 2
Physics / workcell            Planning / IK
      |                           |
      +-------------+-------------+
                    |
                    v
                ROS 2 Humble
                    |
             ros2_control
                    |
                    v
         Python / Tkinter GUI
                    |
        +-----------+-----------+
        |           |           |
        v           v           v
     Jogging     Sequences   Automatic Jig
                              Placement
```

## Software Stack

| Component | Version / Tool |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Simulator | Gazebo Classic 11 |
| Motion Planning | MoveIt 2 |
| Visualization | RViz 2 |
| Control | ros2_control / ros2_controllers |
| Planning | OMPL + Pilz Industrial Motion Planner |
| GUI | Python 3 + Tkinter |
| CAD | SolidWorks |
| Build System | colcon / ament |

## Robot Design

The robot was designed as a 6-axis articulated arm with an intended reach of approximately **1.5 m** and a target payload of approximately **10 kg**.

The mechanical design went through several iterations involving:

- hollow aluminum links
- structural FEA
- stiffness and stress studies
- custom cycloidal reducer research
- planetary/belt reduction concepts
- final selection of integrated robotic joint modules

The final mechanical direction uses **6061-T6 aluminum** for the main lightweight structure and integrated servo-reducer joint modules for the six robot axes.

## Final Actuator Concept

| Joint | Module | Rated Torque | Peak Torque |
|---|---|---:|---:|
| J1–J2 | TD-110-170 | 328 N·m | 702 N·m |
| J3 | TD-100-142 | 169 N·m | 411 N·m |
| J4–J6 | TD-70-90 | 50 N·m | 102 N·m |

The calculated dynamic peak torque at J2 was approximately **411.9 N·m**, making it the governing joint for actuator sizing.

## ROS 2 Workspace

The project workspace contains packages for:

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

## Custom GUI

The custom operator interface includes:

- X / Y / Z Cartesian jog
- Roll / Pitch / Yaw jog
- J1–J6 joint jog
- speed override
- WORLD / FLANGE / TOOL frames
- TCP configuration
- saved poses
- sequence controller
- PTP / LIN motion selection
- STOP / CONTINUE waypoint behavior
- blend radius
- digital I/O
- gripper actions
- automatic jig placement
- full-table automatic execution
- immediate STOP / RESUME

## Automatic Jig Placement

The automatic placement workflow is approximately:

```text
Discover free target frames
        ↓
Infer required jig size
        ↓
Scan for a matching unused jig
        ↓
Compute runtime IK
        ↓
Move above pickup
        ↓
Descend and close gripper
        ↓
Attach jig in Gazebo
        ↓
Lift and move to target
        ↓
Descend and release
        ↓
Detach jig
        ↓
Validate placement
        ↓
Mark jig and target as used
        ↓
Continue to next target
```

Placement targets are discovered dynamically from TF frames ending in:

```text
_target_link
```

Current example target IDs include:

```text
L1
M1
M2
S1
```

where the prefix indicates the required jig size.

## Runtime Inverse Kinematics

The automatic placement system uses MoveIt's IK service and evaluates multiple seed configurations. When several valid IK solutions exist, the system selects a solution that minimizes wrapped joint motion, with additional weighting on wrist joints to reduce unnecessary wrist flips.

This approach was used to eliminate large, visually undesirable wrist rotations while keeping the implementation generic rather than hard-coding target-specific joint solutions.

## Gripper

The gripper is integrated into the robot URDF and controlled through a ROS 2 `FollowJointTrajectory` controller.

The main gripper joint is:

```text
left_gear_joint
```

with the remaining gripper joints linked mechanically through mimic relationships.

## Simulation Grasping

A Gazebo LinkAttacher plugin is used to simulate picking up and releasing jigs.

A stability issue was found during detach operations, where calling `Joint::Detach()` directly from a ROS callback could crash `gzserver`. The implementation was modified so that detach requests are queued and the actual detach operation runs from Gazebo's update thread.

That change significantly improved simulation stability during repeated automatic placements.

## Current Status

The project reached a simulation-validated stage with:

- full robot and gripper model
- Gazebo workcell
- MoveIt planning
- custom operator GUI
- automatic jig placement
- dynamic target discovery
- runtime IK
- placed-jig inventory tracking
- full-table automatic execution
- STOP / RESUME

The project was not physically commissioned as a complete industrial robot cell; the repository represents the CAD, robotics software, simulation, and control development completed during the project.

## Roadmap

Possible future work includes:

- physical actuator integration
- EtherCAT / CAN communication
- real robot hardware interface
- camera-based target detection
- automatic workboard calibration
- TCP calibration tools
- hand-guided teaching
- collision/safety validation
- industrial safety architecture
- custom robot programming language / teach-pendant workflow

## Notes

This repository is intended as an educational and portfolio reference for robotics, ROS 2, simulation, motion planning, and mechatronics development.

The project is a development prototype and should not be treated as a certified industrial safety system.

## Author

**Yousef El-Beltagy**

Robotics / Mechatronics Project
