# Changelog

This changelog records the major public milestones of the 6DOF Robot Arm ROS 2 project.

## 2026-09 — Public simulation baseline

### Added

- complete ROS 2 Humble workspace for the six-axis robot and gripper,
- Gazebo Classic workcell simulation,
- MoveIt 2 planning and RViz integration,
- ros2_control arm and gripper execution,
- custom Python/Tkinter operator interface,
- saved-pose and TCP configuration,
- industrial-style Sequence Controller with PTP/LIN selection, blending, I/O logic, gripper actions, STOP/RESUME, and waypoint behavior,
- runtime multi-seed IK and wrist-aware branch selection,
- automatic jig selection, pickup, transfer, placement, validation, inventory tracking, and full-table execution,
- stable Gazebo link-detach handling using update-thread deferred detach,
- native SolidWorks CAD archives for the robot, gripper, jigs, and workcell,
- neutral STEP exports for the complete workcell and its components,
- project setup, reproduction, continuation, troubleshooting, BOM, architecture, and engineering documentation,
- repository quality checks for Python syntax, shell syntax, package manifests, required project files, and generated-artifact leakage,
- helper scripts for dependency installation, workspace builds, environment verification, and complete startup.

### Validated

- public repository built successfully from its own `src/` tree on Ubuntu 22.04 / ROS 2 Humble,
- environment check completed with all reported checks passing on the development machine,
- Gazebo, MoveIt, RViz, the robot GUI, robot model, arm controller, and gripper controller launched successfully from the public repository.

### Documentation cleanup

- corrected the public workspace layout so the repository is cloned directly as `~/robot_arm_ws`,
- removed the unavailable optional `warehouse_ros_mongo` dependency from the supported workflow,
- documented both native SolidWorks and STEP CAD packages,
- aligned setup and reproduction instructions with the validated public build and launch process.

## Project status

The current public baseline is a **simulation-validated engineering prototype**. The physical six-axis arm was not fully commissioned during the project period because the selected integrated actuator modules had a long procurement lead time.

Future hardware work includes physical actuator integration, EtherCAT/CAN communication, robot calibration, camera-based target recognition, workplane calibration, machine-safety engineering, and complete cell commissioning.
