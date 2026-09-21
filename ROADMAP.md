# Roadmap

The current repository is a **simulation-validated engineering prototype**. This roadmap separates demonstrated software capability from future physical-system work.

## Current validated baseline

- 6-DOF robot and custom gripper CAD
- torque sizing and actuator selection
- structural FEA and stiffness studies
- URDF / STL model generation
- Gazebo Classic 11 simulation
- MoveIt 2 + RViz planning and validation
- ros2_control arm and gripper execution
- manual Cartesian and joint jogging
- saved poses and TCP handling
- industrial-style sequence programming
- synchronized gripper actions
- digital I/O logic
- PTP / LIN sequence selection
- runtime IK with multi-seed branch selection
- dynamic target discovery
- automatic jig pickup and placement
- placement and inventory validation
- full-table automatic execution
- immediate STOP / RESUME
- native SolidWorks CAD archives
- neutral STEP workcell/component exports
- clean public build and launch validation on Ubuntu 22.04 / ROS 2 Humble

## Next engineering phase

### 1. Physical actuator integration

- install the selected integrated joint modules,
- define final mechanical mounting and cable routing,
- validate brake behavior,
- map real joint feedback and commands into ros2_control,
- implement repeatable startup and shutdown procedures.

Tracked in GitHub issue #1.

### 2. Industrial communication

- integrate EtherCAT and/or CAN interfaces supported by the selected actuators,
- define hardware abstraction and fault handling,
- validate command rate, feedback timing, and emergency-stop behavior.

### 3. Robot calibration

- establish joint-zero references,
- calibrate kinematic parameters,
- validate TCP position/orientation,
- characterize backlash, repeatability, and payload-dependent deflection.

### 4. Vision and automatic workplane setup

- select and calibrate a camera,
- detect workboard / harness-layout targets,
- estimate the workplane pose,
- transform detected targets into the robot frame,
- feed approved targets into the existing automatic-placement pipeline.

Tracked in GitHub issue #2.

### 5. Physical performance validation

- payload testing,
- structural-deflection measurements,
- repeatability testing,
- cycle-time measurement,
- long-duration reliability testing.

### 6. Industrial safety and commissioning

- electrical architecture,
- safety-rated stop design,
- guarding / collaborative-operation assessment,
- risk assessment,
- production commissioning.

## Reproduction quality

A clean-machine reproduction test remains useful for confirming that no implicit development-machine dependency is missing from the public instructions. See GitHub issue #3.

## Mechanical-source completeness

Native SolidWorks archives and neutral STEP exports are now included. Further additions should be limited to genuinely useful manufacturing drawings or updated revisions rather than historical CAD backups. See GitHub issue #4 for the original CAD-publication task.
