# Project Gallery

All images below are **real project screenshots/CAD/simulation captures** from the development work. AI-generated robot imagery is intentionally excluded.

---

## 1. Mechanical CAD

### Complete 6-DOF robot model

![SolidWorks robot CAD](../assets/screenshots/cad_robot_overview.png)

### Structural / internal design view

![Transparent CAD structure](../assets/screenshots/cad_transparent_structure.png)

The mechanical design evolved through several iterations covering link geometry, joint packaging, manufacturability and stiffness.

---

## 2. Structural analysis

![FEA stress result](../assets/screenshots/fea_stress_result.png)

FEA was used not only to check stress, but also to understand stiffness, local stress concentrations and end-effector deflection.

---

## 3. CAD to URDF workflow

![CAD to URDF exporter](../assets/screenshots/cad_to_urdf_exporter.png)

The SolidWorks assembly was converted into a ROS-compatible robot model with link hierarchy, revolute joints, coordinate systems, inertial properties, STL meshes and collision geometry.

---

## 4. Gazebo simulation

![Gazebo robot simulation](../assets/screenshots/gazebo_robot_simulation.png)

The complete arm and gripper were validated in Gazebo Classic 11 before adding the larger workcell and autonomous jig-placement behavior.

---

## 5. MoveIt 2 and RViz

![MoveIt and RViz](../assets/screenshots/moveit_rviz_validation.png)

MoveIt 2 handled inverse kinematics, planning, collision checking and trajectory generation while RViz was used to validate planning-scene and robot-state behavior.

---

## 6. Custom operator interface

![Manual robot control GUI](../assets/screenshots/gui_manual_control.png)

The Python/Tkinter interface grew into a complete operator application with Cartesian and joint jogging, speed override, jog frames, TCP settings, saved poses, gripper control and automation tools.

---

## 7. Sequence programming

![Sequence programming GUI](../assets/screenshots/gui_sequence_programming.png)

The sequence controller supports saved waypoints, STOP/CONTINUE behavior, blend radius, PTP/LIN selection, digital I/O conditions and gripper actions.

---

## 8. Automatic jig placement

![Automatic jig placement GUI](../assets/screenshots/automatic_jig_placement_gui.png)

The automatic-placement interface brings together target discovery, jig selection, runtime IK, simulated gripping, occupied-target tracking, full-table execution and immediate STOP/RESUME.

---

## Project progression

```text
Mechanical CAD
    ↓
FEA / torque / actuator sizing
    ↓
URDF + meshes
    ↓
Gazebo simulation
    ↓
MoveIt 2 + RViz
    ↓
Custom GUI
    ↓
Sequence controller
    ↓
Automatic jig placement
    ↓
Full-table autonomous simulation
```

---

[← Back to main README](../README.md)
