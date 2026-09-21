# Project Gallery

All images below are **real project CAD, analysis, simulation, and GUI captures** from the development work. AI-generated robot artwork is intentionally excluded.

---

## 1. Mechanical CAD

### Complete 6-DOF robot model

![SolidWorks robot CAD](../assets/screenshots/cad_robot_overview.png)

### Structural / internal design view

![Transparent CAD structure](../assets/screenshots/cad_transparent_structure.png)

The mechanical design evolved through several iterations covering link geometry, joint packaging, manufacturability, actuator integration, and stiffness.

---

## 2. Structural analysis

![FEA stress result](../assets/screenshots/fea_stress_result.png)

FEA was used not only to check stress, but also to understand stiffness, local stress concentrations, and end-effector deflection.

---

## 3. CAD to URDF workflow

![CAD to URDF exporter](../assets/screenshots/cad_to_urdf_exporter.png)

The SolidWorks assembly was converted into a ROS-compatible robot model with link hierarchy, revolute joints, coordinate systems, inertial properties, STL meshes, and collision geometry.

---

## 4. Gazebo workcell simulation

![Gazebo robot and jig-placement workcell](../assets/screenshots/gazebo_robot_simulation.png)

The current Gazebo Classic 11 capture shows the complete simulated robot, custom gripper, source jig table, and reusable workboard used by the pick-and-place / jig-placement application.

---

## 5. MoveIt 2 and RViz

![MoveIt and RViz](../assets/screenshots/moveit_rviz_validation.png)

MoveIt 2 handled inverse kinematics, planning, collision checking, and trajectory generation while RViz was used to validate planning-scene and robot-state behavior.

---

## 6. Custom operator interface

![Main robot control GUI](../assets/screenshots/gui_manual_control.png)

The Python/Tkinter operator application combines:

- Cartesian X/Y/Z and Roll/Pitch/Yaw jogging,
- target motion and HOME,
- gripper open/close,
- six digital inputs,
- six digital outputs,
- saved robot poses,
- the Sequence Controller,
- Automatic Jig Placement,
- live application status.

---

# 7. Sequence

Sequence programming is a dedicated project feature, not just a small GUI utility. It lets the operator build ordered robot routines from saved poses and combine motion with process logic.

### Sequence entry point in the main GUI

![Main GUI with Sequence Controller entry point](../assets/screenshots/gui_manual_control.png)

The operator teaches or saves robot poses in the main application and then opens **SEQUENCE CONTROLLER** to assemble them into a robot program.

### Sequence Controller next to the simulated robot

![Sequence Controller with Gazebo](../assets/screenshots/gui_sequence_programming.png)

Each sequence row can define:

- a saved **Waypoint**,
- `STOP` or `CONTINUE` behavior,
- a **Blend Radius**,
- `PTP` or `LIN` motion,
- an input source `NONE` / `IN 1` … `IN 6`,
- an expected input state `TRUE` / `FALSE`,
- input timing `BEFORE` / `AFTER`,
- an output `NONE` / `OUT 1` … `OUT 6`,
- `OPEN GRIPPER` / `CLOSE GRIPPER`,
- output state `TRUE` / `FALSE`,
- output timing `BEFORE` / `AFTER`.

The sequence interface also provides **RUN SEQUENCE**, **STOP**, **RESUME**, waypoint removal, and sequence clearing.

A key implementation detail is the **gripper synchronization barrier**: arm motion stops at the relevant waypoint, the gripper trajectory is sent, the software waits for the real controller result, and only then does the next arm segment begin. This prevents asynchronous arm/gripper overlap.

The sequence work also drove fixes for mixed PTP/LIN execution, duplicate `move_group` processes, Pilz linear-planning constraints, stale gripper state, action-result handling, and true mid-motion STOP/RESUME behavior.

➡️ **[Read the full Sequence Programming & Execution documentation](SEQUENCE_PROGRAMMING.md)**

---

## 8. Automatic jig placement

![Automatic jig placement GUI](../assets/screenshots/automatic_jig_placement_gui.png)

The automatic-placement layer combines target discovery, jig selection, runtime IK, simulated gripping, attach/detach, occupied-target tracking, validation, full-table execution, and immediate STOP/RESUME.

---

## Project progression

```text
Mechanical CAD
    ↓
FEA / torque / actuator sizing
    ↓
URDF + meshes
    ↓
Gazebo workcell simulation
    ↓
MoveIt 2 + RViz
    ↓
Custom GUI
    ↓
Saved poses
    ↓
Sequence programming + I/O + gripper synchronization
    ↓
Automatic jig placement
    ↓
Full-table autonomous simulation
```

---

[← Back to main README](../README.md)
