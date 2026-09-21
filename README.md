<div align="center">

# 6DOF Robot Arm — ROS 2

### Mechanical design → digital twin → motion planning → sequence programming → autonomous jig placement

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-F58113)](https://classic.gazebosim.org/)
[![MoveIt](https://img.shields.io/badge/MoveIt-2-2D9CDB)](https://moveit.picknik.ai/)
[![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)](https://www.python.org/)
![Status](https://img.shields.io/badge/Status-Simulation--Validated-success)

**A 6-axis robot arm developed from SolidWorks CAD into a complete ROS 2 simulation, programming and control system for automated wire-harness jig placement.**

</div>

---

## Project at a glance

| | |
|---|---|
| **Robot** | 6-DOF articulated arm + custom gripper |
| **Design target** | ~1.5 m reach, ~10 kg payload |
| **Mechanical CAD** | SolidWorks |
| **Robot model** | URDF + STL meshes |
| **Robotics stack** | ROS 2 Humble, MoveIt 2, RViz 2, ros2_control |
| **Simulation** | Gazebo Classic 11 |
| **Control application** | Custom Python/Tkinter GUI |
| **Programming** | Saved poses, multi-waypoint sequences, PTP/LIN, blending, digital I/O, gripper actions |
| **Autonomy** | Runtime IK, dynamic TF targets, full-table jig placement |
| **Development context** | Robotics / Mechatronics internship project |

> The final system is a **simulation-validated engineering prototype**. The physical six-axis arm was not fully commissioned during the project period because the selected integrated joint modules had a long procurement lead time.

---

## Why this project exists

Wire-harness assembly can depend on dedicated boards and fixed jig layouts for each product or variant. That creates practical problems:

- slow product changeovers,
- storage overhead from dedicated boards,
- limited flexibility when new layouts are introduced.

The concept explored here is a reusable workboard with modular jigs. Instead of rebuilding the board, a robot selects the required jig from a source table and places it at the correct target location automatically.

The long-term vision is a flexible cell where a camera reads a harness layout, identifies target positions, and the robot configures the board automatically.

---

## From CAD to autonomous robot

```text
Mechanical design in SolidWorks
              ↓
     FEA / torque / actuators
              ↓
        URDF + STL meshes
              ↓
     ROS 2 Humble robot model
              ↓
   ┌──────────┴──────────┐
   ↓                     ↓
Gazebo Classic        MoveIt 2
physics/workcell      IK/planning
   └──────────┬──────────┘
              ↓
         ros2_control
              ↓
      Python/Tkinter GUI
              ↓
 manual control + saved poses
              ↓
      sequence programming
              ↓
   automatic jig placement
```

---

## What I built

### Mechanical engineering

- 6-axis articulated robot architecture
- hollow aluminum link design
- actuator packaging and joint housings
- structural FEA and stiffness studies
- torque model for all six axes
- custom cycloidal reducer research
- planetary/belt reduction studies
- final integrated actuator selection
- complete gripper integration

### Robot modeling & simulation

- SolidWorks-to-URDF workflow
- joint axes and coordinate-system definition
- mass, inertia, visual and collision properties
- Gazebo robot + workcell simulation
- MoveIt planning configuration
- RViz planning-scene validation
- ros2_control trajectory execution

### Custom robot controller

- Cartesian X/Y/Z jogging
- Roll/Pitch/Yaw jogging
- J1-J6 joint jogging
- 10/25/50/75/100% speed override
- WORLD / FLANGE / TOOL jog frames
- configurable Tool Center Point
- saved robot poses
- gripper open/close control

### Sequence programming

- ordered saved-waypoint programs
- `STOP` / `CONTINUE` waypoint behavior
- blend radius
- `PTP` / `LIN` motion selection
- digital input conditions
- digital output actions
- `BEFORE` / `AFTER` I/O timing
- synchronized gripper commands
- sequence STOP / RESUME controls
- MoveIt action and ros2_control execution handling

### Autonomous workcell

- dynamic target discovery from TF
- automatic jig-type inference
- matching-jig scan
- runtime inverse kinematics
- multi-seed IK branch selection
- simulated attach/detach
- used-jig inventory tracking
- occupied-target tracking
- placement validation
- full-table automatic run
- immediate STOP / RESUME

---

# Sequence

The **Sequence Controller** is a dedicated robot-programming layer and one of the most important software features in this project. It is where the system moved from manual jogging and saved poses to an **ordered, programmable robotic process**.

### Main Operator Interface

<p align="center">
  <img src="assets/screenshots/gui_manual_control.png" alt="Main Robot Control GUI" width="38%">
</p>

The main interface provides Cartesian/joint control, gripper commands, six digital inputs, six digital outputs, saved poses, the Sequence Controller, Automatic Jig Placement, and live application status.

### Sequence Controller

<p align="center">
  <img src="assets/screenshots/gui_sequence_programming.png" alt="Sequence Controller" width="72%">
</p>

The sequence table can program motion and process logic **row by row**:

| Sequence field | Options / behavior |
|---|---|
| **Waypoint** | Any saved robot pose |
| **Behavior** | `STOP` / `CONTINUE` |
| **Blend Radius** | Configurable transition radius, e.g. `0.001 m` |
| **Motion** | `PTP` / `LIN` |
| **Input Source** | `NONE`, `IN 1` … `IN 6` |
| **Input State** | `TRUE` / `FALSE` |
| **Input Timing** | `BEFORE` / `AFTER` |
| **Output Action** | `NONE`, `OUT 1` … `OUT 6`, `OPEN GRIPPER`, `CLOSE GRIPPER` |
| **Output State** | `TRUE` / `FALSE` where applicable |
| **Output Timing** | `BEFORE` / `AFTER` |

The operator can therefore express logic such as:

```text
IF IN 1 == TRUE BEFORE PICK → move → CLOSE GRIPPER AFTER
IF IN 2 == FALSE BEFORE MID → set OUT 1 = TRUE BEFORE
IF IN 3 == TRUE AFTER PLACE → OPEN GRIPPER AFTER
HOME → final safe stop
```

### STOP / CONTINUE and blending

`STOP` forces a precise stop at a waypoint, useful for gripping, releasing, I/O, synchronization, and inspection. `CONTINUE` allows eligible waypoints to be traversed with the configured blend radius for smoother multi-point motion. The final waypoint always stops safely.

### PTP / LIN

`PTP` performs point-to-point joint-space motion and became the most thoroughly validated sequence mode. `LIN` requests linear Cartesian tool motion and was developed through the Pilz planning pipeline; it is more sensitive to IK, acceleration limits, and start-state constraints.

### BEFORE / AFTER, TRUE / FALSE, inputs and outputs

Each program step can check one of six digital inputs for a required `TRUE` or `FALSE` state either `BEFORE` or `AFTER` the motion. It can also command one of six digital outputs, or issue `OPEN GRIPPER` / `CLOSE GRIPPER`, with `BEFORE` / `AFTER` timing.

This makes the sequence system a small industrial-style robot/process programming environment rather than only a list of poses.

### Gripper synchronization barrier

The arm is not allowed to continue while the gripper is still moving:

```text
arm motion segment
      ↓
stop at gripper waypoint
      ↓
send gripper FollowJointTrajectory goal
      ↓
wait for the real controller result
      ↓
verify completion
      ↓
continue next arm segment
```

The generic output path avoids issuing duplicate gripper commands when the dedicated gripper barrier is active.

### RUN / STOP / RESUME

The Sequence Controller supports **RUN SEQUENCE**, **STOP**, **RESUME**, waypoint removal, and sequence clearing. STOP cancels the active MoveIt sequence goal and the arm-controller trajectory, allowing a genuine mid-motion stop; RESUME retries the interrupted step from the robot's current state.

### Execution path

```text
Saved poses
    ↓
Sequence Controller GUI
    ↓
Waypoint + STOP/CONTINUE + blend + PTP/LIN
    ↓
Input conditions + outputs + gripper actions
    ↓
MoveIt 2 / Pilz
    ↓
/sequence_move_group
    ↓
/arm_controller/follow_joint_trajectory
    ↓
ros2_control
    ↓
Gazebo robot
```

The sequence work also exposed and drove fixes for motion types being unintentionally forced to PTP, an intermediate segmentation bug, duplicate `move_group` processes, LIN/Pilz start-state constraints, stale gripper feedback, asynchronous action-result handling, and gripper synchronization.

➡️ **[Read the full Sequence Programming & Execution documentation](docs/SEQUENCE_PROGRAMMING.md)**

---

## Real project gallery

The repository gallery uses **real project CAD, simulation and GUI screenshots**. AI-generated robot artwork is intentionally excluded.

### SolidWorks robot model

![SolidWorks robot CAD](assets/screenshots/cad_robot_overview.png)

### Gazebo simulation and MoveIt/RViz validation

<table>
<tr>
<td width="50%"><img src="assets/screenshots/gazebo_robot_simulation.png" alt="Gazebo simulation"></td>
<td width="50%"><img src="assets/screenshots/moveit_rviz_validation.png" alt="MoveIt RViz"></td>
</tr>
<tr>
<td align="center"><b>Gazebo Classic 11</b></td>
<td align="center"><b>MoveIt 2 + RViz 2</b></td>
</tr>
</table>

### Custom operator interface

<table>
  <tr>
    <td align="center" width="50%">
      <img src="assets/screenshots/gui_manual_control.png" alt="Main Operator Interface" width="430"><br>
      <b>Main Operator Interface</b>
    </td>
    <td align="center" width="50%">
      <img src="assets/screenshots/gui_sequence_programming.png" alt="Sequence Controller" width="430"><br>
      <b>Sequence Controller</b>
    </td>
  </tr>
</table>

The main interface provides Cartesian/joint control, TCP display and target entry, gripper commands, six digital inputs, six digital outputs, saved poses, access to the Sequence Controller, Automatic Jig Placement, and live application status.

### Sequence Controller

The Sequence Controller allows the operator to build motion logic row by row. Each row can define:

- **Waypoint** — any saved robot pose  
- **Behavior** — stop at a waypoint or continue through it  
- **Blend radius** — smoothing for continuous motion  
- **Motion type** — `PTP` / `LIN`  
- **Conditional logic** — `IF IN x = TRUE/FALSE`  
- **Condition timing** — evaluate conditions **BEFORE** or **AFTER** a waypoint  
- **Output actions** — activate or deactivate digital outputs  
- **Gripper actions** — open or close the gripper  
- **Execution flow** — run, stop, resume, clear, or remove waypoints  

➡️ **[Open the full project gallery](docs/GALLERY.md)**

---

## Mechanical design

The robot was designed around a target reach of approximately **1.5 m** and a target payload of approximately **10 kg**.

The main moving structure uses **6061-T6 aluminum** to keep link mass low while retaining practical machinability and strength.

The design evolved through:

1. cylindrical/tapered hollow links,
2. internal-rib concepts,
3. manufacturability review,
4. unribbed shell design with local reinforcement,
5. custom gearbox studies,
6. integrated robot-joint modules.

### Dynamic joint torque results

| Joint | Peak torque |
|---|---:|
| J1 | 115.58 N·m |
| **J2** | **411.90 N·m** |
| J3 | 182.49 N·m |
| J4 | 35.64 N·m |
| J5 | 15.50 N·m |
| J6 | 5.10 N·m |

J2 became the governing axis because it carries the downstream links, wrist, gripper and payload.

### Final actuator concept

| Joint(s) | Module | Rated torque | Peak/start-stop torque |
|---|---|---:|---:|
| J1-J2 | TD-110-170 | 328 N·m | 702 N·m |
| J3 | TD-100-142 | 169 N·m | 411 N·m |
| J4-J6 | TD-70-90 | 50 N·m | 102 N·m |

➡️ **[Mechanical design details](docs/MECHANICAL_DESIGN.md)**

---

## CAD → URDF → ROS 2

The SolidWorks assembly was prepared with explicit reference coordinate systems and revolute-joint axes before export.

The digital model includes:

- parent/child link hierarchy,
- joint origins and axes,
- motion limits,
- link mass and center of mass,
- inertia tensors,
- visual meshes,
- collision meshes,
- gripper mimic relationships.

![CAD to URDF exporter](assets/screenshots/cad_to_urdf_exporter.png)

---

## Software stack

| Layer | Technology |
|---|---|
| Operating system | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble |
| Physics simulation | Gazebo Classic 11 |
| Motion planning | MoveIt 2 |
| Planning pipelines | OMPL + Pilz |
| Visualization | RViz 2 |
| Control | ros2_control + ros2_controllers |
| GUI | Python 3 + Tkinter |
| Build | colcon / ament |

➡️ **[Software architecture](docs/SOFTWARE_ARCHITECTURE.md)**

---

## Automatic Jig Placement

The **Automatic Jig Placement** system is the highest-level application layer in the project. It combines workcell-state discovery, jig selection, runtime IK, robot motion, simulated grasping, placement validation, and repeatable full-table execution.

<p align="center">
  <img src="assets/screenshots/automatic_jig_placement_gui.png" alt="Automatic Jig Placement Interface" width="58%">
</p>

### Target discovery & jig matching

The interface discovers free target frames at runtime, identifies the required jig family from the target name, and searches for an unused matching jig.

```text
L1 → LARGE jig
M1 → MEDIUM jig
M2 → MEDIUM jig
S1 → SMALL jig
```

Targets are discovered from TF frame names ending in `_target_link` rather than relying only on a fixed coordinate list. The GUI then exposes the selected target, required jig type, selected jig model, jig coordinates, target-center coordinates, and TCP distance before enabling automatic execution.

### Runtime IK & branch selection

The controller does not depend on one prerecorded joint pose for every target. For pickup and placement poses it computes IK at runtime and evaluates several candidate branches:

```text
Target TCP pose
      ↓
Generate multiple IK seeds
      ↓
Call MoveIt IK
      ↓
Collect valid joint solutions
      ↓
Compare wrapped joint deltas
      ↓
Penalize excessive wrist motion
      ↓
Choose the lowest-motion candidate
      ↓
Execute through the PTP path
```

This multi-seed approach was introduced after mathematically valid IK solutions produced unnecessary wrist flips. The scoring logic gives additional weight to wrist motion so the selected branch is more practical from the current robot state.

### Automatic execution

The GUI provides both **single-placement** and **full-table** execution. A complete cycle is:

```text
Discover/select free target
        ↓
Infer required jig size
        ↓
Find unused matching jig
        ↓
Move above pickup
        ↓
Descend
        ↓
Close gripper + attach jig
        ↓
Lift and transfer
        ↓
Move above target
        ↓
Descend
        ↓
Open gripper + detach
        ↓
Validate placement
        ↓
Mark jig used + target occupied
        ↓
Retract and continue
```

The **FULL RUN – COMPLETE TABLE** mode repeats this process automatically for the remaining free targets without requiring manual reselection between placements.

### Jig inventory & target occupancy

Two independent state trackers prevent incorrect repeated operations:

```python
placed_jig_models
filled_placement_targets
```

`placed_jig_models` prevents a physical jig that has already been placed from being selected again. `filled_placement_targets` prevents a second jig from being sent to an occupied target.

This distinction became important during multi-target runs because a previously placed jig could otherwise become the nearest matching jig during a later scan.

### Simulated grasping & stable detach

Jig grasping is simulated with the **IFRA LinkAttacher** plugin. During pickup the gripper closes and the nearest valid jig is attached. During release, detach requests are queued and executed from Gazebo's update thread rather than directly from a ROS service callback.

That threading change solved a native `gzserver` crash that occurred during repeated release operations.

### Placement validation

A placement is only considered complete after the release operation has been validated. Only then are the used-jig inventory and occupied-target state updated before the controller proceeds to the next target.

### Immediate STOP / RESUME

The automatic workflow supports genuine motion interruption. **STOP** cancels both the active MoveIt sequence goal and the active arm-controller `FollowJointTrajectory` goal, allowing the robot to stop during motion rather than merely setting a software flag.

**RESUME** retries the interrupted automatic step from the robot's current state.

### Demonstrated in simulation

The final workcell demonstrated:

- dynamic target discovery,
- automatic jig-size inference,
- unused-jig matching,
- runtime multi-seed IK,
- wrist-aware branch selection,
- simulated pickup and attachment,
- transfer and release,
- placement validation,
- jig inventory tracking,
- target occupancy tracking,
- single-target execution,
- full-table automatic execution,
- immediate STOP / RESUME.

➡️ **[Read the full Automatic Jig Placement documentation](docs/AUTOMATIC_JIG_PLACEMENT.md)**

---

## Debugging highlight: Gazebo detach crash

One of the hardest failures was a native `gzserver` crash during jig release.

The issue was traced to physics/joint manipulation occurring from a ROS service callback thread. The LinkAttacher implementation was changed so that:

```text
ROS service callback
       ↓
queue detach request
       ↓
Gazebo OnUpdate()
       ↓
Joint::Detach()
```

Executing the actual detach operation on Gazebo's update thread significantly improved repeated pick-and-place stability.

---

## FEA and stiffness

Structural analysis produced two important lessons:

- local joint transitions can create stress concentrations even when the rest of the tube is lightly stressed,
- a robot can be strong enough not to yield while still being too flexible for accurate positioning.

![FEA stress result](assets/screenshots/fea_stress_result.png)

---

## Repository structure

```text
6DOF-Robot-Arm-ROS2/
├── README.md
├── src/                         # ROS 2 source packages
├── scripts/                     # startup scripts
├── config/                      # TCP and saved-pose configuration
├── assets/
│   └── screenshots/             # real project screenshots
└── docs/
    ├── SETUP.md
    ├── PROJECT_STORY.md
    ├── MECHANICAL_DESIGN.md
    ├── SOFTWARE_ARCHITECTURE.md
    ├── SEQUENCE_PROGRAMMING.md
    ├── AUTOMATIC_JIG_PLACEMENT.md
    ├── ENGINEERING_LESSONS.md
    ├── RESULTS.md
    └── GALLERY.md
```

---

## Getting started

The project was developed on **Ubuntu 22.04 + ROS 2 Humble**.

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_arm_ws

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

➡️ **[Setup Guide](docs/SETUP.md)**

---

## Documentation

| Document | What it covers |
|---|---|
| [Project Story](docs/PROJECT_STORY.md) | How the project evolved from manufacturing problem to autonomous workcell |
| [Mechanical Design](docs/MECHANICAL_DESIGN.md) | Structure, materials, torque, reducers, actuators, FEA |
| [Software Architecture](docs/SOFTWARE_ARCHITECTURE.md) | ROS 2, URDF, MoveIt, Gazebo, ros2_control, GUI |
| **[Sequence Programming](docs/SEQUENCE_PROGRAMMING.md)** | **Waypoint programming, PTP/LIN, blending, TRUE/FALSE input logic, BEFORE/AFTER timing, outputs, gripper barriers and STOP/RESUME** |
| [Automatic Jig Placement](docs/AUTOMATIC_JIG_PLACEMENT.md) | Dynamic targets, IK, full run, inventory and placement logic |
| [Engineering Lessons](docs/ENGINEERING_LESSONS.md) | Problems encountered and what each one taught |
| [Results](docs/RESULTS.md) | Demonstrated capabilities and honest project status |
| [Gallery](docs/GALLERY.md) | Real project screenshots |
| [Setup](docs/SETUP.md) | Reproducing the ROS 2 environment |

---

## What this project taught me

This project forced several engineering disciplines to work together rather than in isolation:

**mechanical design → FEA → actuator sizing → kinematics → URDF → ROS 2 → planning → control → sequence programming → simulation → GUI → autonomy → debugging**

Some of the most valuable work came from failures: gearbox interference, flexible links, duplicate planning processes, sequence execution bugs, stale state, controller timing, poor IK branches, reused jig inventory and simulator-threading crashes.

➡️ **[Read the engineering lessons](docs/ENGINEERING_LESSONS.md)**

---

## Current status & roadmap

### Completed / validated in simulation

- [x] 6-DOF robot CAD
- [x] gripper CAD and URDF
- [x] Gazebo simulation
- [x] MoveIt 2 planning
- [x] RViz validation
- [x] ros2_control integration
- [x] custom Python GUI
- [x] saved poses
- [x] multi-waypoint sequence programming
- [x] STOP / CONTINUE waypoint behavior
- [x] blend radius
- [x] PTP / LIN sequence selection
- [x] digital I/O sequence logic
- [x] synchronized gripper sequence actions
- [x] runtime IK
- [x] dynamic target discovery
- [x] autonomous single placement
- [x] full-table automatic execution
- [x] immediate automatic STOP / RESUME

### Future work

- [ ] physical actuator integration
- [ ] EtherCAT / CAN hardware interface
- [ ] real robot calibration
- [ ] camera-based target recognition
- [ ] automatic workplane calibration
- [ ] hand-guided teaching
- [ ] physical payload/stiffness validation
- [ ] industrial electrical and safety architecture
- [ ] complete cell commissioning

---

## Internship context & authorship

This repository documents the robot design, ROS 2 stack, simulation, control GUI, sequence-programming system and automatic-placement engineering work developed during a broader internship project.

The internship project involved a team and supervision; this repository focuses on the technical robotics work represented here and is presented as an educational/portfolio reference rather than as a claim that every part of the wider internship project was completed by one person.

---

## Author

**Yousef El-Beltagy**  
Robotics & Mechatronics

If this project helps you learn ROS 2, robot modeling, MoveIt, robot programming or simulation, feel free to explore the code and documentation.

---

<div align="center">

### CAD. SIMULATE. PLAN. PROGRAM. CONTROL. AUTOMATE.

</div>
