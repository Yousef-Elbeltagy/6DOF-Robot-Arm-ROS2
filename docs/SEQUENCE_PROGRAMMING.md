# Sequence Programming & Execution

The **Sequence Controller** is one of the most important software features in this project. It turned the robot from a manually jogged simulation into a programmable robotic system capable of executing ordered multi-step routines through the custom GUI.

It was also a major engineering milestone on the path toward the later automatic jig-placement system.

---

## Why I built it

Manual jogging is useful for teaching and testing positions, but an industrial robot needs repeatable motion programs.

The goal of the sequence system was to let an operator:

1. teach or save useful robot poses,
2. arrange those poses into an ordered program,
3. choose how the robot behaves at each waypoint,
4. combine arm motion with digital I/O and gripper actions,
5. execute the complete routine through MoveIt 2 and `ros2_control`.

This moved the project closer to the workflow of a real industrial robot controller rather than a simple visualization demo.

---

## Sequence Controller

![Sequence programming GUI](../assets/screenshots/gui_sequence_programming.png)

The GUI sequence window supports a row-based motion program. Each row can define a saved waypoint together with its motion and process behavior.

### Core controls

- **ADD WAYPOINT** — append a saved pose to the program
- **REMOVE SELECTED** — remove a row
- **CLEAR ALL** — reset the sequence
- **RUN SEQUENCE** — execute the programmed routine
- **STOP** — interrupt execution
- **RESUME** — continue after a pause where supported
- **CLOSE SEQUENCE** — close the programming window

---

## Waypoint programming

Each sequence row can contain:

| Field | Purpose |
|---|---|
| **Waypoint** | Saved robot pose to execute |
| **Behavior** | `STOP` or `CONTINUE` at the waypoint |
| **Blend Radius** | Controls continuous transition through eligible poses |
| **Motion** | `PTP` or `LIN` selection |
| **Input Source** | Optional digital input condition |
| **Input State** | Required `TRUE` / `FALSE` state |
| **Input Timing** | Check input `BEFORE` or `AFTER` the waypoint |
| **Output Action** | Digital output or gripper action |
| **Output State** | Output state where applicable |
| **Output Timing** | Execute output `BEFORE` or `AFTER` the waypoint |

The final waypoint is forced to stop safely rather than blending through the end of a program.

---

## STOP vs CONTINUE

Two waypoint behaviors were implemented:

### STOP

The arm reaches the programmed pose and stops before the next operation.

This is useful for:

- gripping,
- releasing,
- process synchronization,
- inspection points,
- I/O operations,
- safety-critical transitions.

### CONTINUE

The sequence can continue through the waypoint using the configured blend behavior, reducing unnecessary stop-start motion.

This was inspired by the way industrial robot programs use approximation/blending to create smoother multi-point trajectories.

---

## PTP and LIN motion types

The sequence interface exposes two motion types:

### PTP — Point-to-Point

The robot moves through joint space toward the target pose.

PTP became the most reliable and heavily validated motion mode in the project and is intentionally used by the automatic jig-placement workflow.

### LIN — Linear Cartesian motion

LIN was added to support straight Cartesian tool motion between programmed points.

This feature required considerably more debugging because Cartesian planning, IK, Pilz limits, start-state validation and sequence execution all interact. Several issues were found during development, including motion types being unintentionally overwritten by PTP and Pilz rejecting some trajectories because of acceleration/start-state constraints.

The GUI retains LIN selection because it is part of the intended industrial-style programming model, while PTP is the more thoroughly validated mode in the final automatic workflow.

---

## Motion-type bug that had to be fixed

An early version of the mixed-sequence code contained logic equivalent to:

```python
working_motion_types = ["PTP"] * len(...)
```

That silently replaced the motion type selected by the operator, meaning a row shown as LIN could still execute through the PTP path.

The sequence logic was changed to preserve the actual per-waypoint motion-type array instead of regenerating it as all-PTP.

An intermediate edit also introduced an undefined `segment_end` variable, which was corrected while restructuring the sequence segmentation logic.

These bugs were important because a sequence controller has to execute **what the operator programmed**, not just what the GUI displays.

---

## Digital I/O integration

The sequence controller grew beyond motion waypoints and added process I/O.

### Inputs

Supported logical sources include:

```text
NONE
IN 1
IN 2
IN 3
IN 4
IN 5
IN 6
```

Each input can specify:

```text
required state: TRUE / FALSE
timing: BEFORE / AFTER
```

This provides the basis for waiting on external process conditions before continuing a robot program.

### Outputs

Supported actions include:

```text
NONE
OUT 1 ... OUT 6
OPEN GRIPPER
CLOSE GRIPPER
```

Outputs can also be executed before or after the associated waypoint.

This turns the sequence table into a basic robot/process programming environment rather than just a list of poses.

---

## Gripper synchronization — the barrier problem

One of the harder sequence problems was making sure the arm did **not** continue to the next waypoint while the gripper was still moving.

The final approach treats a gripper operation as an execution barrier:

```text
arm trajectory segment
        ↓
stop at gripper waypoint
        ↓
send gripper FollowJointTrajectory goal
        ↓
wait for the actual controller result
        ↓
continue with the next arm segment
```

The sequence therefore forces the arm to stop before a gripper action, waits for the real gripper controller result, and only then resumes arm motion.

The generic digital-output handler also avoids issuing duplicate OPEN/CLOSE commands when the dedicated gripper barrier is already handling them.

---

## Why cached gripper state was removed

An earlier shortcut trusted a cached variable similar to:

```python
self.current_gripper_position
```

Because GUI state can lag behind Gazebo/controller state, that caused false conditions such as "already open" or incorrect post-command validation.

The more reliable design is:

```text
send controller goal
      ↓
wait for acceptance
      ↓
wait for FollowJointTrajectory result
      ↓
check controller result code
      ↓
continue sequence
```

This was an important lesson in asynchronous robot-control software: **controller acknowledgement is more reliable than a stale UI cache**.

---

## Sequence execution architecture

At a high level:

```text
Saved Poses
    ↓
Sequence Controller GUI
    ↓
Waypoint + behavior + blend + motion + I/O
    ↓
Segment sequence around STOP / I/O / gripper barriers
    ↓
MoveIt 2 planning / sequence action
    ↓
arm_controller FollowJointTrajectory
    ↓
ros2_control
    ↓
Gazebo simulated robot
```

The project uses the MoveIt sequence action:

```text
/sequence_move_group
```

and the arm controller action:

```text
/arm_controller/follow_joint_trajectory
```

with the gripper using its own FollowJointTrajectory controller.

---

## Problems encountered during development

The sequence system took substantial iteration because the failure could occur at several different layers.

### 1. Large plans failed while small moves worked

Small Cartesian/joint increments could succeed while larger sequence plans failed. This required separating planning problems from controller/execution problems instead of assuming every failure had the same cause.

### 2. Duplicate MoveIt instances

At one point duplicate `move_group` processes produced confusing sequence-action behavior and apparent failures even when the robot moved.

The startup workflow was cleaned so only one intended planning stack runs.

### 3. Motion type was being overwritten

As described above, parts of the code unintentionally forced PTP. The mixed-sequence path had to preserve each row's actual motion type.

### 4. LIN / Pilz constraints

Linear motion exposed acceleration-limit, IK and start-state issues that did not appear in the same way during PTP execution.

### 5. Gripper timing

The arm initially could continue before the gripper had actually completed. The explicit gripper barrier fixed this.

### 6. Action result handling

The system had to distinguish between:

- goal accepted,
- motion actually executed,
- action result returned,
- cancellation,
- apparent sequence failure caused by duplicate servers.

This debugging work significantly improved the project's ROS 2 action-handling architecture.

---

## Relationship to automatic jig placement

The sequence controller and the automatic jig-placement subsystem are different layers, but the sequence work was foundational.

The project evolved approximately as:

```text
manual joint/cartesian control
          ↓
saved poses
          ↓
sequence programming
          ↓
motion + I/O + gripper synchronization
          ↓
reliable controller/action handling
          ↓
automatic jig-placement state machine
```

The later automatic system uses more specialized runtime logic, but many of the hard lessons about planning, execution, cancellation, gripper synchronization and controller state came from building the sequence feature first.

---

## Why this feature matters

The Sequence Controller demonstrates more than GUI development. It combines:

- robot programming concepts,
- reusable taught poses,
- motion planning,
- industrial-style PTP/LIN selection,
- blending,
- digital I/O,
- gripper synchronization,
- asynchronous ROS 2 action handling,
- STOP/RESUME concepts,
- error handling across MoveIt and ros2_control.

For this project, sequence programming was the point where the robot stopped being just a model that could move and started becoming a **programmable robotic system**.

---

[← Back to main README](../README.md)
