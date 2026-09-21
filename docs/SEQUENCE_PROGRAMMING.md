# Sequence Programming & Execution

The **Sequence Controller** is a dedicated robot-programming layer in this project. It turns taught/saved poses into ordered motion programs and combines robot motion with process logic, digital I/O, gripper actions, blending, and execution control.

This was one of the most important software milestones between manual jogging and the later automatic jig-placement system.

---

## Sequence entry point in the main robot GUI

The main operator application provides jogging, gripper control, digital I/O, saved poses, sequence programming, and automatic jig placement from one interface.

![Main robot control GUI](../assets/screenshots/gui_manual_control.png)

From the **Saved Robot Poses** area, the operator can teach/store reusable poses and open the **SEQUENCE CONTROLLER** to build a program from them.

---

## Sequence Controller running with the simulated robot

The sequence window is a separate programming interface while Gazebo continues to show the robot and workcell.

![Sequence Controller with Gazebo robot](../assets/screenshots/gui_sequence_programming.png)

The example capture shows a four-step program — `pick`, `mid`, `place`, and `home` — with motion type, stop behavior, blend radius, input conditions, output/gripper actions, and timing all configured row by row.

---

## What can be programmed in each row

| Sequence field | Available behavior | Purpose |
|---|---|---|
| **Waypoint** | Any saved robot pose | Defines the target pose for that program step |
| **Behavior** | `STOP` / `CONTINUE` | Stop precisely at the waypoint or continue through an eligible blended transition |
| **Blend Radius** | e.g. `0.001 m` | Controls the continuous transition around a waypoint when blending is allowed |
| **Motion** | `PTP` / `LIN` | Selects point-to-point or linear Cartesian motion |
| **Input Source** | `NONE`, `IN 1` … `IN 6` | Selects the digital input used as a process condition |
| **Input State** | `TRUE` / `FALSE` | Defines the state that the selected input must satisfy |
| **Input Timing** | `BEFORE` / `AFTER` | Checks the condition before or after the waypoint operation |
| **Output Action** | `NONE`, `OUT 1` … `OUT 6`, `OPEN GRIPPER`, `CLOSE GRIPPER` | Commands a digital output or the gripper |
| **Output State** | `TRUE` / `FALSE` | Defines the output state where applicable |
| **Output Timing** | `BEFORE` / `AFTER` | Executes the output/gripper action before or after waypoint motion |

The final waypoint is treated as a safe stopping point rather than being blended through the end of the program.

---

## Example process logic

A sequence can express logic such as:

```text
1. PICK
   IF IN 1 == TRUE BEFORE
   move with LIN
   CLOSE GRIPPER AFTER

2. MID
   IF IN 2 == FALSE BEFORE
   move through the intermediate pose
   set OUT 1 = TRUE BEFORE

3. PLACE
   IF IN 3 == TRUE AFTER
   move to the placement pose
   OPEN GRIPPER AFTER

4. HOME
   return to the home pose
   stop safely at the final waypoint
```

This makes the table more than a pose list: it becomes a small industrial-style robot/process program.

---

## STOP vs CONTINUE

### STOP

The robot reaches the programmed waypoint and stops before the next operation. This is useful for:

- gripping and releasing,
- process synchronization,
- digital I/O changes,
- inspection points,
- any step where the next action must wait for the current one to finish.

### CONTINUE

The sequence can continue through an eligible waypoint using the configured blend radius. This reduces unnecessary stop-start motion and follows the same general programming idea used by industrial robot controllers for approximated/blended paths.

---

## PTP and LIN motion

### PTP — Point-to-Point

PTP plans a joint-space move toward the target pose. It became the most reliable and thoroughly validated sequence mode in this project and is intentionally used for the final automatic jig-placement workflow.

### LIN — Linear Cartesian motion

LIN was implemented so the operator can request straight Cartesian tool motion between programmed points. It required substantially more debugging because Pilz planning, IK, acceleration limits, and the robot start state all affect whether a linear path can be accepted.

An important bug found during development was that an older mixed-sequence path could overwrite the operator's selected motion types and force all rows to PTP. The execution logic was changed so the real per-row `PTP` / `LIN` selection is preserved.

---

## Digital inputs: BEFORE / AFTER + TRUE / FALSE

The sequence controller supports six logical digital inputs:

```text
IN 1  IN 2  IN 3  IN 4  IN 5  IN 6
```

For each programmed input condition the operator selects:

```text
expected state: TRUE or FALSE
check timing:   BEFORE or AFTER
```

Examples:

```text
IF IN 1 == TRUE BEFORE PICK
IF IN 2 == FALSE BEFORE MID
IF IN 3 == TRUE AFTER PLACE
```

This provides the basis for synchronizing the robot with sensors, fixtures, PLC-style process states, or other cell conditions.

---

## Digital outputs

Six logical outputs are available:

```text
OUT 1  OUT 2  OUT 3  OUT 4  OUT 5  OUT 6
```

An output can be commanded `TRUE` or `FALSE`, and its action can occur `BEFORE` or `AFTER` the associated waypoint. That allows the robot program to coordinate motion with process equipment rather than treating motion and I/O as separate systems.

---

## Gripper actions inside a sequence

The output-action field also supports:

```text
OPEN GRIPPER
CLOSE GRIPPER
```

These are not treated like ordinary fire-and-forget outputs. The arm and gripper must be synchronized so that the next arm move cannot begin while the gripper is still opening or closing.

The final execution model uses a **gripper barrier**:

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
continue the next arm segment
```

The generic output handler also ignores gripper outputs when the dedicated gripper barrier is handling them, preventing duplicate open/close commands.

---

## Why cached gripper state was removed

An older implementation used a cached GUI value similar to:

```python
self.current_gripper_position
```

That state could lag behind the actual Gazebo/controller state. The more robust design waits for the real `FollowJointTrajectory` action result and trusts successful controller completion (`error_code == 0`) before continuing.

This was an important ROS 2 control lesson: **goal acceptance is not the same as motion completion**.

---

## RUN, STOP, and RESUME

The Sequence Controller provides:

- **RUN SEQUENCE** — starts the programmed routine,
- **STOP** — interrupts active execution,
- **RESUME** — continues/retries the interrupted step from the robot's current state,
- **REMOVE WAYPOINT** — removes an unwanted program row,
- **CLEAR SEQUENCE** — resets the programmed sequence.

The STOP path was developed beyond a simple GUI flag. It cancels the active MoveIt sequence goal and the active arm-controller `FollowJointTrajectory` goal so the simulated robot can actually stop during motion. RESUME then restarts the interrupted operation from the current state rather than assuming the previous trajectory completed.

---

## Execution architecture

```text
Saved robot poses
      ↓
Sequence Controller GUI
      ↓
Waypoint + behavior + blend + motion
      ↓
Input conditions + output/gripper actions
      ↓
Segment around STOP / I/O / gripper barriers
      ↓
MoveIt 2 / Pilz planning
      ↓
/sequence_move_group
      ↓
/arm_controller/follow_joint_trajectory
      ↓
ros2_control
      ↓
Gazebo robot
```

The gripper uses its own trajectory controller:

```text
/gripper_controller/follow_joint_trajectory
```

---

## Important sequence bugs and engineering lessons

The system required substantial iteration because failures could occur at the GUI, planner, action, controller, or simulator layer. Key issues solved during development included:

- mixed-sequence logic unintentionally forcing rows to PTP,
- an intermediate `segment_end` segmentation bug,
- duplicate `move_group` processes causing confusing action behavior,
- LIN/Pilz acceleration and start-state constraints,
- IK/start-state failures that appeared only on some linear moves,
- stale cached gripper state,
- distinguishing goal acceptance from actual execution completion,
- preventing duplicate gripper commands,
- forcing synchronization barriers around open/close actions,
- implementing actual mid-motion STOP and useful RESUME behavior.

---

## Relationship to automatic jig placement

The sequence controller and automatic jig placement are separate application layers, but the sequence work laid much of the control foundation:

```text
manual jogging
      ↓
saved poses
      ↓
sequence programming
      ↓
PTP/LIN + blending
      ↓
I/O + gripper synchronization
      ↓
robust action handling and cancellation
      ↓
automatic jig-placement state machine
```

The automatic placement workflow later added runtime IK, dynamic TF target discovery, jig inventory, attach/detach logic, occupied-target tracking, validation, and full-table execution.

---

## Why this feature matters

The Sequence Controller demonstrates the integration of:

- reusable taught robot poses,
- industrial-style waypoint programming,
- PTP and LIN motion selection,
- STOP/CONTINUE behavior,
- path blending,
- digital input conditions,
- TRUE/FALSE process logic,
- BEFORE/AFTER event timing,
- digital outputs,
- synchronized gripper operations,
- MoveIt 2 sequence execution,
- ros2_control trajectory actions,
- cancellation and STOP/RESUME behavior,
- debugging across asynchronous ROS 2 components.

For this project, sequence programming is the point where the robot moved from **"a simulated arm that can be jogged"** to **"a programmable robotic system that can execute an ordered process."**

---

[← Back to main README](../README.md)
