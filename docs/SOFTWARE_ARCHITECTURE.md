# Software Architecture

The project combines a CAD-derived robot model with ROS 2 motion planning, physics simulation, trajectory control and a custom operator interface.

## Platform

```text
Ubuntu 22.04 LTS
ROS 2 Humble
Gazebo Classic 11
MoveIt 2
RViz 2
ros2_control
ros2_controllers
OMPL
Pilz Industrial Motion Planner
Python 3 / Tkinter
```

## Data flow

```text
SolidWorks CAD
      ↓
URDF + STL meshes
      ↓
ROS 2 robot description
      ↓
+-----------------------------+
|                             |
↓                             ↓
Gazebo Classic             MoveIt 2
physics / joints           planning / IK
|                             |
+-------------+---------------+
              ↓
          ros2_control
              ↓
      trajectory controllers
              ↓
      custom Python GUI
```

## Main ROS packages

The development workspace contains packages for the robot, workcell and application logic, including:

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

## URDF model

The robot model contains six revolute arm joints plus the full gripper hierarchy.

The CAD-to-URDF workflow preserves:

- parent/child link hierarchy
- joint origin and axis
- joint limits
- mass
- center of mass
- inertia tensor
- visual meshes
- collision meshes
- gripper mimic relationships

## MoveIt 2

MoveIt is used for:

- inverse kinematics
- collision-aware planning
- joint-limit checking
- trajectory generation
- planning scene visualization
- sequence execution

The main planning group is:

```text
arm
```

The project uses OMPL and Pilz planning pipelines. PTP is the known-good motion mode used by the automatic placement workflow.

## ros2_control

The simulated arm is controlled through a FollowJointTrajectory controller for J1-J6, with a separate trajectory controller for the gripper.

Important actions include:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
/sequence_move_group
```

## Custom operator GUI

The Python/Tkinter controller evolved into a full robot operator interface.

It supports:

- Cartesian X/Y/Z jogging
- Roll/Pitch/Yaw jogging
- J1-J6 joint jogging
- speed override
- WORLD / FLANGE / TOOL jog frames
- TCP configuration
- saved poses
- HOME
- STOP / RESUME
- gripper open/close
- sequence programming
- PTP / LIN row selection
- STOP / CONTINUE waypoint behavior
- blend radius
- digital I/O
- automatic jig placement

## Tool Center Point

The working TCP used by the controller is approximately:

```text
[0, 0, 0.4193463143, 0, 0, 0]
```

This is intentionally different from the shorter `link_6 -> gripper_base` transform. The GUI performs the TCP/flange conversion mathematically when generating motion targets.

## Sequence controller

The sequence system is designed to feel closer to an industrial robot programming workflow than a basic list of poses.

Each row can contain:

- saved waypoint
- STOP / CONTINUE behavior
- blend radius
- PTP / LIN selection
- digital input condition
- digital output action
- gripper actions

Gripper actions act as execution barriers: arm motion stops, the gripper trajectory completes, and only then does the next arm segment begin.

## Immediate STOP / RESUME

A major behavior improvement was making STOP interrupt an active automatic movement rather than waiting for the current trajectory to finish.

The implementation cancels both the MoveIt sequence goal and the active arm-controller trajectory. RESUME then retries the interrupted automatic step from the robot's current state.

## Gazebo grasping

The simulated pick-and-place uses IFRA LinkAttacher services to attach and detach jig models.

A native Gazebo crash was traced to detach operations being performed from the ROS service callback thread. The plugin was modified so that detach requests are queued and the actual `Joint::Detach()` occurs inside Gazebo's update thread.

This was one of the most important simulation-stability fixes in the project.

## Development philosophy

The project was debugged using an evidence-first workflow:

1. reproduce the failure
2. inspect ROS/Gazebo logs
3. isolate the failing layer
4. change one behavior at a time
5. rebuild
6. cold-restart when runtime state matters
7. create checkpoints after stable milestones

That workflow was especially important because failures could originate from the GUI, MoveIt, controller timing, TF, Gazebo physics or plugin threading.
