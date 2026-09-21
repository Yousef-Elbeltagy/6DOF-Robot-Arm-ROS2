# Troubleshooting

Use this guide before changing coordinates, controller names, IK logic, timeouts, or workcell geometry. Many failures in this project were caused by process duplication, stale state, asynchronous execution, or simulator-thread timing rather than by the target pose itself.

## 1. More than one MoveIt process

Check:

```bash
ros2 node list | grep move_group
```

There should normally be one `/move_group` node.

Symptoms of duplicate MoveIt instances can include:

- confusing action behavior
- goals accepted by the wrong process
- inconsistent planning state
- sequence actions failing while the robot still moves

Stop duplicate launch processes before changing planning code.

## 2. Small motions work but larger plans fail

Check:

- current robot state is valid
- joint limits
- collision scene
- start state freshness
- planning pipeline
- requested orientation

Try a very small Cartesian or joint change first. If that works, increase motion gradually rather than immediately changing URDF geometry or controller settings.

## 3. LIN motion fails

PTP became the most thoroughly validated sequence mode. LIN through Pilz is more sensitive to:

- start-state validity
- IK feasibility
- acceleration constraints
- joint-state freshness
- Cartesian path geometry

Confirm PTP works first. Then isolate one LIN segment and test it independently.

## 4. Sequence says failed although motion occurred

Inspect both the MoveIt sequence action and the underlying trajectory-controller result.

Relevant interfaces:

```text
/sequence_move_group
/arm_controller/follow_joint_trajectory
```

Do not assume that goal acceptance means successful completion.

## 5. Gripper command overlaps the next arm move

The known-good sequence architecture uses a gripper barrier:

```text
arm segment
   ↓
stop
   ↓
send gripper goal
   ↓
wait for FollowJointTrajectory result
   ↓
continue arm motion
```

If the arm moves before the gripper finishes, check that the generic output handler is not also issuing the gripper command and that execution waits for the real action result.

## 6. STOP does not stop motion immediately

The intended STOP path cancels both:

```text
active MoveIt goal
active arm-controller FollowJointTrajectory goal
```

A software flag alone is not sufficient.

Verify action cancellation before changing GUI state logic.

## 7. Automatic placement cannot find a target

Inspect available TF frames:

```bash
ros2 run tf2_tools view_frames
```

or inspect TF output manually.

Automatic target discovery expects frame names ending in:

```text
_target_link
```

Also verify that the target has not already been added to `filled_placement_targets`.

## 8. Wrong jig selected

The system must exclude jig models already recorded in:

```python
placed_jig_models
```

If a jig already placed on the board becomes the nearest jig during a later scan, verify used-jig filtering before changing source-table coordinates.

## 9. "Gripper closed but no jig was acquired"

Possible causes include:

- stale GUI TCP pose
- selected jig is outside pickup range
- attach service unavailable
- unexpected model/link name
- attach service returned failure

The GUI should refresh its pose before nearest-jig selection rather than relying on old cached coordinates.

Useful checks:

```bash
ros2 service list | grep attach
```

and inspect runtime logs for attach-distance diagnostics.

Do not immediately enlarge pickup thresholds or change jig coordinates without first checking the reported TCP/jig distance.

## 10. Gazebo crashes during jig release

A critical historical issue was a native `gzserver` crash caused by calling `Joint::Detach()` directly from a ROS service callback thread.

Known-good behavior:

```text
ROS service callback
    ↓
queue detach request
    ↓
Gazebo OnUpdate()
    ↓
Joint::Detach()
```

If detach crashes return, verify this modified threading behavior is still present in `src/IFRA_LinkAttacher/`.

Do not replace the package with an upstream copy without preserving the fix.

## 11. Gazebo is running but windows are not visible

Check running processes:

```bash
ps aux | grep -E 'gzserver|gzclient|rviz2|move_group'
```

A process can be alive even when its window is hidden, off-screen, or attached to another workspace.

Use the diagnostic launcher if needed:

```bash
bash scripts/start_robot_DIAGNOSTIC_GAZEBO.sh
```

## 12. Controllers are not active

Check:

```bash
ros2 control list_controllers
```

Expected trajectory controllers should be active before starting the GUI.

Also inspect available actions:

```bash
ros2 action list
```

You should see arm and gripper `FollowJointTrajectory` actions.

## 13. GUI starts but poses/TCP are missing

The GUI expects:

```text
~/robot_arm_saved_poses.json
~/robot_arm_tcp_config.json
```

Restore the repository copies:

```bash
cp config/robot_arm_saved_poses.json ~/robot_arm_saved_poses.json
cp config/robot_arm_tcp_config.json ~/robot_arm_tcp_config.json
```

## 14. Wrist flips during automatic placement

A valid IK solution is not necessarily a good motion.

The current automatic controller intentionally:

1. tries multiple IK seeds
2. collects valid solutions
3. computes wrapped joint deltas
4. weights wrist movement more strongly
5. selects the lowest-motion candidate

Before adding a target-specific joint override, inspect candidate selection and confirm the current joint state is fresh.

## 15. Robot moves but TCP is wrong

Do not confuse:

```text
link_6 -> gripper_base
```

with the configured tool-center-point offset.

The validated TCP Z translation is approximately:

```text
0.4193463143 m
```

Check `~/robot_arm_tcp_config.json`.

## 16. Full-table run reuses a target or jig

Both state trackers are required:

```python
placed_jig_models
filled_placement_targets
```

They solve different problems and must remain independent.

## 17. Build fails after editing packages

Clean only when necessary:

```bash
cd ~/robot_arm_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Then source again:

```bash
source ~/robot_arm_ws/install/setup.bash
```

## 18. First debugging principle

When a previously working behavior breaks:

1. reproduce once
2. capture the exact error/output
3. verify process state
4. verify controller/action/service availability
5. inspect current robot/TCP state
6. change one thing only
7. re-test

Avoid simultaneous changes to motion planning, coordinates, timeouts, and simulator logic. That makes regressions much harder to isolate.
