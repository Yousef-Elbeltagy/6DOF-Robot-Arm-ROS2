# Continue Development

This document is for the next developer who wants to continue the project from the current simulation-validated state rather than reconstructing its history from commits.

## Current validated state

The repository contains a working ROS 2 Humble simulation/control stack for a 6-DOF arm with gripper and a wire-harness jig-placement workcell.

Validated in simulation:

- 6-DOF arm URDF and meshes
- gripper integration
- Gazebo Classic workcell
- ros2_control arm and gripper trajectory execution
- MoveIt 2 planning
- RViz validation
- Cartesian and joint jogging
- saved robot poses
- configurable TCP
- sequence programming
- STOP / CONTINUE waypoint behavior
- blend radius
- PTP / LIN selection
- digital input/output logic
- synchronized gripper actions
- runtime IK
- dynamic placement-target discovery
- unused-jig inventory tracking
- target occupancy tracking
- single automatic placement
- full-table automatic execution
- immediate STOP / RESUME

## Important files

```text
src/robot_arm_python/robot_arm_python/robot_gui.py
    Main GUI, manual control, sequence logic, runtime IK and automatic placement.

src/robot_arm_with_gripper_urdf/
    Robot/gripper model, Gazebo launch files and workcell-related simulation assets.

src/robot_arm_moveit_config/
    MoveIt configuration, planning pipelines and controllers.

src/IFRA_LinkAttacher/
    Modified link-attacher implementation used for simulated jig grasp/release.

config/robot_arm_saved_poses.json
    Portable copy of saved poses.

config/robot_arm_tcp_config.json
    Portable copy of the validated TCP configuration.

scripts/start_robot.sh
    Main system launcher.

scripts/start_robot_DIAGNOSTIC_GAZEBO.sh
    Diagnostic startup path for Gazebo-related failures.
```

## Runtime invariants worth preserving

### TCP

The validated TCP offset is approximately:

```text
[0, 0, 0.4193463143, 0, 0, 0]
```

Do not substitute the `link_6 -> gripper_base` transform as the TCP.

### Controller names

Arm:

```text
/arm_controller/follow_joint_trajectory
```

Gripper:

```text
/gripper_controller/follow_joint_trajectory
```

MoveIt sequence action:

```text
/sequence_move_group
```

### Automatic-placement state

Two independent sets matter:

```python
placed_jig_models
filled_placement_targets
```

The first prevents reusing a physical jig. The second prevents placing into an already occupied target. Do not merge them.

## Automatic jig placement architecture

The high-level flow is:

```text
Discover free target TFs
        ↓
Infer jig family from target name
        ↓
Find unused matching jig
        ↓
Generate runtime IK candidates
        ↓
Score candidate joint motion
        ↓
Move to pickup
        ↓
Close + attach
        ↓
Transfer
        ↓
Open + detach
        ↓
Validate
        ↓
Update inventory + occupancy
        ↓
Continue
```

Targets are discovered dynamically from TF frame names ending in `_target_link`.

## Runtime IK

The automatic controller intentionally uses multiple IK seeds because the first valid mathematical solution may contain unnecessary wrist rotation.

The candidate score is based on wrapped joint deltas from the current state, with additional weight on wrist motion. This should remain generic; avoid target-specific joint hard-coding unless a clearly documented exception is unavoidable.

## Sequence Controller

Each programmed row can contain:

- saved waypoint
- STOP / CONTINUE
- blend radius
- PTP / LIN
- input source
- TRUE / FALSE input state
- BEFORE / AFTER condition timing
- output action
- output state
- BEFORE / AFTER output timing
- OPEN GRIPPER / CLOSE GRIPPER

Gripper actions use an execution barrier: arm motion reaches the appropriate step, the gripper trajectory is sent, the actual action result is awaited, then arm execution continues.

Do not reintroduce asynchronous gripper execution or duplicate gripper commands through the generic output path.

## Immediate STOP / RESUME

STOP is intended to cancel both:

```text
active MoveIt sequence goal
active arm FollowJointTrajectory goal
```

RESUME retries the interrupted step from the robot's current state.

If changing execution logic, verify that STOP still interrupts actual robot motion rather than merely setting a software flag.

## Gazebo detach stability

A critical stability fix exists in the modified IFRA LinkAttacher implementation.

Unsafe historical pattern:

```text
ROS service callback -> Joint::Detach()
```

Stable architecture:

```text
ROS service callback
    ↓
queue detach request
    ↓
Gazebo OnUpdate()
    ↓
Joint::Detach()
```

Do not casually replace the modified implementation with a stock copy without porting this behavior.

## Recommended next software work

1. Add repeatable automated smoke tests.
2. Improve launch orchestration so fewer external terminal windows are required.
3. Add parameterization for workcell geometry and target naming.
4. Improve LIN reliability and start-state handling.
5. Add structured logging instead of relying only on console text.
6. Add camera/perception integration for target extraction.
7. Add calibration helpers for a real workplane.
8. Separate simulation-specific attach/detach logic from future hardware gripper logic.

## Recommended next hardware work

1. Integrate the selected actuators.
2. Implement EtherCAT or CAN hardware interfaces.
3. Validate motor brakes and startup behavior.
4. Perform joint-zero calibration.
5. Validate payload and structural deflection physically.
6. Measure TCP repeatability.
7. Implement electrical safety architecture.
8. Add guarding / safety-rated functions as required by the final application.
9. Commission the complete cell.

## Modifying the robot geometry

If the mechanical geometry changes:

1. Update the CAD assembly.
2. Re-export affected meshes.
3. Update URDF joint origins / axes / inertias.
4. Rebuild the workspace.
5. Validate the TF tree.
6. Validate collision geometry in RViz.
7. Test small motions before large Cartesian moves.
8. Re-check the TCP.
9. Re-check saved poses; old Cartesian poses may no longer be valid.

## Adding a new target

Prefer the existing dynamic naming model:

```text
<target_id>_target_link
```

Choose a target prefix that maps clearly to a jig family, then ensure the target TF is published and that the corresponding jig model can be discovered by the matching logic.

After adding a target, test:

- target appears in free-target discovery
- correct jig family is inferred
- used jigs are excluded
- placement IK has a reasonable branch
- target becomes occupied only after validated placement

## Adding a new jig family

Update all layers consistently:

1. jig model / URDF
2. spawn or workcell configuration
3. naming convention
4. GUI family inference
5. matching-jig search
6. pickup validation
7. attach/detach logic if link names differ

## Before a risky change

Create a Git checkpoint first:

```bash
git status
git add -A
git commit -m "Checkpoint before <change>"
```

Then change one subsystem at a time and verify the previous known-good behavior before continuing.

## Project status statement

This repository should be described as a simulation-validated engineering prototype. It is not a physically commissioned or safety-certified collaborative robot.
