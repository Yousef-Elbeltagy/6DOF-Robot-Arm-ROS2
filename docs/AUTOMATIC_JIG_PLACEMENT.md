# Automatic Jig Placement

The automatic jig-placement system is the most application-specific part of the project. It turns the robot from a manual simulation into a complete pick-and-place workflow for a reconfigurable wire-harness workcell.

## Problem being solved

Traditional wire-harness boards are often dedicated to a product or variant. That makes changeover slow, storage-heavy and inflexible.

The project concept uses a reusable workboard with modular jigs. The robot selects the required jig from a source table and places it at the correct target on the board.

## Workcell model

The simulated workcell contains:

- 6-DOF robot arm
- gripper
- source jig table
- large / medium / small jig models
- target placement table
- named target TF frames

Example targets:

```text
L1
M1
M2
S1
```

The target prefix represents the required jig family:

```text
L -> large jig
M -> medium jig
S -> small jig
```

## Dynamic target discovery

The target list is not intended to be hard-coded.

The GUI inspects TF frames at runtime and searches for frame names ending in:

```text
_target_link
```

It then converts those frame names into operator-friendly target IDs and excludes targets that have already been filled.

This means the GUI can adapt to the workcell state instead of relying only on a fixed list of coordinates.

## Jig inventory tracking

A separate set tracks jig models that have already been used:

```python
self.placed_jig_models
```

This was added after a real simulation bug: once M1 had been filled, the nearest medium jig to M2 could accidentally be the jig already placed at M1.

The fix was to keep an explicit used-jig inventory and ignore placed models during future scans.

Target occupancy is tracked separately using:

```python
self.filled_placement_targets
```

These two states solve different problems:

- `placed_jig_models` prevents reusing a physical jig
- `filled_placement_targets` prevents filling the same target twice

## Automatic sequence

A typical placement cycle is:

```text
1. Discover/select target
2. Infer required jig size
3. Scan for matching unused jig
4. Open gripper
5. Move above pickup
6. Descend to pickup
7. Close gripper and attach jig
8. Lift
9. Move above target
10. Descend to placement
11. Open gripper and detach jig
12. Validate placement
13. Mark jig used
14. Mark target occupied
15. Retract
16. Continue to next target
```

Known working height conventions during the simulation were approximately:

```text
pickup Z         0.370 m
placement Z      0.371 m
approach/retract 0.500 m
```

The safe travel height was increased during development after inter-target motion passed too close to already placed jigs.

## Runtime inverse kinematics

The automatic routine does not depend on one pre-recorded joint pose for every target.

Instead it uses MoveIt's IK service to compute a joint solution for the requested TCP pose.

The runtime IK process:

1. accepts a desired TCP pose
2. converts it to the `link_6` flange pose
3. calls MoveIt IK for planning group `arm`
4. tries multiple seed configurations
5. evaluates valid solutions
6. chooses the solution requiring the most reasonable wrapped joint motion
7. executes through the known-good PTP path

## Closest IK branch selection

One of the most visible problems during development was a valid IK solution that caused unnecessarily large wrist rotations.

Rather than hard-code one target, the controller evaluates multiple IK candidates and scores their wrapped joint differences from the current robot state.

The score gives additional weight to the wrist joints, which reduces undesirable wrist flips while keeping the solution generic.

A generic target-azimuth seed was also added to improve branch selection for targets such as S1.

## Simulated grasping

Jig grasping is simulated through the IFRA LinkAttacher plugin.

The robot closes its gripper and calls the attach service for the nearest valid jig within the pickup range.

During release, the detach operation is queued and executed in the Gazebo update thread. This threading change was necessary to prevent native `gzserver` crashes during repeated automatic runs.

## Full-table run

The GUI supports a full-run mode that repeatedly:

- searches for remaining free targets
- identifies the required jig type
- finds an unused matching jig
- executes placement
- validates completion
- updates occupancy/inventory
- continues automatically

This allowed the simulated workcell to complete the available L1/M1/M2/S1 placement sequence without manual reselection between targets.

## STOP / RESUME

The automatic controller supports immediate interruption.

STOP cancels the active MoveIt goal and arm-controller trajectory instead of waiting for the movement to finish.

RESUME then retries the interrupted automatic step from the current robot state.

This was particularly useful during development and demonstrations because the complete routine could be safely interrupted without restarting the whole run.

## Honest project status

The automatic workcell was validated in simulation. It demonstrates the control architecture and application logic, but it is not a physically commissioned production cell.

A real implementation would still require actuator/hardware integration, calibration, real gripping validation, electrical safety, machine safety and full commissioning.
