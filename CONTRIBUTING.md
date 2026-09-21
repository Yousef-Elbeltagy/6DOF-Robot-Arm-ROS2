# Contributing

Contributions that improve reproducibility, simulation stability, robot modeling, control, sequence programming, or automatic jig placement are welcome.

## Before changing code

1. Read [docs/REPRODUCE_PROJECT.md](docs/REPRODUCE_PROJECT.md).
2. Read [docs/CONTINUE_DEVELOPMENT.md](docs/CONTINUE_DEVELOPMENT.md).
3. Check [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) if the change is intended to fix an existing runtime issue.
4. Confirm the current `main` branch works before introducing a change.

## Development baseline

The validated environment is:

```text
Ubuntu 22.04 LTS
ROS 2 Humble
Gazebo Classic 11
MoveIt 2
RViz 2
ros2_control
Pilz + OMPL
Python 3
```

Changes targeting a different ROS/Gazebo generation should clearly state that they have not been validated against the original baseline unless tested.

## Branching

Use a descriptive branch name, for example:

```text
feature/camera-target-detection
fix/link-attacher-detach
feature/hardware-interface
refactor/gui-logging
```

## Commit style

Prefer small commits that each have one purpose:

```text
Fix detach request handling on Gazebo update thread
Add camera-derived target frame publisher
Document EtherCAT hardware interface plan
```

Avoid mixing unrelated mechanical, GUI, MoveIt, and simulator changes into one commit.

## Pull requests

A useful pull request should state:

- what changed
- why it changed
- how it was tested
- which project behavior was verified afterward
- whether the change affects simulation only or future hardware behavior

Screenshots, terminal output, or short clips are useful for GUI/simulation changes.

## Testing expectations

At minimum, verify the area you changed plus one adjacent subsystem.

Examples:

### URDF / geometry changes

- robot loads in Gazebo
- TF tree is valid
- MoveIt model loads
- collision geometry is reasonable
- a small arm motion succeeds

### Sequence changes

- two-waypoint PTP sequence
- STOP / RESUME
- one input condition
- one digital output action
- one gripper action

### Automatic-placement changes

- target discovery
- matching-jig scan
- one automatic placement
- used-jig filtering
- occupied-target filtering

### LinkAttacher changes

- attach
- repeated detach
- multiple pick/place cycles without `gzserver` crash

## Important invariants

Please preserve these unless the change intentionally redesigns them and documents the migration:

- dynamic target suffix: `_target_link`
- separate `placed_jig_models` and `filled_placement_targets`
- gripper execution barrier in sequences
- STOP cancellation of actual active motion goals
- queued Gazebo detach executed on the simulation update thread
- configured TCP separate from the `link_6 -> gripper_base` transform

## Coding style

For Python:

- prefer readable functions over large inline blocks
- avoid target-specific hard-coded IK solutions when a generic rule can solve the problem
- keep ROS action/service error handling explicit
- use meaningful log messages for failures that need field diagnosis

For C++/Gazebo plugin code:

- respect simulator-thread ownership for physics/joint operations
- document any cross-thread queue or mutex behavior
- avoid changing plugin behavior without a repeated attach/detach test

## Documentation

If a change affects how the project is installed, launched, configured, or extended, update the corresponding document in `docs/` in the same pull request.

## Safety

This repository is a simulation-validated engineering prototype, not a certified industrial safety system. Do not represent simulation behavior as proof of safe physical operation. Hardware contributions should clearly separate ordinary control logic from safety-rated functions and commissioning requirements.
