# Documentation Index

Use this page as the map for the project documentation.

## Start here

| Goal | Document |
|---|---|
| Get the project running quickly | [../QUICKSTART.md](../QUICKSTART.md) |
| Reproduce the project from a clean machine | [REPRODUCE_PROJECT.md](REPRODUCE_PROJECT.md) |
| Continue development from the current state | [CONTINUE_DEVELOPMENT.md](CONTINUE_DEVELOPMENT.md) |
| Diagnose runtime/build problems | [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |
| Inspect native/neutral CAD | [../cad/README.md](../cad/README.md) |
| See planned next engineering phases | [../ROADMAP.md](../ROADMAP.md) |
| Review public project milestones | [../CHANGELOG.md](../CHANGELOG.md) |
| Contribute changes | [../CONTRIBUTING.md](../CONTRIBUTING.md) |

## Engineering documentation

| Topic | Document |
|---|---|
| Project evolution | [PROJECT_STORY.md](PROJECT_STORY.md) |
| Mechanical design | [MECHANICAL_DESIGN.md](MECHANICAL_DESIGN.md) |
| Major components / actuator BOM | [BOM.md](BOM.md) |
| ROS 2 / MoveIt / Gazebo architecture | [SOFTWARE_ARCHITECTURE.md](SOFTWARE_ARCHITECTURE.md) |
| Sequence Controller | [SEQUENCE_PROGRAMMING.md](SEQUENCE_PROGRAMMING.md) |
| Automatic jig placement | [AUTOMATIC_JIG_PLACEMENT.md](AUTOMATIC_JIG_PLACEMENT.md) |
| Engineering lessons | [ENGINEERING_LESSONS.md](ENGINEERING_LESSONS.md) |
| Demonstrated results and current status | [RESULTS.md](RESULTS.md) |
| Screenshots | [GALLERY.md](GALLERY.md) |
| Setup details | [SETUP.md](SETUP.md) |

## Mechanical source packages

The `cad/` directory contains:

- native SolidWorks archives for the robot/gripper, jigs, and workcell tables,
- a neutral STEP archive containing the complete workcell and separate component STEP exports.

See [../cad/README.md](../cad/README.md) before modifying or redistributing mechanical source files.

## Recommended reading order for a new developer

```text
QUICKSTART
    ↓
REPRODUCE_PROJECT
    ↓
SOFTWARE_ARCHITECTURE
    ↓
SEQUENCE_PROGRAMMING
    ↓
AUTOMATIC_JIG_PLACEMENT
    ↓
CONTINUE_DEVELOPMENT
    ↓
TROUBLESHOOTING as needed
```

For mechanical continuation, also read `MECHANICAL_DESIGN.md`, `BOM.md`, and `../cad/README.md` before changing link geometry or actuator selection.

## Status language

The public repository represents a **simulation-validated engineering prototype**. Do not describe the project as a fully commissioned or certified industrial/collaborative robot unless future physical testing and safety validation support that claim.
