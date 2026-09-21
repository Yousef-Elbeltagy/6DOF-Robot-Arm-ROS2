# SolidWorks CAD Models

This directory contains the native mechanical CAD packages used for the 6-DOF robot-arm project.

## Available CAD archives

| Archive | Contents | Approx. size |
|---|---|---:|
| `Solidworks_Cad_models/full_Cobot_Solidworks_Cad.zip` | Full 6-DOF robot arm and gripper SolidWorks CAD package | 32.7 MB |
| `Solidworks_Cad_models/Cobot_Jigs_Solidworks_Cad.zip` | Jig CAD models used by the automatic placement workcell | 0.9 MB |
| `Solidworks_Cad_models/Pick_Table_Solidworks_Cad.zip` | Pick/placement table and target-related SolidWorks CAD package | 1.2 MB |

## Relationship to the ROS 2 model

The CAD models are the mechanical source geometry behind the robot and workcell that were later converted into ROS 2 / Gazebo assets.

The corresponding software models are located under `src/`, including:

- `robot_arm_with_gripper_urdf`
- `table_harness_urdf_v3`
- `pick_table_urdf`
- `large_jig_urdf`
- `medium_jig_urdf`
- `small_jig_urdf`

The robot assembly was prepared in SolidWorks with joint axes, reference geometry, mass properties, and link relationships before export to URDF and STL meshes for ROS 2.

## Mechanical design status

The published CAD represents the engineering prototype developed during the project. The system was validated extensively in simulation, but the complete physical six-axis robot was not fully commissioned during the project period because the selected integrated actuator modules had a long procurement lead time.

Important mechanical design notes:

- 6-DOF articulated robot architecture
- approximately 1.5 m design reach
- approximately 10 kg payload design target
- 6061-T6 aluminum as the main structural material
- integrated servo/reducer joint modules selected as the final actuator direction
- early internal-rib concepts were investigated, but the final manufacturing direction used simpler unribbed cylindrical shells with local reinforcement and appropriate wall thickness
- custom gripper integrated into the final robot assembly

## Opening the files

The archives contain SolidWorks CAD data. Extract each ZIP archive before opening the assemblies or parts.

For best results, keep all referenced files from an archive together after extraction so that SolidWorks can resolve assembly references correctly.

## Interoperability

Native SolidWorks files are included here because they preserve the original design intent and assembly relationships. Neutral CAD exports such as STEP can be added later if required for users who do not have SolidWorks.

## Notes for contributors

Please do not commit SolidWorks temporary/lock files such as `~$*` or backup files. Large handover archives, private company documents, supplier quotations, and internal project backups are intentionally excluded from this public repository.

For the mechanical design background, see [`../docs/MECHANICAL_DESIGN.md`](../docs/MECHANICAL_DESIGN.md).
