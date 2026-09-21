# Mechanical Design

This project started as a mechanical robot-arm design problem before it became a ROS 2 software project.

## Design target

The main concept was a 6-axis articulated arm for a flexible wire-harness manufacturing cell.

- Target reach: approximately **1.5 m**
- Target payload: approximately **10 kg**
- Architecture: 6 revolute joints
- Main structural material: **Aluminum 6061-T6**
- Manufacturing goal: geometry that could realistically be produced and assembled

## Joint architecture

| Joint | Main role |
|---|---|
| J1 | Base rotation |
| J2 | Shoulder elevation |
| J3 | Elbow positioning |
| J4 | Wrist orientation |
| J5 | Wrist orientation |
| J6 | Tool/flange orientation |

J2 became the governing mechanical joint because it carries the payload together with the mass of the downstream links, wrist, gripper and actuators.

## Link design evolution

The arm went through several iterations. Early designs explored thicker cylindrical shells and internal ribs. Later iterations moved toward lighter hollow links, with wall thickness and local joint-housing geometry carrying the structural load.

The final manufacturing direction avoided internal ribs after fabrication feedback showed that the ribbed concept would be unnecessarily difficult to manufacture.

Instead, stiffness improvements were focused on:

- increasing local wall thickness where needed
- larger diameters where practical
- smoother transitions between links and joint housings
- larger fillets at high-stress transitions
- stronger local actuator mounting regions

## Material selection

6061-T6 aluminum was selected for the main moving links because of its combination of:

- low density
- good strength-to-weight ratio
- corrosion resistance
- machinability
- practical availability

Other materials considered during development included 7075-T6, stainless steel and alloy steels for localized high-stress parts such as shafts and pins.

## Torque model

The final dynamic peak torque values used in the engineering presentation were approximately:

| Joint | Dynamic peak torque |
|---|---:|
| J1 | 115.58 N·m |
| J2 | **411.90 N·m** |
| J3 | 182.49 N·m |
| J4 | 35.64 N·m |
| J5 | 15.50 N·m |
| J6 | 5.10 N·m |

This made J2 the governing axis for actuator sizing.

## Gearbox research

Several drivetrain concepts were investigated before the final actuator selection.

### Custom cycloidal reducers

A custom cycloidal gearbox was explored because precision robot reducers were difficult to source locally.

The work included:

- single- and multi-stage ratio studies
- dual cycloidal discs
- eccentric shaft design
- roller/pin geometry
- interference checking
- bearing-pocket sizing
- material selection and heat-treatment considerations

The concept was ultimately not selected because the machining precision, backlash control, heat treatment and development schedule created too much project risk.

### Planetary + timing belt

A planetary gearbox with optional timing-belt pre-reduction was also studied. This provided more realistic local manufacturability but still added packaging and compliance complexity.

### Final integrated joint modules

The final concept moved to integrated robotic joint modules containing the motor, encoder, brake and reducer in one unit.

| Joints | Module | Rated torque | Peak/start-stop torque |
|---|---|---:|---:|
| J1-J2 | TD-110-170 | 328 N·m | 702 N·m |
| J3 | TD-100-142 | 169 N·m | 411 N·m |
| J4-J6 | TD-70-90 | 50 N·m | 102 N·m |

This dramatically simplified the mechanical integration compared with designing six custom reducer assemblies.

## FEA lessons

Structural analysis showed two distinct engineering challenges:

1. **Local stress concentration** around joint transitions, especially near J2.
2. **Global deflection** of the long links.

One study showed a high local peak around the J2 transition, while stress dropped substantially a short distance away. That indicated a geometry-driven stress concentration rather than uniform tube failure.

Another displacement study showed end-effector deflection on the order of a few centimetres under the assumed loading case, indicating that stiffness was a more important issue than simply increasing material strength.

The main design lessons were therefore:

- do not optimize only for yield stress
- examine displacement and stiffness at the TCP
- use smooth load paths
- avoid sharp housing transitions
- increase local thickness only where it improves the load path

## CAD-to-robot-model workflow

SolidWorks was used to define:

- link geometry
- joint axes
- reference coordinate systems
- mass properties
- centers of mass
- inertias
- visual/collision meshes

These definitions were then carried into the URDF model used by ROS 2, Gazebo and MoveIt.

## Status

The mechanical design reached a simulation-ready and actuator-integrated CAD stage. The complete physical robot was not commissioned during the internship/project period because the selected integrated joint modules had a long procurement lead time.
