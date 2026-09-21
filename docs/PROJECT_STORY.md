# Project Story

## From a manufacturing problem to a complete robotics stack

This project was developed during a robotics/mechatronics internship project around a real wire-harness manufacturing challenge.

The underlying problem was product changeover. Traditional harness assembly can rely on dedicated boards and fixed jig layouts. New variants create extra board fabrication, storage and reconfiguration work.

The project explored a different idea: use a reusable workboard and let a robot place the required modular jigs automatically.

That simple application idea grew into a full robotics development workflow.

## Phase 1 — Mechanical concept

The first challenge was not ROS or software. It was designing a robot that could physically reach the work area and carry a meaningful payload.

The arm evolved through:

- multiple link geometries
- material studies
- ribbed and unribbed structures
- torque calculations
- joint-housing design
- FEA
- custom reducer research
- final integrated actuator selection

The target concept became a six-axis arm with roughly 1.5 m reach and a 10 kg payload target.

## Phase 2 — Gearbox and actuator decisions

Precision reducers were difficult to source locally, so the project initially investigated custom cycloidal gearboxes and later planetary/belt concepts.

The gearbox work was useful because it exposed the manufacturing difficulty hidden behind high-ratio robot joints: backlash, heat treatment, bearing stiffness, surface finish and tolerance control.

The final design moved to integrated servo-reducer joint modules, allowing the project to progress from component research to full-system integration.

## Phase 3 — CAD becomes a digital robot

The SolidWorks assembly was prepared for URDF export using explicit coordinate systems and revolute-joint axes.

The digital model then carried the robot into the ROS ecosystem:

```text
SolidWorks
  -> URDF + meshes
  -> ROS 2
  -> Gazebo
  -> MoveIt
  -> RViz
```

This stage connected mechanical engineering decisions directly to kinematics, collision geometry and simulation.

## Phase 4 — Motion planning and control

After the robot model was stable in Gazebo, MoveIt 2 and ros2_control were integrated.

The project progressed from simple joint motion to:

- Cartesian jogging
- joint jogging
- speed override
- TCP handling
- multiple jogging frames
- saved poses
- sequence programming
- PTP and LIN experimentation
- digital I/O behavior
- gripper trajectory control

The controller eventually became a custom Python/Tkinter operator interface rather than a collection of terminal commands.

## Phase 5 — Autonomous jig placement

The most advanced software phase was automatic jig placement.

The GUI learned to:

- discover free targets from TF
- infer required jig size from target naming
- scan for a compatible jig
- reject already-used jig models
- compute runtime IK
- choose a good IK branch
- pick and attach the jig
- move to the target
- release/detach
- validate placement
- mark target occupancy
- continue through the full table

At that point, the project was no longer just a robot model. It was a simulated manufacturing application.

## Phase 6 — Reliability engineering

Some of the most useful work came from failures.

Examples include:

- duplicate MoveIt servers
- stale TCP data during pickup
- gripper timeouts caused by slow simulation time
- poor IK branches causing wrist flips
- reusing jigs that had already been placed
- unsafe Gazebo detach behavior causing native simulator crashes
- STOP behavior that initially did not interrupt motion immediately

Each failure forced the architecture to become more robust.

## What the project demonstrates

The project combines several disciplines that are often taught separately:

- mechanical design
- FEA
- actuator sizing
- kinematics
- robot description formats
- ROS 2
- motion planning
- simulation
- controller integration
- GUI development
- autonomous state logic
- debugging across a distributed robotics stack

## Scope and honesty

The project reached a strong simulation-validated stage. It did **not** reach complete physical commissioning of the six-axis arm during the internship period because the selected joint modules had a long procurement lead time.

That distinction is important: the repository represents a real engineering development project and a functioning simulated robot application, not a certified production robot.

## Personal contribution

This repository documents my robotics/mechatronics work on the project and the technical system I helped develop during the internship. The broader internship project involved a project team and supervision, while this repository focuses specifically on the robot design, ROS 2 stack, simulation, GUI and automatic-placement engineering work represented here.
