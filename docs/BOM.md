# Major Components / BOM Reference

This is a reproducibility-oriented bill of materials for the major robot concepts represented in the repository. It is not a purchasing quotation and does not claim that every physical component was procured or commissioned.

## Robot design targets

| Item | Target / design basis |
|---|---|
| Degrees of freedom | 6 revolute joints |
| Reach | approximately 1.5 m |
| Payload target | approximately 10 kg |
| Higher design-load checks | approximately 15 kg used in selected sizing/safety calculations |
| Primary structural material | Aluminum 6061-T6 |
| High-stress shaft/pin material considered | 4140 / 42CrMo4 steel |
| Robot status | simulation-validated prototype; physical arm not fully commissioned |

## Final integrated actuator concept

| Joint | Module | Rated torque | Peak/start-stop torque | Rated power | Reduction | Approx. module mass |
|---|---|---:|---:|---:|---:|---:|
| J1 | TD-110-170 | 328 N·m | 702 N·m | 1500 W | 101:1 | 7.61 kg |
| J2 | TD-110-170 | 328 N·m | 702 N·m | 1500 W | 101:1 | 7.61 kg |
| J3 | TD-100-142 | 169 N·m | 411 N·m | 1000 W | 101:1 | 4.46 kg |
| J4 | TD-70-90 | 50 N·m | 102 N·m | 500 W | 101:1 | 1.60 kg |
| J5 | TD-70-90 | 50 N·m | 102 N·m | 500 W | 101:1 | 1.60 kg |
| J6 | TD-70-90 | 50 N·m | 102 N·m | 500 W | 101:1 | 1.60 kg |

The selected modules were intended to simplify the final mechanical integration compared with continuing to develop custom high-ratio reducers for every joint.

## Structural components

| Component | Reference design direction |
|---|---|
| Long links | Hollow cylindrical aluminum shells |
| Later lightweight link section explored | approximately 80 mm OD, 3 mm wall, 74 mm ID |
| Earlier heavier section explored | approximately 110 mm OD, 5 mm wall |
| Internal ribs | explored early, removed from final manufacturing direction |
| Local reinforcement | thicker housings / transitions / bearing and reducer interfaces |
| Joint shafts / pins | high-strength steel concept such as 4140 / 42CrMo4 |
| Fasteners | high-strength metric fasteners sized per final joint design |

Exact wall thickness and local reinforcement should be revalidated against the final actuator packaging and desired stiffness before physical manufacture.

## End effector

The simulated/digital system includes a custom gripper integrated into the robot URDF.

Key requirements for a physical version:

- compatible mounting flange
- sufficient jaw travel for the jig geometry
- position or state feedback if sequence synchronization depends on confirmed closure
- physical grasp force validation
- repeatable TCP calibration

## Control / communication

The selected integrated actuator direction supports industrial communication options including EtherCAT / CAN-class interfaces. The repository currently represents the simulation/control architecture rather than a completed physical fieldbus implementation.

A physical build would additionally require:

- robot controller or industrial PC
- real-time-compatible fieldbus interface
- suitable 24–48 V-class power distribution as required by the selected actuators
- brake control where applicable
- emergency stop / safety architecture
- limit and reference strategy
- wiring, connectors, grounding and shielding

## Simulation software BOM

| Layer | Baseline |
|---|---|
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS 2 Humble |
| Simulation | Gazebo Classic 11 |
| Motion planning | MoveIt 2 |
| Planning pipelines | OMPL + Pilz |
| Visualization | RViz 2 |
| Execution | ros2_control + ros2_controllers |
| GUI | Python 3 + Tkinter |
| Build system | colcon / ament |

## Workcell models

The simulated application also includes:

- harness/placement table
- jig source table
- large jig models
- medium jig models
- small jig models
- target TF frames
- IFRA LinkAttacher-based simulated grasping

## Before purchasing hardware

Re-run or confirm:

1. final link masses and inertias
2. worst-case joint torque including payload and acceleration
3. structural stiffness / TCP deflection
4. actuator thermal duty cycle
5. brake requirements
6. bearing loads
7. shaft/pin stress
8. cable routing and joint rotation limits
9. electrical power budget
10. safety requirements for the actual installation

The actuator table above documents the final project direction, not a substitute for a fresh design review before manufacture.
