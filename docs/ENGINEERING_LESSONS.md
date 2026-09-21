# Engineering Lessons from the Project

This project was valuable not because every idea worked immediately, but because it required mechanical, simulation and software problems to be solved across several layers of a robotics stack.

## 1. Robot design is a system problem

A payload number by itself is not enough to size a robot joint. The joint must also carry downstream links, actuators, wrist components, gripper mass and dynamic loading.

That is why J2 became the governing axis even though a simple payload-only gravity calculation looked much smaller than the final dynamic torque requirement.

## 2. Strength is not the same as stiffness

One FEA study could look acceptable from a yield-stress perspective while the TCP deflection was still too large.

For a long robotic arm, stiffness matters directly to positioning accuracy. The project therefore had to evaluate both stress and displacement rather than treating factor of safety as the only structural metric.

## 3. Local stress concentrations need geometric fixes

Very high stress at a sharp housing transition did not mean the entire link was overloaded. Probe results showed stress falling rapidly away from the transition.

The useful response was not simply to make every tube thicker. Better fixes were larger fillets, smoother geometry and better local load paths.

## 4. A custom gearbox can become a project by itself

The custom cycloidal reducer work was technically interesting, but it also demonstrated how quickly precision gearbox development can consume a schedule.

Interference, bearing stiffness, heat treatment, surface finish, backlash and manufacturing tolerances all matter. Moving to integrated joint modules reduced risk and allowed the project to focus on the complete robot system.

## 5. CAD coordinate systems matter later

Joint axes and coordinate frames defined in SolidWorks directly affected URDF behavior, MoveIt kinematics and Gazebo motion.

Good coordinate-system discipline early in the CAD model saved large amounts of debugging later.

## 6. Simulation failures can come from different layers

A motion failure could originate from:

- GUI state
- stale pose data
- TF
- IK branch selection
- MoveIt planning
- trajectory timing
- ros2_control
- Gazebo physics
- plugin threading

The only reliable way to debug the system was to identify which layer actually failed before changing parameters.

## 7. Stale GUI state can cause real control logic bugs

The jig-acquisition logic initially used a periodically refreshed TCP pose. Immediately after a descent, the cached pose could still describe the higher approach point, making the robot appear too far from the jig.

The fix was to read the live pose synchronously at the moment the pickup-distance decision was made.

## 8. Simulation time and wall-clock time are not the same

Gazebo sometimes ran with a real-time factor well below 1.0. A simulated trajectory that looked short could take considerably longer in wall-clock time.

This explained gripper-controller results that arrived after the GUI's original timeout even though the controller itself had executed correctly.

## 9. Valid IK is not always good IK

The first mathematically valid IK solution is not necessarily the best motion for a real robot.

The project encountered solutions with large wrist rotations. Evaluating several seeds and choosing the branch closest to the current joint configuration produced much more natural motion without introducing target-specific hacks.

## 10. Track state explicitly in autonomous routines

The automatic system needed two different state sets:

- which physical jig models had already been used
- which target locations had already been filled

Without both, a full-table routine could reuse a placed jig or revisit an occupied target.

## 11. Threading matters in simulator plugins

The most severe Gazebo stability problem was a native `gzserver` crash during link detach.

The important lesson was that changing Gazebo physics/joint state from a ROS callback thread can be unsafe. Queuing the request and executing `Joint::Detach()` from Gazebo's own update thread stabilized the behavior.

## 12. STOP should mean STOP

An operator-facing STOP control is not useful if it only prevents the next command while allowing the current long trajectory to complete.

The final automatic-stop behavior cancels both the MoveIt-level goal and the active controller trajectory so the robot can interrupt motion during the current step.

## 13. Preserve known-good checkpoints

As the GUI accumulated manual jogging, TCP logic, sequences, I/O, gripper control and autonomous placement, large refactors became risky.

Dated checkpoints made it possible to compare regressions without losing newer features.

## 14. Build the simplest version that proves the architecture

The physical robot was not completed within the project timeframe, but the project still validated a substantial architecture:

- CAD-to-URDF
- planning
- simulation
- control
- GUI
- gripper
- autonomous workcell logic

That creates a much stronger basis for future hardware integration than starting directly with unvalidated hardware and software at the same time.
