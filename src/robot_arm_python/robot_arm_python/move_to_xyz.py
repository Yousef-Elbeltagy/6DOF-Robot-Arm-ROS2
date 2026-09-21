#!/usr/bin/env python3

from math import asin, atan2, cos, degrees, radians, sin, sqrt
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node

from pymoveit2 import MoveIt2
from tf2_ros import Buffer, TransformListener


MAXIMUM_STEP_DISTANCE = 0.10
MAXIMUM_ANGLE_CHANGE = 15.0


def quaternion_to_rpy(qx, qy, qz, qw):
    sin_roll = 2.0 * (qw * qx + qy * qz)
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = asin(sin_pitch)

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(sin_yaw, cos_yaw)

    return degrees(roll), degrees(pitch), degrees(yaw)


def rpy_to_quaternion(roll_degrees, pitch_degrees, yaw_degrees):
    roll = radians(roll_degrees)
    pitch = radians(pitch_degrees)
    yaw = radians(yaw_degrees)

    cr = cos(roll / 2.0)
    sr = sin(roll / 2.0)
    cp = cos(pitch / 2.0)
    sp = sin(pitch / 2.0)
    cy = cos(yaw / 2.0)
    sy = sin(yaw / 2.0)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return [qx, qy, qz, qw]


def angle_difference(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def read_robot_pose(tf_buffer):
    transform = tf_buffer.lookup_transform(
        "base_link",
        "link_6",
        rclpy.time.Time(),
        timeout=Duration(seconds=10.0),
    )

    position = transform.transform.translation
    rotation = transform.transform.rotation

    roll, pitch, yaw = quaternion_to_rpy(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )

    return position, roll, pitch, yaw


def main():
    rclpy.init()

    node = Node("move_to_xyz")
    node.set_parameters(
        [rclpy.parameter.Parameter("use_sim_time", value=True)]
    )

    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ],
        base_link_name="base_link",
        end_effector_name="link_6",
        group_name="arm",
        callback_group=callback_group,
    )

    moveit2.planner_id = "RRTConnectkConfigDefault"
    moveit2.max_velocity = 0.10
    moveit2.max_acceleration = 0.10

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    print("\n=====================================")
    print(" Robot XYZ + Orientation Controller")
    print("=====================================")
    print("XYZ values are in metres.")
    print("Roll, pitch and yaw are in degrees.")
    print("Maximum movement: 0.10 m")
    print("Maximum angle change: 15 degrees")
    print("Type q instead of X to quit.\n")

    try:
        while rclpy.ok():
            try:
                current_position, current_roll, current_pitch, current_yaw = (
                    read_robot_pose(tf_buffer)
                )
            except Exception as error:
                print(f"\nCould not read the robot pose: {error}")
                continue

            print("\nCurrent tool pose:")
            print(f"X     = {current_position.x:.3f}")
            print(f"Y     = {current_position.y:.3f}")
            print(f"Z     = {current_position.z:.3f}")
            print(f"Roll  = {current_roll:.1f} degrees")
            print(f"Pitch = {current_pitch:.1f} degrees")
            print(f"Yaw   = {current_yaw:.1f} degrees")

            x_input = input("\nEnter new X, or q to quit: ").strip()

            if x_input.lower() == "q":
                print("Controller closed.")
                break

            try:
                x = float(x_input)
                y = float(input("Enter new Y: "))
                z = float(input("Enter new Z: "))

                roll = float(input("Enter new Roll: "))
                pitch = float(input("Enter new Pitch: "))
                yaw = float(input("Enter new Yaw: "))
            except ValueError:
                print("\nERROR: Please enter valid numbers.")
                continue

            dx = x - current_position.x
            dy = y - current_position.y
            dz = z - current_position.z
            movement_distance = sqrt(dx**2 + dy**2 + dz**2)

            roll_change = abs(angle_difference(roll, current_roll))
            pitch_change = abs(angle_difference(pitch, current_pitch))
            yaw_change = abs(angle_difference(yaw, current_yaw))
            maximum_angle_change = max(
                roll_change,
                pitch_change,
                yaw_change,
            )

            print(f"\nMovement distance: {movement_distance:.3f} m")
            print(f"Maximum angle change: {maximum_angle_change:.1f} degrees")

            if movement_distance > MAXIMUM_STEP_DISTANCE:
                print("\nSAFETY REJECTION:")
                print("The requested XYZ movement is greater than 0.10 m.")
                continue

            if maximum_angle_change > MAXIMUM_ANGLE_CHANGE:
                print("\nSAFETY REJECTION:")
                print("The requested orientation change is greater than 15 degrees.")
                continue

            confirm = input(
                "\nExecute this XYZ and orientation target? [y/n]: "
            ).strip().lower()

            if confirm != "y":
                print("Movement cancelled.")
                continue

            quaternion = rpy_to_quaternion(roll, pitch, yaw)

            print("\nPlanning and executing movement...")

            moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=quaternion,
                cartesian=False,
            )

            controller_success = moveit2.wait_until_executed()

            try:
                final_position, final_roll, final_pitch, final_yaw = (
                    read_robot_pose(tf_buffer)
                )

                position_error = sqrt(
                    (x - final_position.x) ** 2
                    + (y - final_position.y) ** 2
                    + (z - final_position.z) ** 2
                )

                orientation_error = max(
                    abs(angle_difference(roll, final_roll)),
                    abs(angle_difference(pitch, final_pitch)),
                    abs(angle_difference(yaw, final_yaw)),
                )

                print(f"\nFinal position error: {position_error:.4f} m")
                print(
                    f"Final orientation error: "
                    f"{orientation_error:.2f} degrees"
                )

                measured_success = (
                    position_error <= 0.005
                    and orientation_error <= 1.0
                )

                if controller_success or measured_success:
                    print("\nSUCCESS: Target pose reached.")

                    if not controller_success:
                        print(
                            "The controller reported ABORTED, but the "
                            "measured pose is within tolerance."
                        )
                else:
                    print("\nFAILED: Target pose was not reached.")
                    print("Try a smaller movement.")

            except Exception as error:
                print(f"\nCould not verify the final pose: {error}")

    except KeyboardInterrupt:
        print("\nController stopped.")

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
