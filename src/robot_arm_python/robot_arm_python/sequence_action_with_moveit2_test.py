#!/usr/bin/env python3

import json
from pathlib import Path
from threading import Thread
from time import sleep

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
)

from pymoveit2 import MoveIt2


SAVED_POSES_FILE = Path.home() / "robot_arm_saved_poses.json"

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def create_joint_constraints(joint_positions):
    constraints = Constraints()

    for joint_name, position in zip(
        JOINT_NAMES,
        joint_positions,
    ):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = float(position)
        constraint.tolerance_above = 0.001
        constraint.tolerance_below = 0.001
        constraint.weight = 1.0

        constraints.joint_constraints.append(constraint)

    return constraints


def main():
    rclpy.init()

    node = Node("sequence_action_with_moveit2_test")

    node.set_parameters(
        [
            rclpy.parameter.Parameter(
                "use_sim_time",
                value=True,
            )
        ]
    )

    moveit_callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name="base_link",
        end_effector_name="link_6",
        group_name="arm",
        callback_group=moveit_callback_group,
    )

    moveit2.planner_id = "RRTConnectkConfigDefault"
    moveit2.max_velocity = 0.10
    moveit2.max_acceleration = 0.10

    sequence_callback_group = ReentrantCallbackGroup()

    action_client = ActionClient(
        node,
        MoveGroupSequence,
        "/sequence_move_group",
        callback_group=sequence_callback_group,
    )

    executor = MultiThreadedExecutor(
        num_threads=6
    )

    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )

    executor_thread.start()

    print("MoveIt2 created on shared node.")
    print("Waiting for /sequence_move_group...")

    if not action_client.wait_for_server(
        timeout_sec=10.0
    ):
        print(
            "FAILED: /sequence_move_group "
            "is unavailable."
        )
        rclpy.shutdown()
        return

    print("Action server is available.")

    try:
        with SAVED_POSES_FILE.open(
            "r",
            encoding="utf-8",
        ) as file:
            saved_poses = json.load(file)

    except Exception as error:
        print(
            f"FAILED to load saved poses: {error}"
        )
        rclpy.shutdown()
        return

    print()
    print("Available saved poses:")

    for name in saved_poses:
        print(f"  {name}")

    print()

    names_text = input(
        "Enter sequence pose names separated by commas: "
    )

    sequence_names = [
        name.strip()
        for name in names_text.split(",")
        if name.strip()
    ]

    if not sequence_names:
        print("No poses entered.")
        rclpy.shutdown()
        return

    for name in sequence_names:
        if name not in saved_poses:
            print(
                f'FAILED: Pose "{name}" '
                "does not exist."
            )
            rclpy.shutdown()
            return

        joints = saved_poses[name].get("joints")

        if joints is None or len(joints) != 6:
            print(
                f'FAILED: Pose "{name}" does not '
                "contain six saved joint positions."
            )
            rclpy.shutdown()
            return

    run_number = 0

    while rclpy.ok():
        run_number += 1

        input(
            f"\nPress ENTER to send run "
            f"{run_number} "
            "(Ctrl+C to stop)..."
        )

        goal = MoveGroupSequence.Goal()

        for index, name in enumerate(
            sequence_names
        ):
            joints = saved_poses[name]["joints"]

            request = MotionPlanRequest()
            request.group_name = "arm"
            request.pipeline_id = (
                "pilz_industrial_motion_planner"
            )
            request.planner_id = "PTP"
            request.num_planning_attempts = 1
            request.allowed_planning_time = 10.0
            request.max_velocity_scaling_factor = 0.10
            request.max_acceleration_scaling_factor = 0.10

            request.goal_constraints.append(
                create_joint_constraints(joints)
            )

            item = MotionSequenceItem()
            item.req = request
            item.blend_radius = 0.0

            goal.request.items.append(item)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        print(
            f"[RUN {run_number}] "
            "before send_goal_async()"
        )

        send_future = (
            action_client.send_goal_async(goal)
        )

        print(
            f"[RUN {run_number}] "
            "send_goal_async() returned future"
        )

        while not send_future.done():
            sleep(0.05)

        print(
            f"[RUN {run_number}] "
            "SEND FUTURE DONE"
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            print(
                f"[RUN {run_number}] "
                "FAILED: goal handle is None"
            )
            continue

        print(
            f"[RUN {run_number}] goal accepted = "
            f"{goal_handle.accepted}"
        )

        if not goal_handle.accepted:
            continue

        result_future = (
            goal_handle.get_result_async()
        )

        print(
            f"[RUN {run_number}] "
            "waiting for result future..."
        )

        while not result_future.done():
            sleep(0.05)

        print(
            f"[RUN {run_number}] "
            "RESULT FUTURE DONE"
        )

        wrapped_result = (
            result_future.result()
        )

        print(
            f"[RUN {run_number}] "
            f"action status = "
            f"{wrapped_result.status}"
        )

        print(
            f"[RUN {run_number}] "
            f"MoveIt error code = "
            f"{wrapped_result.result.response.error_code.val}"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDiagnostic stopped.")
