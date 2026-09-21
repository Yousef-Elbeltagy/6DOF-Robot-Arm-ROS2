#!/usr/bin/env python3

from math import asin, atan2, cos, degrees, radians, sin, sqrt
from threading import Thread
from time import monotonic, sleep
import tkinter as tk
import json
import yaml
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from gazebo_msgs.srv import GetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from pymoveit2 import MoveIt2
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive

MAXIMUM_STEP_DISTANCE = float("inf")
MAXIMUM_ANGLE_CHANGE = float("inf")
JOG_POSITION_STEP = 0.01
JOG_ANGLE_STEP = 5.0
STOP_DWELL_SECONDS = 1.0
JOINT_MINIMUM_DEGREES = [
    -180.0, -90.0, -135.0, -180.0, -120.0, -180.0
]
JOINT_MAXIMUM_DEGREES = [
    180.0, 180.0, 135.0, 180.0, 120.0, 180.0
]


SAVED_POSES_FILE = Path.home() / "robot_arm_saved_poses.json"
TCP_CONFIG_FILE = Path.home() / "robot_arm_tcp_config.json" 


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

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]

def quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second

    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return [-x, -y, -z, w]


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(
            quaternion,
            [vector[0], vector[1], vector[2], 0.0],
        ),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]

def angle_difference(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def axis_angle_quaternion(axis, angle_degrees):
    half_angle = radians(angle_degrees) / 2.0
    sine = sin(half_angle)

    components = {
        "X": [sine, 0.0, 0.0, cos(half_angle)],
        "Y": [0.0, sine, 0.0, cos(half_angle)],
        "Z": [0.0, 0.0, sine, cos(half_angle)],
    }

    return components[axis]


class RobotGUI:
    def __init__(self, root, node, moveit2, tf_buffer):
        self.root = root
        self.node = node
        self.moveit2 = moveit2
        self.tf_buffer = tf_buffer

        self.sequence_node = Node(
            "robot_arm_gui_sequence_client"
        )
        self.sequence_callback_group = ReentrantCallbackGroup()

        self.sequence_action_client = ActionClient(
            self.sequence_node,
            MoveGroupSequence,
            "/sequence_move_group",
            callback_group=self.sequence_callback_group,
        )

        self.sequence_executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=2
        )
        self.sequence_executor.add_node(self.sequence_node)

        self.sequence_executor_thread = Thread(
            target=self.sequence_executor.spin,
            daemon=True,
        )
        self.sequence_executor_thread.start()

        self.planning_scene_publisher = self.node.create_publisher(
            PlanningScene,
            "/planning_scene",
            10,
        )
        self.sequence_goal_handle = None
        self.gripper_action_client = ActionClient(
            self.sequence_node,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            callback_group=self.sequence_callback_group,
        )
        self.gripper_goal_handle = None
        self.gripper_motion_active = False

        self.active_jig_model = None
        self.active_jig_link = None
        self.attached_jig_model = None
        self.attached_jig_link = None

        # Jigs that have already been released at a placement
        # target during this simulation run.
        self.placed_jig_models = set()

        # Placement targets that are already occupied during this
        # simulation run. Prevents placing a second jig on the same target.
        self.filled_placement_targets = set()

        self.jig_candidates = [
            (f"large_jig_{index}", "large_jig_link")
            for index in range(1, 5)
        ] + [
            (f"medium_jig_{index}", "medium_jig_link")
            for index in range(1, 5)
        ] + [
            (f"small_jig_{index}", "small_jig_link")
            for index in range(1, 5)
        ]

        self.placement_targets = {
            "L1": {
                "position": (0.900, -0.350, 0.352),
                "jig_type": "large_jig_",
            },
            "M1": {
                "position": (1.131, 0.276, 0.352),
                "jig_type": "medium_jig_",
            },
            "M2": {
                "position": (0.739, 0.192, 0.352),
                "jig_type": "medium_jig_",
            },
            "S1": {
                "position": (0.899, 0.016, 0.352),
                "jig_type": "small_jig_",
            },
        }

        self.attach_link_client = self.sequence_node.create_client(
            AttachLink,
            "/ATTACHLINK",
            callback_group=self.sequence_callback_group,
        )
        self.detach_link_client = self.sequence_node.create_client(
            DetachLink,
            "/DETACHLINK",
            callback_group=self.sequence_callback_group,
        )

        self.get_entity_state_client = self.sequence_node.create_client(
            GetEntityState,
            "/gazebo/get_entity_state",
            callback_group=self.sequence_callback_group,
        )

        self.compute_ik_client = self.sequence_node.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self.sequence_callback_group,
        )

        self.controller_cancel_client = self.node.create_client(
            CancelGoal,
            "/arm_controller/follow_joint_trajectory/_action/cancel_goal",
            callback_group=self.sequence_callback_group,
        )

        self.current_pose = None
        self.current_joint_positions = None
        self.current_gripper_position = None
        self.joint_state_subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )
        self.target_initialized = False
        self.busy = False
        self.motion_cancel_requested = False
        self.jog_buttons = []
        self.saved_poses = {}
        self.tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tcp_window = None
        self.tcp_vars = {}
        self.load_tcp_config()
                
        self.sequence = []
        self.sequence_blend_radii = []
        self.sequence_motion_types = []
        self.sequence_input_numbers = []
        self.sequence_input_states = []
        self.sequence_input_timings = []
        self.sequence_output_timings = []

        self.digital_inputs = {
            number: tk.BooleanVar(value=False)
            for number in range(1, 7)
        }

        self.sequence_row_widgets = []
        self.selected_sequence_index = None
        self.sequence_running = False
        self.stop_sequence_requested = False
        self.sequence_paused = False
        self.sequence_resume_index = 0
        self.sequence_resume_base_index = 0
        self.sequence_window = None
        self.sequence_listbox = None
        self.auto_jig_window = None
        self.auto_jig_running = False

        # Full automatic board-run state.
        self.full_auto_run_active = False
        self.full_auto_run_targets = []
        self.full_auto_run_index = 0

        # Safe automatic-operation pause/resume state.
        # STOP pauses at the next completed automatic step rather than
        # cancelling an active trajectory midway through execution.
        self.auto_pause_requested = False
        self.auto_pause_active = False

        self.waypoint_behavior = tk.StringVar(value="stop")
        self.sequence_blend_radius = tk.StringVar(value="0.001")
        self.selected_pose = tk.StringVar()
        self.control_mode = tk.StringVar(value="cartesian")
        self.jog_frame_mode = tk.StringVar(value="WORLD")
        self.speed_override = tk.StringVar(value="100%")
        self.current_joint_vars = {
            f"J{number}": tk.StringVar(value="Waiting...")
            for number in range(1, 7)
        }
        self.target_joint_vars = {
            f"J{number}": tk.StringVar()
            for number in range(1, 7)
        }
        self.current_name_labels = []
        self.current_value_labels = []
        self.current_unit_labels = []
        self.target_name_labels = []
        self.target_entries = []
        self.target_unit_labels = []
        self.jog_axis_labels = []
        self.load_saved_poses()

        self.root.title("6-DOF Robot Arm - Pick & Place Cell")
        self.root.geometry("570x1000")
        self.root.resizable(True, True)
        self.root.tk.call("tk", "scaling", 1.0)
        
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill="both", expand=True)

        self.main_canvas = tk.Canvas(
            self.main_container,
            highlightthickness=0,
        )
        self.main_scrollbar = ttk.Scrollbar(
            self.main_container,
            orient="vertical",
            command=self.main_canvas.yview,
        )

        self.scrollable_frame = ttk.Frame(self.main_canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda event: self.main_canvas.configure(
                scrollregion=self.main_canvas.bbox("all")
            ),
        )

        self.canvas_window = self.main_canvas.create_window(
            (0, 0),
            window=self.scrollable_frame,
            anchor="nw",
        )

        self.main_canvas.bind(
            "<Configure>",
            lambda event: self.main_canvas.itemconfigure(
                self.canvas_window,
                width=event.width,
            ),
        )

        self.main_canvas.configure(
            yscrollcommand=self.main_scrollbar.set
        )

        self.main_canvas.pack(
            side="left",
            fill="both",
            expand=True,
        )
        self.main_scrollbar.pack(
            side="right",
            fill="y",
        )

        self.root.bind_all(
            "<MouseWheel>",
            lambda event: self.main_canvas.yview_scroll(
                int(-event.delta / 120),
                "units",
            ),
        )

        self.current_vars = {
            name: tk.StringVar(value="Waiting...")
            for name in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        }

        self.target_vars = {
            name: tk.StringVar()
            for name in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        }

        self.status_var = tk.StringVar(value="Connecting to robot...")
        self.detected_jig_var = tk.StringVar(value="Detected Jig: none")
        self.holding_var = tk.StringVar(value="Holding: none")
        self.last_placement_status = None

        self.create_widgets()

        self.root.after(1500, self.add_ground_collision)
        self.root.after(3000, self.add_ground_collision)
        self.root.after(3500, self.add_table_collision)
        self.root.after(4000, self.add_pick_table_collision)

        self.refresh_pose()
    def request_gripper_position(self, target_position, status_message):
        if self.busy:
            return

        self.set_busy(True)
        self.gripper_motion_active = True
        self.status_var.set(status_message)

        Thread(
            target=self.execute_gripper_position,
            args=(target_position,),
            daemon=True,
        ).start()

    def execute_gripper_position(self, target_position):
        try:
            if not self.gripper_action_client.wait_for_server(
                timeout_sec=3.0
            ):
                message = (
                    "GRIPPER ERROR: Controller action server "
                    "is not available."
                )
                self.root.after(
                    0,
                    lambda: self.finish_gripper_motion(message),
                )
                return

            if self.motion_cancel_requested:
                self.root.after(
                    0,
                    lambda: self.finish_gripper_motion(
                        "Gripper movement stopped by user."
                    ),
                )
                return

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ["left_gear_joint"]

            point = JointTrajectoryPoint()
            point.positions = [float(target_position)]
            point.time_from_start = Duration(seconds=1).to_msg()
            goal.trajectory.points = [point]

            send_future = self.gripper_action_client.send_goal_async(goal)
            send_deadline = monotonic() + 3.0

            while (
                rclpy.ok()
                and not send_future.done()
                and monotonic() < send_deadline
            ):
                sleep(0.02)

            if not send_future.done():
                if (
                    self.current_gripper_position is not None
                    and abs(
                        target_position
                        - self.current_gripper_position
                    ) < 0.01
                ):
                    if abs(target_position - 1.75) < 1e-6:
                        message = (
                            "SUCCESS: Gripper fully opened to 1.75 rad."
                        )
                    else:
                        message = (
                            "SUCCESS: Gripper fully closed to 0.0 rad."
                        )

                    self.root.after(
                        0,
                        lambda: self.finish_gripper_motion(message),
                    )
                    return

                raise RuntimeError(
                    "Gripper goal response timed out."
                )

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("Gripper controller rejected the goal.")

            self.gripper_goal_handle = goal_handle

            is_open_action = abs(float(target_position) - 1.75) < 1e-6
            is_close_action = abs(float(target_position) - 0.04) < 1e-6

            manual_detach_message = None

            if is_open_action:
                detach_success, manual_detach_message = (
                    self.call_link_attacher(attach=False)
                )

                if not detach_success:
                    if (
                        manual_detach_message
                        == "No jig is currently marked as attached."
                    ):
                        manual_detach_message = (
                            "No jig was attached; gripper opened normally."
                        )
                    else:
                        goal_handle.cancel_goal_async()
                        raise RuntimeError(
                            f"DETACH failed: {manual_detach_message}"
                        )

            if self.motion_cancel_requested:
                goal_handle.cancel_goal_async()

            result_future = goal_handle.get_result_async()
            result_deadline = monotonic() + 3.0

            while (
                rclpy.ok()
                and not result_future.done()
                and monotonic() < result_deadline
            ):
                sleep(0.02)

            if self.motion_cancel_requested:
                message = "Gripper movement stopped by user."

            elif result_future.done():
                result = result_future.result().result

                if result.error_code == 0:
                    if is_open_action:
                        if manual_detach_message:
                            message = (
                                "SUCCESS: Gripper fully opened to 1.75 rad. "
                                f"{manual_detach_message}"
                            )
                        else:
                            message = (
                                "SUCCESS: Gripper fully opened to 1.75 rad."
                            )

                    elif is_close_action:
                        attach_success, attach_message = (
                            self.call_link_attacher(attach=True)
                        )

                        if not attach_success:
                            if attach_message.startswith(
                                "No jig is within pickup range."
                            ):
                                message = (
                                    "SUCCESS: Gripper fully closed to 0.04 rad. "
                                    "No jig detected; gripper closed normally."
                                )
                            else:
                                message = (
                                    "GRIPPER ERROR: "
                                    f"ATTACH failed: {attach_message}"
                                )
                        else:
                            message = (
                                "SUCCESS: Gripper fully closed to 0.04 rad. "
                                f"{attach_message}"
                            )

                    else:
                        message = (
                            f"SUCCESS: Gripper moved to "
                            f"{float(target_position):.2f} rad."
                        )
                else:
                    message = (
                        "GRIPPER FAILED: "
                        f"{result.error_string or 'controller error'}"
                    )

            elif (
                self.current_gripper_position is not None
                and abs(
                    target_position
                    - self.current_gripper_position
                ) < 0.01
            ):
                goal_handle.cancel_goal_async()

                if abs(target_position - 1.75) < 1e-6:
                    message = (
                        "SUCCESS: Gripper fully opened to 1.75 rad."
                    )
                else:
                    message = (
                        "SUCCESS: Gripper fully closed to 0.0 rad."
                    )

            else:
                raise RuntimeError(
                    "Gripper result timed out before reaching target."
                )

        except Exception as error:
            message = f"GRIPPER ERROR: {error}"

        self.root.after(
            0,
            lambda: self.finish_gripper_motion(message),
        )

    def finish_gripper_motion(self, message):
        self.gripper_goal_handle = None
        self.gripper_motion_active = False
        self.finish_movement(message)

    def get_entity_position(self, entity_name):
        if not self.get_entity_state_client.wait_for_service(
            timeout_sec=3.0
        ):
            return None

        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = "world"

        future = self.get_entity_state_client.call_async(request)
        deadline = monotonic() + 3.0

        while (
            rclpy.ok()
            and not future.done()
            and monotonic() < deadline
        ):
            sleep(0.02)

        if not future.done():
            return None

        response = future.result()

        if response is None or not response.success:
            return None

        position = response.state.pose.position
        return (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def find_nearest_jig(self):
        try:
            self.current_pose = self.read_pose()
        except Exception:
            pass

        if self.current_pose is None:
            return None, None, None

        gx, gy, gz = self.current_pose[:3]
        nearest_model = None
        nearest_link = None
        nearest_distance = float("inf")

        for model_name, link_name in self.jig_candidates:
            jig_position = self.get_entity_position(model_name)

            if jig_position is None:
                continue

            jx, jy, jz = jig_position

            distance = sqrt(
                (gx - jx) ** 2
                + (gy - jy) ** 2
                + (gz - jz) ** 2
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_model = model_name
                nearest_link = link_name

        return nearest_model, nearest_link, nearest_distance

    def evaluate_placement(self, model_name):
        jig_position = self.get_entity_position(model_name)

        if jig_position is None:
            return None, None, None

        jx, jy, jz = jig_position
        nearest_target = None
        nearest_distance = float("inf")
        correct_type = False

        for target_name, target_data in self.placement_targets.items():
            tx, ty, tz = target_data["position"]

            distance = sqrt(
                (jx - tx) ** 2
                + (jy - ty) ** 2
                + (jz - tz) ** 2
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_target = target_name
                correct_type = model_name.startswith(
                    target_data["jig_type"]
                )

        return nearest_target, nearest_distance, correct_type

    def call_link_attacher(self, attach=True):
        client = (
            self.attach_link_client
            if attach
            else self.detach_link_client
        )

        service_name = "/ATTACHLINK" if attach else "/DETACHLINK"

        if not client.wait_for_service(timeout_sec=3.0):
            return False, f"{service_name} service unavailable."

        request = (
            AttachLink.Request()
            if attach
            else DetachLink.Request()
        )

        if attach:
            model_name, link_name, distance = self.find_nearest_jig()

            print(
                "ATTACH DEBUG: current_pose=",
                self.current_pose,
                "nearest_model=",
                model_name,
                "nearest_link=",
                link_name,
                "distance=",
                distance,
                flush=True,
            )

            if model_name is None:
                return False, "No jig position could be determined."

            if distance > 0.09:
                return (
                    False,
                    f"No jig is within pickup range. "
                    f"Nearest jig is {distance:.3f} m away.",
                )

            self.active_jig_model = model_name
            self.active_jig_link = link_name

            self.root.after(
                0,
                lambda name=model_name: self.detected_jig_var.set(
                    f"Detected Jig: {name}"
                ),
            )
        else:
            model_name = self.attached_jig_model
            link_name = self.attached_jig_link

            if model_name is None or link_name is None:
                return False, "No jig is currently marked as attached."

        request.model1_name = "robot_arm_with_gripper"
        request.link1_name = "link_6"
        request.model2_name = model_name
        request.link2_name = link_name

        future = client.call_async(request)
        deadline = monotonic() + 3.0

        while (
            rclpy.ok()
            and not future.done()
            and monotonic() < deadline
        ):
            sleep(0.02)

        if not future.done():
            return False, f"{service_name} timed out."

        response = future.result()

        if response is None:
            return False, f"{service_name} returned no response."

        success = bool(response.success)

        if success:
            if attach:
                self.attached_jig_model = model_name
                self.attached_jig_link = link_name

                self.root.after(
                    0,
                    lambda name=model_name: self.holding_var.set(
                        f"Holding: {name}"
                    ),
                )
            else:
                target_name, target_distance, correct_type = (
                    self.evaluate_placement(model_name)
                )

                self.attached_jig_model = None
                self.attached_jig_link = None
                self.active_jig_model = None
                self.active_jig_link = None

                self.root.after(
                    0,
                    lambda: self.holding_var.set("Holding: none"),
                )
                self.root.after(
                    0,
                    lambda: self.detected_jig_var.set(
                        "Detected Jig: none"
                    ),
                )

                if (
                    target_name is not None
                    and target_distance is not None
                    and target_distance <= 0.06
                ):
                    if correct_type:
                        self.last_placement_status = (
                            f"PLACED CORRECTLY: {target_name}"
                        )
                    else:
                        self.last_placement_status = (
                            f"WRONG JIG TYPE AT {target_name}"
                        )
                else:
                    self.last_placement_status = (
                        "Jig released outside a valid target."
                    )

                self.root.after(
                    0,
                    lambda message=self.last_placement_status:
                        self.status_var.set(message),
                )

        return success, response.message

    def execute_sequence_gripper_action(self, target_position, action_name):
        self.gripper_motion_active = True
        self.gripper_goal_handle = None

        try:
            self.root.after(
                0,
                lambda name=action_name: self.status_var.set(
                    f"{name}..."
                ),
            )

            if not self.gripper_action_client.wait_for_server(
                timeout_sec=3.0
            ):
                return (
                    False,
                    "GRIPPER ERROR: Controller action server "
                    "is not available.",
                )

            if (
                self.stop_sequence_requested
                or self.motion_cancel_requested
            ):
                return False, "Gripper action cancelled."

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ["left_gear_joint"]

            point = JointTrajectoryPoint()
            point.positions = [float(target_position)]
            point.time_from_start = Duration(seconds=1).to_msg()
            goal.trajectory.points = [point]

            send_future = self.gripper_action_client.send_goal_async(
                goal
            )
            send_deadline = monotonic() + 3.0

            while (
                rclpy.ok()
                and not send_future.done()
                and monotonic() < send_deadline
            ):
                if (
                    self.stop_sequence_requested
                    or self.motion_cancel_requested
                ):
                    return False, "Gripper action cancelled."

                sleep(0.02)

            if not send_future.done():
                if (
                    self.current_gripper_position is not None
                    and abs(
                        float(target_position)
                        - self.current_gripper_position
                    ) < 0.01
                ):
                    if action_name == "CLOSE GRIPPER":
                        attach_success, attach_message = (
                            self.call_link_attacher(attach=True)
                        )
                        if not attach_success:
                            if attach_message.startswith(
                                "No jig is within pickup range."
                            ):
                                return True, (
                                    f"{action_name} complete. "
                                    "No jig detected; gripper closed normally."
                                )

                            return False, (
                                f"{action_name} reached position, but "
                                f"ATTACH failed: {attach_message}"
                            )

                        return True, (
                            f"{action_name} complete. "
                            f"{attach_message}"
                        )

                    if action_name == "OPEN GRIPPER":
                        detach_success, detach_message = (
                            self.call_link_attacher(attach=False)
                        )
                        if not detach_success:
                            if (
                                detach_message
                                == "No jig is currently marked as attached."
                            ):
                                return True, (
                                    f"{action_name} complete. "
                                    "No jig was attached; gripper opened normally."
                                )

                            return False, (
                                f"{action_name} reached position, but "
                                f"DETACH failed: {detach_message}"
                            )

                        return True, (
                            f"{action_name} complete. "
                            f"{detach_message}"
                        )

                    return True, f"{action_name} complete."

                return (
                    False,
                    "GRIPPER ERROR: Goal response timed out.",
                )

            goal_handle = send_future.result()

            if goal_handle is None or not goal_handle.accepted:
                return (
                    False,
                    "GRIPPER ERROR: Controller rejected the goal.",
                )

            self.gripper_goal_handle = goal_handle

            early_detach_message = None

            if action_name == "OPEN GRIPPER":
                detach_success, early_detach_message = (
                    self.call_link_attacher(attach=False)
                )

                if not detach_success:
                    if (
                        early_detach_message
                        == "No jig is currently marked as attached."
                    ):
                        early_detach_message = (
                            "No jig was attached; gripper opened normally."
                        )
                    else:
                        goal_handle.cancel_goal_async()
                        return (
                            False,
                            f"{action_name} started, but "
                            f"DETACH failed: {early_detach_message}",
                        )

            result_future = goal_handle.get_result_async()
            result_deadline = monotonic() + 8.0

            while (
                rclpy.ok()
                and not result_future.done()
                and monotonic() < result_deadline
            ):
                if (
                    self.stop_sequence_requested
                    or self.motion_cancel_requested
                ):
                    goal_handle.cancel_goal_async()
                    return False, "Gripper action cancelled."

                sleep(0.02)

            if result_future.done():
                wrapped_result = result_future.result()

                if (
                    wrapped_result is not None
                    and wrapped_result.result.error_code == 0
                ):
                    if action_name == "CLOSE GRIPPER":
                        attach_success, attach_message = (
                            self.call_link_attacher(attach=True)
                        )
                        if not attach_success:
                            if attach_message.startswith(
                                "No jig is within pickup range."
                            ):
                                return (
                                    True,
                                    f"{action_name} complete. "
                                    "No jig detected; gripper closed normally.",
                                )

                            return (
                                False,
                                f"{action_name} succeeded, but "
                                f"ATTACH failed: {attach_message}",
                            )

                        return (
                            True,
                            f"{action_name} complete. "
                            f"{attach_message}",
                        )

                    if action_name == "OPEN GRIPPER":
                        return (
                            True,
                            f"{action_name} complete. "
                            f"{early_detach_message}",
                        )

                    return (
                        True,
                        f"{action_name} complete "
                        "(controller confirmed success).",
                    )

                if wrapped_result is not None:
                    error_string = (
                        wrapped_result.result.error_string
                        or "controller error"
                    )
                else:
                    error_string = "no result returned"

                return (
                    False,
                    f"GRIPPER FAILED: {error_string}",
                )

            goal_handle.cancel_goal_async()

            return (
                False,
                "GRIPPER ERROR: Controller result timed out.",
            )

        except Exception as error:
            return False, f"GRIPPER ERROR: {error}"

        finally:
            self.gripper_goal_handle = None
            self.gripper_motion_active = False

    def get_speed_scale(self):
        try:
            return float(
                self.speed_override.get().replace("%", "")
            ) / 100.0
        except ValueError:
            return 0.10

    def toggle_control_mode(self):
        if self.busy:
            return

        if self.control_mode.get() == "cartesian":
            self.control_mode.set("joint")
            names = ["J1", "J2", "J3", "J4", "J5", "J6"]
            current_variables = self.current_joint_vars
            target_variables = self.target_joint_vars
            units = ["°"] * 6

            self.mode_button.config(text="MODE: JOINT")
            self.current_frame.config(text="Current Joint Angles")
            self.target_frame.config(text="Target Joint Angles")
            self.jog_frame.config(text="Joint Jog Controls")
            self.jog_info_label.config(text="Joint rotation: 5°/click")
            self.copy_button.config(text="Copy Current Joints")
        else:
            self.control_mode.set("cartesian")
            names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            current_variables = self.current_vars
            target_variables = self.target_vars
            units = ["m", "m", "m", "°", "°", "°"]

            self.mode_button.config(text="MODE: CARTESIAN")
            self.current_frame.config(text="Current TCP Pose")
            self.target_frame.config(text="Target TCP Pose")
            self.jog_frame.config(text="Jog Controls")
            self.jog_info_label.config(
                text="Position: 10 mm/click | Rotation: 5°/click"
            )
            self.copy_button.config(text="COPY CURRENT POSE")

        for index, name in enumerate(names):
            self.current_name_labels[index].config(text=f"{name}:")
            self.current_value_labels[index].config(
                textvariable=current_variables[name]
            )
            self.current_unit_labels[index].config(text=units[index])

            self.target_name_labels[index].config(text=f"{name}:")
            self.target_entries[index].config(
                textvariable=target_variables[name]
            )
            self.target_unit_labels[index].config(text=units[index])

            self.jog_axis_labels[index].config(text=name)

        self.copy_current_pose()
        
    def joint_state_callback(self, message):
        positions = dict(zip(message.name, message.position))

        if "left_gear_joint" in positions:
            self.current_gripper_position = positions[
                "left_gear_joint"
            ]

        try:
            self.current_joint_positions = tuple(
                positions[f"joint_{number}"]
                for number in range(1, 7)
            )
        except KeyError:
            pass

    def add_ground_collision(self):
        ground = CollisionObject()
        ground.header.frame_id = "base_link"
        ground.id = "ground"

        ground_shape = SolidPrimitive()
        ground_shape.type = SolidPrimitive.BOX
        ground_shape.dimensions = [10.0, 10.0, 0.10]

        ground_pose = Pose()
        ground_pose.position.x = 0.0
        ground_pose.position.y = 0.0
        ground_pose.position.z = -0.06
        ground_pose.orientation.w = 1.0

        ground.primitives.append(ground_shape)
        ground.primitive_poses.append(ground_pose)
        ground.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(ground)
        planning_scene.is_diff = True

        self.planning_scene_publisher.publish(planning_scene)

    def add_table_collision(self):
        table = CollisionObject()
        table.header.frame_id = "base_link"
        table.id = "table"

        table_shape = SolidPrimitive()
        table_shape.type = SolidPrimitive.BOX
        table_shape.dimensions = [0.70, 0.90, 0.3525]

        table_pose = Pose()
        table_pose.position.x = 0.90
        table_pose.position.y = 0.00
        table_pose.position.z = 0.17625
        table_pose.orientation.w = 1.0

        table.primitives.append(table_shape)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(table)
        planning_scene.is_diff = True

        self.planning_scene_publisher.publish(planning_scene)

    def add_pick_table_collision(self):
        pick_table = CollisionObject()
        pick_table.header.frame_id = "base_link"
        pick_table.id = "pick_table"

        pick_table_shape = SolidPrimitive()
        pick_table_shape.type = SolidPrimitive.BOX
        pick_table_shape.dimensions = [0.60, 0.40, 0.35]

        pick_table_pose = Pose()
        pick_table_pose.position.x = 0.00
        pick_table_pose.position.y = -0.90
        pick_table_pose.position.z = 0.175
        pick_table_pose.orientation.w = 1.0

        pick_table.primitives.append(pick_table_shape)
        pick_table.primitive_poses.append(pick_table_pose)
        pick_table.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(pick_table)
        planning_scene.is_diff = True

        self.planning_scene_publisher.publish(planning_scene)

    def load_tcp_config(self):
        if not TCP_CONFIG_FILE.exists():
            return

        try:
            with TCP_CONFIG_FILE.open("r", encoding="utf-8") as file:
                values = json.load(file)

            if isinstance(values, list) and len(values) == 6:
                self.tcp_offset = [float(value) for value in values]

        except (OSError, ValueError, json.JSONDecodeError):
            self.tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def write_tcp_config(self):
        with TCP_CONFIG_FILE.open("w", encoding="utf-8") as file:
            json.dump(self.tcp_offset, file, indent=4)
        
    def load_saved_poses(self):
        if not SAVED_POSES_FILE.exists():
            return

        try:
            with SAVED_POSES_FILE.open("r", encoding="utf-8") as file:
                loaded_poses = json.load(file)

            normalized_poses = {}

            for pose_name, saved_data in loaded_poses.items():
                if isinstance(saved_data, list):
                    normalized_poses[pose_name] = {
                        "pose": saved_data,
                        "joints": None,
                        "saved_mode": "cartesian",
                    }
                elif isinstance(saved_data, dict):
                    normalized_poses[pose_name] = {
                        "pose": saved_data.get("pose"),
                        "joints": saved_data.get("joints"),
                        "saved_mode": saved_data.get(
                            "saved_mode",
                            "cartesian",
                        ),
                    }

            self.saved_poses = normalized_poses

        except (OSError, json.JSONDecodeError):
            self.saved_poses = {}

    def write_saved_poses(self):
        with SAVED_POSES_FILE.open("w", encoding="utf-8") as file:
            json.dump(self.saved_poses, file, indent=4)

    def open_tcp_window(self):
        if self.tcp_window is not None and self.tcp_window.winfo_exists():
            self.tcp_window.lift()
            return

        self.tcp_window = tk.Toplevel(self.root)
        self.tcp_window.title("TCP Settings")
        self.tcp_window.geometry("390x360")
        self.tcp_window.resizable(False, False)

        ttk.Label(
            self.tcp_window,
            text="Tool Center Point relative to link_6",
            font=("Arial", 14, "bold"),
        ).pack(pady=12)

        ttk.Label(
            self.tcp_window,
            text="Position in mm | Orientation in degrees",
        ).pack(pady=(0, 10))

        values = [
            self.tcp_offset[0] * 1000.0,
            self.tcp_offset[1] * 1000.0,
            self.tcp_offset[2] * 1000.0,
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        ]
        names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        units = ["mm", "mm", "mm", "°", "°", "°"]

        fields_frame = ttk.Frame(self.tcp_window)
        fields_frame.pack(pady=5)

        self.tcp_vars = {}

        for row, (name, value, unit) in enumerate(
            zip(names, values, units)
        ):
            self.tcp_vars[name] = tk.StringVar(
                value=f"{value:.3f}"
            )

            ttk.Label(
                fields_frame,
                text=f"{name}:",
                width=8,
            ).grid(row=row, column=0, padx=5, pady=5)

            ttk.Entry(
                fields_frame,
                textvariable=self.tcp_vars[name],
                width=15,
            ).grid(row=row, column=1, padx=5, pady=5)

            ttk.Label(
                fields_frame,
                text=unit,
                width=5,
            ).grid(row=row, column=2, padx=5, pady=5)

        buttons_frame = ttk.Frame(self.tcp_window)
        buttons_frame.pack(pady=15)

        ttk.Button(
            buttons_frame,
            text="APPLY",
            command=self.apply_tcp_settings,
        ).grid(row=0, column=0, padx=5)

        ttk.Button(
            buttons_frame,
            text="SAVE",
            command=self.save_tcp_settings,
        ).grid(row=0, column=1, padx=5)

        ttk.Button(
            buttons_frame,
            text="RESET",
            command=self.reset_tcp_settings,
        ).grid(row=0, column=2, padx=5)


    def apply_tcp_settings(self):
        try:
            x = float(self.tcp_vars["X"].get()) / 1000.0
            y = float(self.tcp_vars["Y"].get()) / 1000.0
            z = float(self.tcp_vars["Z"].get()) / 1000.0
            roll = float(self.tcp_vars["Roll"].get())
            pitch = float(self.tcp_vars["Pitch"].get())
            yaw = float(self.tcp_vars["Yaw"].get())
        except ValueError:
            messagebox.showerror(
                "Invalid TCP",
                "Enter valid numbers in all six TCP fields.",
                parent=self.tcp_window,
            )
            return False

        self.tcp_offset = [x, y, z, roll, pitch, yaw]
        self.status_var.set("TCP settings applied.")
        return True

    def save_tcp_settings(self):
        if not self.apply_tcp_settings():
            return

        try:
            self.write_tcp_config()
            self.status_var.set("TCP settings saved.")
        except OSError as error:
            messagebox.showerror(
                "TCP Save Failed",
                str(error),
                parent=self.tcp_window,
            )

    def reset_tcp_settings(self):
        for name in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            self.tcp_vars[name].set("0.000")

        self.apply_tcp_settings()
        
    def create_widgets(self):
        title_frame = ttk.Frame(self.scrollable_frame)
        title_frame.pack(pady=15)

        ttk.Label(
            title_frame,
            text="6-DOF Robot Arm - Pick & Place Cell",
            font=("Arial", 20, "bold"),
        ).grid(
            row=0,
            column=0,
            columnspan=6,
            pady=(0, 4),
        )

        ttk.Label(
            title_frame,
            text="Generic Pick & Place • Automatic Jig Detection • Placement Validation",
            font=("Arial", 10),
        ).grid(
            row=1,
            column=0,
            columnspan=6,
            pady=(0, 10),
        )

        self.mode_button = ttk.Button(
            title_frame,
            text="MODE: CARTESIAN",
            command=self.toggle_control_mode,
        )
        self.mode_button.grid(
            row=2,
            column=0,
            padx=(0, 10),
        )

        ttk.Label(
            title_frame,
            text="Speed:",
        ).grid(
            row=2,
            column=1,
            padx=(0, 5),
        )

        self.speed_selector = ttk.Combobox(
            title_frame,
            textvariable=self.speed_override,
            values=["10%", "25%", "50%", "75%", "100%"],
            state="readonly",
            width=6,
        )
        self.speed_selector.grid(
            row=2,
            column=2,
            padx=(0, 10),
        )

        self.tcp_button = ttk.Button(
            title_frame,
            text="TCP SETTINGS",
            command=self.open_tcp_window,
        )
        self.tcp_button.grid(
            row=2,
            column=3,
            padx=(0, 10),
        )

        ttk.Label(
            title_frame,
            text="Jog Frame:",
        ).grid(
            row=2,
            column=4,
            padx=(0, 5),
        )

        self.jog_frame_selector = ttk.Combobox(
            title_frame,
            textvariable=self.jog_frame_mode,
            values=["WORLD", "FLANGE", "TOOL"],
            state="readonly",
            width=8,
        )
        self.jog_frame_selector.grid(
            row=2,
            column=5,
        )
        
        limits = ttk.Label(
            self.scrollable_frame,
            text="Motion Safety: Joint Limits | Self-Collision | Ground Protection",
            foreground="darkred",
        )
        limits.pack(pady=5)

        current_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Current TCP Pose",
            padding=15,
        )
        current_frame.pack(fill="x", padx=20, pady=10)
        self.current_frame = current_frame

        for row, name in enumerate(
            ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        ):
            unit = "m" if name in ["X", "Y", "Z"] else "°"

            name_label = ttk.Label(
                current_frame,
                text=f"{name}:",
                width=10,
            )
            name_label.grid(
                row=row,
                column=0,
                padx=5,
                pady=3,
                sticky="w",
            )

            value_label = ttk.Label(
                current_frame,
                textvariable=self.current_vars[name],
                width=15,
                font=("Arial", 11, "bold"),
            )
            value_label.grid(
                row=row,
                column=1,
                padx=5,
                pady=3,
            )

            unit_label = ttk.Label(
                current_frame,
                text=unit,
            )
            unit_label.grid(
                row=row,
                column=2,
                padx=5,
                pady=3,
            )

            self.current_name_labels.append(name_label)
            self.current_value_labels.append(value_label)
            self.current_unit_labels.append(unit_label)

        target_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Target TCP Pose",
            padding=15,
        )
        target_frame.pack(fill="x", padx=20, pady=10)
        self.target_frame = target_frame

        for row, name in enumerate(
            ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        ):
            unit = "m" if name in ["X", "Y", "Z"] else "°"

            name_label = ttk.Label(
                target_frame,
                text=f"{name}:",
                width=10,
            )
            name_label.grid(
                row=row,
                column=0,
                padx=5,
                pady=4,
                sticky="w",
            )

            target_entry = ttk.Entry(
                target_frame,
                textvariable=self.target_vars[name],
                width=18,
            )
            target_entry.grid(
                row=row,
                column=1,
                padx=5,
                pady=4,
            )

            unit_label = ttk.Label(
                target_frame,
                text=unit,
            )
            unit_label.grid(
                row=row,
                column=2,
                padx=5,
                pady=4,
            )

            self.target_name_labels.append(name_label)
            self.target_entries.append(target_entry)
            self.target_unit_labels.append(unit_label)

        self.copy_button = ttk.Button(
            target_frame,
            text="Copy Current Pose",
            command=self.copy_current_pose,
        )
        self.copy_button.grid(
            row=6,
            column=0,
            columnspan=3,
            pady=10,
        )

        jog_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Jog Controls",
            padding=10,
        )
        jog_frame.pack(fill="x", padx=20, pady=5)
        self.jog_frame = jog_frame

        self.jog_info_label = ttk.Label(
            jog_frame,
            text="Position: 10 mm/click | Rotation: 5°/click",
        )
        self.jog_info_label.grid(
            row=0,
            column=0,
            columnspan=7,
            pady=(0, 8),
        )

        jog_axes = [
            ("X", JOG_POSITION_STEP),
            ("Y", JOG_POSITION_STEP),
            ("Z", JOG_POSITION_STEP),
            ("Roll", JOG_ANGLE_STEP),
            ("Pitch", JOG_ANGLE_STEP),
            ("Yaw", JOG_ANGLE_STEP),
        ]

        for index, (axis, step) in enumerate(jog_axes):
            group_column = 0 if index < 3 else 4
            row = (index % 3) + 1

            axis_label = ttk.Label(
                jog_frame,
                text=axis,
                width=6,
            )
            axis_label.grid(
                row=row,
                column=group_column,
                padx=3,
                pady=4,
            )
            self.jog_axis_labels.append(axis_label)

            minus_button = ttk.Button(
                jog_frame,
                text="−",
                width=4,
                command=lambda a=axis, s=-step: self.request_jog(a, s),
            )
            minus_button.grid(
                row=row,
                column=group_column + 1,
                padx=3,
                pady=4,
            )

            plus_button = ttk.Button(
                jog_frame,
                text="+",
                width=4,
                command=lambda a=axis, s=step: self.request_jog(a, s),
            )
            plus_button.grid(
                row=row,
                column=group_column + 2,
                padx=3,
                pady=4,
            )

            self.jog_buttons.extend([minus_button, plus_button])
        button_frame = ttk.Frame(self.scrollable_frame)
        button_frame.pack(pady=15)

        self.move_button = ttk.Button(
            button_frame,
            text="MOVE TO TARGET",
            command=self.request_pose_move,
            width=15,
        )
        self.move_button.grid(row=0, column=0, padx=10, pady=5)

        self.home_button = ttk.Button(
            button_frame,
            text="HOME",
            command=self.request_home,
            width=15,
        )
        self.home_button.grid(row=0, column=1, padx=10, pady=5)
        self.stop_motion_button = ttk.Button(
            button_frame,
            text="STOP",
            command=self.stop_active_motion,
            width=15,
            state="disabled",
        )
        self.stop_motion_button.grid(
            row=0,
            column=2,
            padx=10,
            pady=5,
        )

        gripper_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Gripper Control",
            padding=10,
        )
        gripper_frame.pack(fill="x", padx=20, pady=5)

        self.open_gripper_button = ttk.Button(
            gripper_frame,
            text="OPEN GRIPPER",
            command=lambda: self.request_gripper_position(
                1.75, "Opening gripper..."
            ),
            width=18,
        )
        self.open_gripper_button.grid(
            row=0, column=0, padx=10, pady=5
        )

        self.close_gripper_button = ttk.Button(
            gripper_frame,
            text="CLOSE GRIPPER",
            command=lambda: self.request_gripper_position(
                0.04, "Closing gripper..."
            ),
            width=18,
        )
        self.close_gripper_button.grid(
            row=0, column=1, padx=10, pady=5
        )

        inputs_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Digital Inputs",
            padding=10,
        )
        inputs_frame.pack(fill="x", padx=20, pady=5)

        for number in range(1, 7):
            ttk.Checkbutton(
                inputs_frame,
                text=f"Input {number}",
                variable=self.digital_inputs[number],
                onvalue=True,
                offvalue=False,
            ).grid(
                row=(number - 1) // 3,
                column=(number - 1) % 3,
                padx=15,
                pady=5,
                sticky="w",
            )

        inputs_frame.grid_columnconfigure(3, weight=1)

        self.clear_inputs_button = ttk.Button(
            inputs_frame,
            text="CLEAR ALL",
            command=self.clear_all_digital_inputs,
        )
        self.clear_inputs_button.grid(
            row=0,
            column=3,
            rowspan=2,
            padx=(20, 10),
            sticky="e",
        )

        outputs_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Digital Outputs",
            padding=10,
        )
        outputs_frame.pack(fill="x", padx=20, pady=5)

        self.output_lamps = {}

        for number in range(1, 7):
            output_item_frame = ttk.Frame(outputs_frame)
            output_item_frame.grid(
                row=(number - 1) // 3,
                column=(number - 1) % 3,
                padx=15,
                pady=5,
            )

            ttk.Label(
                output_item_frame,
                text=f"OUT {number}",
                font=("Arial", 11, "bold"),
            ).pack(side="left")

            lamp = tk.Label(
                output_item_frame,
                text="●",
                fg="gray",
                font=("Arial", 14, "bold"),
            )
            lamp.pack(side="left", padx=(6, 0))

            self.output_lamps[number] = lamp

        outputs_frame.grid_columnconfigure(3, weight=1)

        self.all_outputs_off_button = ttk.Button(
            outputs_frame,
            text="ALL OFF",
            command=self.turn_all_digital_outputs_off,
        )
        self.all_outputs_off_button.grid(
            row=0,
            column=3,
            rowspan=2,
            padx=(20, 10),
            sticky="e",
        )
        poses_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Saved Robot Poses",
            padding=10,
        )
        poses_frame.pack(fill="x", padx=20, pady=5)

        self.pose_selector = ttk.Combobox(
            poses_frame,
            textvariable=self.selected_pose,
            values=sorted(self.saved_poses.keys()),
            state="readonly",
            width=25,
        )
        self.pose_selector.grid(
            row=0,
            column=0,
            columnspan=3,
            padx=5,
            pady=5,
        )

        self.save_pose_button = ttk.Button(
            poses_frame,
            text="SAVE CURRENT POSE",
            command=self.save_current_pose,
        )
        self.save_pose_button.grid(row=1, column=0, padx=5, pady=5)

        self.load_pose_button = ttk.Button(
            poses_frame,
            text="LOAD AS TARGET",
            command=self.load_selected_pose,
        )
        self.load_pose_button.grid(row=1, column=1, padx=5, pady=5)

        self.delete_pose_button = ttk.Button(
            poses_frame,
            text="DELETE POSE",
            command=self.delete_selected_pose,
        )
        self.delete_pose_button.grid(row=1, column=2, padx=5, pady=5)
        self.go_pose_button = ttk.Button(
            poses_frame,
            text="GO TO SELECTED POSE",
            command=self.go_to_selected_pose,
        )
        self.go_pose_button.grid(
            row=2,
            column=0,
            columnspan=3,
            padx=5,
            pady=5,
            sticky="ew",
        )
        self.sequence_button = ttk.Button(
            poses_frame,
            text="SEQUENCE CONTROLLER",
            command=self.open_sequence_window,
        )
        self.sequence_button.grid(
            row=3,
            column=0,
            columnspan=3,
            padx=5,
            pady=5,
            sticky="ew",
        )

        self.auto_jig_button = ttk.Button(
            poses_frame,
            text="AUTOMATIC JIG PLACEMENT",
            command=self.open_automatic_jig_placement_window,
        )
        self.auto_jig_button.grid(
            row=4,
            column=0,
            columnspan=3,
            padx=5,
            pady=5,
            sticky="ew",
        )



        status_frame = ttk.LabelFrame(
            self.scrollable_frame,
            text="Application Status",
            padding=15,
        )
        status_frame.pack(fill="x", padx=20, pady=10)

        self.status_label = ttk.Label(
            status_frame,
            textvariable=self.status_var,
            font=("Arial", 11, "bold"),
            wraplength=490,
            anchor="w",
            justify="left",
        )
        self.status_label.pack(fill="x")

        self.detected_jig_label = ttk.Label(
            status_frame,
            textvariable=self.detected_jig_var,
            font=("Arial", 10),
            anchor="w",
        )
        self.detected_jig_label.pack(fill="x", pady=(10, 0))

        self.holding_label = ttk.Label(
            status_frame,
            textvariable=self.holding_var,
            font=("Arial", 10, "bold"),
            anchor="w",
        )
        self.holding_label.pack(fill="x", pady=(6, 0))

        ttk.Button(
            self.scrollable_frame,
            text="EXIT CONTROLLER",
            command=self.close,
        ).pack(pady=10)

    def open_automatic_jig_placement_window(self):
        if (
            self.auto_jig_window is not None
            and self.auto_jig_window.winfo_exists()
        ):
            self.auto_jig_window.lift()
            self.auto_jig_window.focus_force()
            return

        window = tk.Toplevel(self.root)
        self.auto_jig_window = window

        window.title("Automatic Jig Placement")
        window.geometry("680x760")
        window.minsize(680, 590)
        window.resizable(True, True)

        def close_auto_window():
            self.auto_jig_window = None
            window.destroy()

        window.protocol(
            "WM_DELETE_WINDOW",
            close_auto_window,
        )

        ttk.Label(
            window,
            text="Automatic Jig Placement",
            font=("Arial", 16, "bold"),
        ).pack(pady=(20, 8))

        ttk.Label(
            window,
            text=(
                "Select a placement target. The system will determine "
                "the required jig type and locate a matching jig in Gazebo."
            ),
            font=("Arial", 10),
            wraplength=610,
            justify="center",
        ).pack(pady=(0, 15))

        # ---------------- TARGET ----------------
        target_frame = ttk.LabelFrame(
            window,
            text="Placement Target",
            padding=15,
        )
        target_frame.pack(
            fill="x",
            padx=30,
            pady=8,
        )

        ttk.Label(
            target_frame,
            text="Target:",
            font=("Arial", 11, "bold"),
        ).grid(
            row=0,
            column=0,
            padx=(5, 10),
            pady=5,
            sticky="w",
        )

        self.auto_jig_target_var = tk.StringVar(value="")

        self.auto_jig_target_selector = ttk.Combobox(
            target_frame,
            textvariable=self.auto_jig_target_var,
            values=[],
            state="readonly",
            width=12,
        )
        self.auto_jig_target_selector.grid(
            row=0,
            column=1,
            padx=5,
            pady=5,
            sticky="w",
        )

        self.search_free_targets_button = ttk.Button(
            target_frame,
            text="SEARCH FOR FREE TARGETS",
            command=self.search_for_free_placement_targets,
        )
        self.search_free_targets_button.grid(
            row=0,
            column=2,
            padx=5,
            pady=5,
            sticky="w",
        )

        self.auto_required_jig_var = tk.StringVar()
        self.auto_target_position_var = tk.StringVar()

        ttk.Label(
            target_frame,
            textvariable=self.auto_required_jig_var,
            font=("Arial", 11, "bold"),
        ).grid(
            row=1,
            column=0,
            columnspan=3,
            padx=5,
            pady=(10, 3),
            sticky="w",
        )

        ttk.Label(
            target_frame,
            textvariable=self.auto_target_position_var,
            font=("Arial", 10),
        ).grid(
            row=2,
            column=0,
            columnspan=3,
            padx=5,
            pady=3,
            sticky="w",
        )

        # ---------------- PERCEPTION ----------------
        detection_frame = ttk.LabelFrame(
            window,
            text="Virtual Camera / Jig Detection",
            padding=15,
        )
        detection_frame.pack(
            fill="x",
            padx=30,
            pady=8,
        )

        self.auto_detected_jig_var = tk.StringVar(
            value="Selected Jig: none"
        )
        self.auto_detected_position_var = tk.StringVar(
            value="Jig Position: --"
        )
        self.auto_detected_distance_var = tk.StringVar(
            value="TCP Distance: --"
        )

        ttk.Label(
            detection_frame,
            textvariable=self.auto_detected_jig_var,
            font=("Arial", 11, "bold"),
        ).pack(
            anchor="w",
            pady=3,
        )

        ttk.Label(
            detection_frame,
            textvariable=self.auto_detected_position_var,
        ).pack(
            anchor="w",
            pady=3,
        )

        ttk.Label(
            detection_frame,
            textvariable=self.auto_detected_distance_var,
        ).pack(
            anchor="w",
            pady=3,
        )

        ttk.Button(
            detection_frame,
            text="SCAN FOR MATCHING JIG",
            command=self.scan_automatic_jig_candidate,
        ).pack(
            pady=(12, 3),
        )

        # ---------------- EXECUTION ----------------
        action_frame = ttk.LabelFrame(
            window,
            text="Automatic Operation",
            padding=15,
        )
        action_frame.pack(
            fill="x",
            padx=30,
            pady=8,
        )

        self.auto_operation_status_var = tk.StringVar(
            value="Status: Ready to scan."
        )

        ttk.Label(
            action_frame,
            textvariable=self.auto_operation_status_var,
            font=("Arial", 10, "bold"),
            wraplength=580,
            justify="left",
        ).pack(
            anchor="w",
            pady=(0, 10),
        )

        self.auto_start_button = ttk.Button(
            action_frame,
            text="START AUTOMATIC PLACEMENT",
            command=self.start_automatic_jig_placement,
            state="disabled",
        )
        self.auto_start_button.pack(
            pady=5,
        )

        self.full_auto_run_button = ttk.Button(
            action_frame,
            text="FULL RUN - COMPLETE TABLE",
            command=self.start_full_automatic_run,
            state="normal",
        )
        self.full_auto_run_button.pack(
            pady=5,
        )

        auto_control_frame = ttk.Frame(action_frame)
        auto_control_frame.pack(pady=5)

        self.auto_pause_button = ttk.Button(
            auto_control_frame,
            text="STOP",
            command=self.pause_automatic_operation,
            state="normal",
        )
        self.auto_pause_button.grid(
            row=0,
            column=0,
            padx=5,
        )

        self.auto_resume_button = ttk.Button(
            auto_control_frame,
            text="RESUME",
            command=self.resume_automatic_operation,
            state="disabled",
        )
        self.auto_resume_button.grid(
            row=0,
            column=1,
            padx=5,
        )

        ttk.Label(
            action_frame,
            text=(
                "Automatic motion is intentionally locked until "
                "the target and matching jig are validated."
            ),
            foreground="darkred",
            wraplength=580,
            justify="center",
        ).pack(
            pady=(8, 0),
        )

        def update_required_jig(event=None):
            self.update_automatic_target_information()

        self.auto_jig_target_selector.bind(
            "<<ComboboxSelected>>",
            update_required_jig,
        )

        self.update_automatic_target_information()


    def discover_placement_targets_from_tf(self):
        discovered_targets = {}

        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            frame_data = yaml.safe_load(frames_yaml) or {}
        except Exception as error:
            print(
                "TARGET DISCOVERY ERROR:",
                error,
                flush=True,
            )
            return {}

        jig_types = {
            "L": "large_jig_",
            "M": "medium_jig_",
            "S": "small_jig_",
        }

        for frame_name in frame_data:
            normalized_name = frame_name.lower()

            if not normalized_name.endswith("_target_link"):
                continue

            target_id = normalized_name[
                :-len("_target_link")
            ].upper()

            if (
                len(target_id) < 2
                or target_id[0] not in jig_types
                or not target_id[1:].isdigit()
            ):
                continue

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    frame_name,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except Exception as error:
                print(
                    "TARGET TF LOOKUP FAILED:",
                    frame_name,
                    error,
                    flush=True,
                )
                continue

            translation = transform.transform.translation

            discovered_targets[target_id] = {
                "position": (
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ),
                "jig_type": jig_types[target_id[0]],
                "frame": frame_name,
            }

        return discovered_targets


    def search_for_free_placement_targets(self):
        discovered_targets = self.discover_placement_targets_from_tf()

        free_targets = {
            target_name: target_data
            for target_name, target_data in discovered_targets.items()
            if target_name not in self.filled_placement_targets
        }

        def target_sort_key(target_name):
            prefix = target_name[0]
            number = int(target_name[1:])
            return (prefix, number)

        ordered_targets = sorted(
            free_targets,
            key=target_sort_key,
        )

        # Replace the old fixed placement-target database with the
        # targets that actually exist in the live TF scene.
        self.placement_targets = free_targets

        if (
            hasattr(self, "auto_jig_target_selector")
            and self.auto_jig_target_selector.winfo_exists()
        ):
            self.auto_jig_target_selector["values"] = ordered_targets

        if ordered_targets:
            current_target = self.auto_jig_target_var.get()

            if current_target not in free_targets:
                self.auto_jig_target_var.set(ordered_targets[0])

            self.update_automatic_target_information()

            self.auto_operation_status_var.set(
                "Status: Found free targets: "
                + ", ".join(ordered_targets)
            )
        else:
            self.auto_jig_target_var.set("")

            self.auto_required_jig_var.set(
                "Required Jig: --"
            )
            self.auto_target_position_var.set(
                "Target Center: --"
            )
            self.auto_operation_status_var.set(
                "Status: No free placement targets found."
            )

        print(
            "FREE TARGET SEARCH:",
            ordered_targets,
            flush=True,
        )

        return ordered_targets


    def update_automatic_target_information(self):
        if not hasattr(self, "auto_jig_target_var"):
            return

        target_name = self.auto_jig_target_var.get()

        target_data = self.placement_targets.get(target_name)

        jig_type_names = {
            "large_jig_": "LARGE",
            "medium_jig_": "MEDIUM",
            "small_jig_": "SMALL",
        }

        required_type = (
            jig_type_names.get(
                target_data.get("jig_type"),
                "UNKNOWN",
            )
            if target_data is not None
            else "UNKNOWN"
        )

        self.auto_required_jig_var.set(
            f"Required Jig: {required_type}"
        )

        target_data = self.placement_targets.get(target_name)

        if target_data is not None:
            x, y, z = target_data["position"]

            self.auto_target_position_var.set(
                f"Target Center: X {x:.3f}   Y {y:.3f}   Z {z:.3f} m"
            )
        else:
            self.auto_target_position_var.set(
                "Target Center: unavailable"
            )

        self.auto_detected_jig_var.set(
            "Selected Jig: none"
        )
        self.auto_detected_position_var.set(
            "Jig Position: --"
        )
        self.auto_detected_distance_var.set(
            "TCP Distance: --"
        )
        self.auto_operation_status_var.set(
            "Status: Target selected. Scan for a matching jig."
        )
        self.auto_start_button.config(state="disabled")


    def scan_automatic_jig_candidate(self):
        if self.current_pose is None:
            self.auto_operation_status_var.set(
                "Status: Robot TCP pose is not available."
            )
            self.auto_start_button.config(state="disabled")
            return

        target_name = self.auto_jig_target_var.get()

        prefixes = {
            "L1": "large_jig_",
            "M1": "medium_jig_",
            "M2": "medium_jig_",
            "S1": "small_jig_",
        }

        required_prefix = prefixes.get(target_name)

        if required_prefix is None:
            self.auto_operation_status_var.set(
                "Status: Invalid placement target."
            )
            self.auto_start_button.config(state="disabled")
            return

        gx, gy, gz = self.current_pose[:3]

        best_model = None
        best_link = None
        best_position = None
        best_distance = float("inf")

        for model_name, link_name in self.jig_candidates:
            if not model_name.startswith(required_prefix):
                continue

            # Do not select a jig already being held.
            if model_name == self.attached_jig_model:
                continue

            # Do not recycle a jig that has already been placed.
            if model_name in self.placed_jig_models:
                continue

            jig_position = self.get_entity_position(model_name)

            if jig_position is None:
                continue

            jx, jy, jz = jig_position

            distance = sqrt(
                (gx - jx) ** 2
                + (gy - jy) ** 2
                + (gz - jz) ** 2
            )

            if distance < best_distance:
                best_distance = distance
                best_model = model_name
                best_link = link_name
                best_position = jig_position

        if best_model is None or best_position is None:
            self.auto_detected_jig_var.set(
                "Selected Jig: none"
            )
            self.auto_detected_position_var.set(
                "Jig Position: --"
            )
            self.auto_detected_distance_var.set(
                "TCP Distance: --"
            )
            self.auto_operation_status_var.set(
                "Status: No matching jig could be detected."
            )
            self.auto_start_button.config(state="disabled")
            return

        self.auto_selected_jig_model = best_model
        self.auto_selected_jig_link = best_link
        self.auto_selected_jig_position = best_position

        jx, jy, jz = best_position

        self.auto_detected_jig_var.set(
            f"Selected Jig: {best_model}"
        )
        self.auto_detected_position_var.set(
            f"Jig Position: X {jx:.3f}   Y {jy:.3f}   Z {jz:.3f} m"
        )
        self.auto_detected_distance_var.set(
            f"TCP Distance: {best_distance:.3f} m"
        )

        self.auto_operation_status_var.set(
            f"Status: {best_model} matches {target_name} and is ready."
        )

        self.auto_start_button.config(state="normal")


    def pause_automatic_operation(self):
        if not self.auto_jig_running:
            return

        self.auto_pause_requested = True

        if (
            hasattr(self, "auto_pause_button")
            and self.auto_pause_button.winfo_exists()
        ):
            self.auto_pause_button.config(state="disabled")

        if (
            hasattr(self, "auto_resume_button")
            and self.auto_resume_button.winfo_exists()
        ):
            self.auto_resume_button.config(state="normal")

        self.auto_operation_status_var.set(
            "Status: STOP requested. Cancelling current motion..."
        )

        if self.sequence_goal_handle is not None:
            try:
                self.sequence_goal_handle.cancel_goal_async()
            except Exception:
                pass


    def resume_automatic_operation(self):
        if not self.auto_pause_requested:
            return

        self.auto_pause_requested = False

        if (
            hasattr(self, "auto_pause_button")
            and self.auto_pause_button.winfo_exists()
        ):
            self.auto_pause_button.config(state="normal")

        if (
            hasattr(self, "auto_resume_button")
            and self.auto_resume_button.winfo_exists()
        ):
            self.auto_resume_button.config(state="disabled")

        self.auto_operation_status_var.set(
            "Status: Resume requested."
        )


    def start_full_automatic_run(self):
        if self.busy or self.auto_jig_running or self.full_auto_run_active:
            return

        remaining_targets = self.search_for_free_placement_targets()

        if not remaining_targets:
            messagebox.showinfo(
                "Full Automatic Run",
                "All placement targets are already FILLED.",
                parent=self.auto_jig_window,
            )
            self.auto_operation_status_var.set(
                "Status: Full table is already complete."
            )
            return

        self.full_auto_run_targets = remaining_targets
        self.full_auto_run_index = 0

        approved = messagebox.askyesno(
            "Full Automatic Run",
            "Run the complete automatic table sequence?\n\n"
            "Targets: "
            + " -> ".join(remaining_targets),
            parent=self.auto_jig_window,
        )

        if not approved:
            return

        self.full_auto_run_active = True

        if (
            hasattr(self, "full_auto_run_button")
            and self.full_auto_run_button.winfo_exists()
        ):
            self.full_auto_run_button.config(state="disabled")

        if not self.prepare_next_full_automatic_target():
            self.full_auto_run_active = False
            return

        if not self.launch_prepared_full_automatic_target():
            self.full_auto_run_active = False
            self.auto_operation_status_var.set(
                "Status: Full run could not launch the first target."
            )
            return


    def prepare_next_full_automatic_target(self):
        while self.full_auto_run_index < len(self.full_auto_run_targets):
            target_name = self.full_auto_run_targets[
                self.full_auto_run_index
            ]

            # If the target became filled while the run was active,
            # skip it safely.
            if target_name in self.filled_placement_targets:
                self.full_auto_run_index += 1
                continue

            self.auto_selected_jig_model = None
            self.auto_selected_jig_link = None
            self.auto_selected_jig_position = None

            self.auto_jig_target_var.set(target_name)
            self.update_automatic_target_information()
            self.scan_automatic_jig_candidate()

            selected_model = getattr(
                self,
                "auto_selected_jig_model",
                None,
            )

            selected_link = getattr(
                self,
                "auto_selected_jig_link",
                None,
            )

            if selected_model is None or selected_link is None:
                self.auto_operation_status_var.set(
                    f"Status: Full run stopped. "
                    f"No matching jig available for {target_name}."
                )
                return False

            self.auto_operation_status_var.set(
                f"Status: Full run prepared {target_name} "
                f"with {selected_model}."
            )
            return True

        self.auto_operation_status_var.set(
            "Status: Full table is complete."
        )
        return False


    def launch_prepared_full_automatic_target(self):
        target_name = self.auto_jig_target_var.get()

        selected_model = getattr(
            self,
            "auto_selected_jig_model",
            None,
        )

        selected_link = getattr(
            self,
            "auto_selected_jig_link",
            None,
        )

        if (
            target_name not in self.placement_targets
            or selected_model is None
            or selected_link is None
        ):
            return False

        self.auto_jig_running = True
        self.motion_cancel_requested = False
        self.last_placement_status = None

        self.set_busy(True)

        if (
            hasattr(self, "auto_start_button")
            and self.auto_start_button.winfo_exists()
        ):
            self.auto_start_button.config(state="disabled")

        if (
            hasattr(self, "full_auto_run_button")
            and self.full_auto_run_button.winfo_exists()
        ):
            self.full_auto_run_button.config(state="disabled")

        self.set_automatic_operation_status(
            f"Full run: starting {target_name} with {selected_model}..."
        )

        Thread(
            target=self.run_automatic_jig_placement,
            args=(
                target_name,
                selected_model,
                selected_link,
            ),
            daemon=True,
        ).start()

        return True


    def continue_full_automatic_run(self):
        if not self.full_auto_run_active:
            return

        self.full_auto_run_index += 1

        if self.full_auto_run_index >= len(self.full_auto_run_targets):
            self.full_auto_run_active = False

            self.auto_operation_status_var.set(
                "Status: FULL RUN COMPLETE - all requested targets finished."
            )

            if (
                hasattr(self, "full_auto_run_button")
                and self.full_auto_run_button.winfo_exists()
            ):
                self.full_auto_run_button.config(state="normal")

            return

        if not self.prepare_next_full_automatic_target():
            self.full_auto_run_active = False

            if (
                hasattr(self, "full_auto_run_button")
                and self.full_auto_run_button.winfo_exists()
            ):
                self.full_auto_run_button.config(state="normal")

            return

        if not self.launch_prepared_full_automatic_target():
            self.full_auto_run_active = False

            self.auto_operation_status_var.set(
                "Status: Full run stopped because the next target "
                "could not be launched."
            )

            if (
                hasattr(self, "full_auto_run_button")
                and self.full_auto_run_button.winfo_exists()
            ):
                self.full_auto_run_button.config(state="normal")


    def start_automatic_jig_placement(self):
        if self.busy or self.auto_jig_running:
            return

        target_name = self.auto_jig_target_var.get()

        if target_name not in self.placement_targets:
            messagebox.showerror(
                "Automatic Jig Placement",
                "The selected placement target is invalid.",
                parent=self.auto_jig_window,
            )
            return

        if target_name in self.filled_placement_targets:
            messagebox.showwarning(
                "Target Already Filled",
                f"{target_name} already contains a jig.\n\n"
                "Automatic placement has been cancelled.",
                parent=self.auto_jig_window,
            )

            self.auto_operation_status_var.set(
                f"Status: {target_name} is already FILLED."
            )

            return

        selected_model = getattr(
            self,
            "auto_selected_jig_model",
            None,
        )

        selected_link = getattr(
            self,
            "auto_selected_jig_link",
            None,
        )

        if selected_model is None or selected_link is None:
            messagebox.showwarning(
                "No Jig Selected",
                "Scan for a matching jig before starting.",
                parent=self.auto_jig_window,
            )
            return

        required_prefix = self.placement_targets[
            target_name
        ]["jig_type"]

        if not selected_model.startswith(required_prefix):
            messagebox.showerror(
                "Jig Mismatch",
                f"{selected_model} does not match {target_name}.",
                parent=self.auto_jig_window,
            )
            return

        approved = messagebox.askyesno(
            "Start Automatic Jig Placement",
            f"Target: {target_name}\n"
            f"Selected jig: {selected_model}\n\n"
            "Run the autonomous pick-and-place operation?",
            parent=self.auto_jig_window,
        )

        if not approved:
            return

        self.auto_jig_running = True
        self.motion_cancel_requested = False
        self.last_placement_status = None

        self.set_busy(True)

        if (
            hasattr(self, "auto_start_button")
            and self.auto_start_button.winfo_exists()
        ):
            self.auto_start_button.config(state="disabled")

        self.auto_operation_status_var.set(
            "Status: Starting automatic jig placement..."
        )

        Thread(
            target=self.run_automatic_jig_placement,
            args=(
                target_name,
                selected_model,
                selected_link,
            ),
            daemon=True,
        ).start()


    def set_automatic_operation_status(self, message):
        self.root.after(
            0,
            lambda text=message:
                self.auto_operation_status_var.set(
                    f"Status: {text}"
                ),
        )

        self.root.after(
            0,
            lambda text=message:
                self.status_var.set(text),
        )


    def finish_automatic_jig_placement(
        self,
        success,
        message,
    ):
        self.auto_jig_running = False

        self.root.after(
            0,
            lambda: self.set_busy(False),
        )

        self.root.after(
            0,
            lambda text=message:
                self.auto_operation_status_var.set(
                    f"Status: {text}"
                ),
        )

        self.root.after(
            0,
            lambda text=message:
                self.status_var.set(text),
        )

        if self.full_auto_run_active:
            if success:
                # Continue only after the current operation has fully
                # released the busy state.
                self.root.after(
                    100,
                    self.continue_full_automatic_run,
                )
            else:
                # Any failed target stops the complete-table run.
                self.full_auto_run_active = False

                if (
                    self.auto_jig_window is not None
                    and self.auto_jig_window.winfo_exists()
                ):
                    self.root.after(
                        0,
                        lambda:
                            self.auto_start_button.config(
                                state="normal"
                            ),
                    )

                    self.root.after(
                        0,
                        lambda:
                            self.full_auto_run_button.config(
                                state="normal"
                            ),
                    )
        else:
            if (
                self.auto_jig_window is not None
                and self.auto_jig_window.winfo_exists()
            ):
                self.root.after(
                    0,
                    lambda:
                        self.auto_start_button.config(
                            state="normal"
                        ),
                )


    def compute_runtime_ik(self, target):
        if self.current_joint_positions is None:
            print(
                "RUNTIME IK ERROR: current joint state is unavailable.",
                flush=True,
            )
            return None

        if not self.compute_ik_client.wait_for_service(
            timeout_sec=2.0
        ):
            print(
                "RUNTIME IK ERROR: /compute_ik service unavailable.",
                flush=True,
            )
            return None

        x, y, z, roll, pitch, yaw = target

        tcp_quaternion = rpy_to_quaternion(
            roll,
            pitch,
            yaw,
        )

        tcp_offset_quaternion = rpy_to_quaternion(
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        )

        flange_quaternion = quaternion_multiply(
            tcp_quaternion,
            quaternion_conjugate(
                tcp_offset_quaternion
            ),
        )

        rotated_translation = rotate_vector(
            flange_quaternion,
            self.tcp_offset[:3],
        )

        joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        current_joints = [
            float(value)
            for value in self.current_joint_positions
        ]

        def wrap_angle(value):
            return atan2(sin(value), cos(value))

        # Try several IK seeds. The first is the actual current robot
        # configuration. The others encourage MoveIt to expose alternate
        # wrist branches when they exist.
        seed_candidates = [
            list(current_joints),
            [
                current_joints[0],
                current_joints[1],
                current_joints[2],
                wrap_angle(current_joints[3] + 3.141592653589793),
                -current_joints[4],
                wrap_angle(current_joints[5] + 3.141592653589793),
            ],
            [
                current_joints[0],
                current_joints[1],
                current_joints[2],
                wrap_angle(current_joints[3] - 3.141592653589793),
                -current_joints[4],
                wrap_angle(current_joints[5] - 3.141592653589793),
            ],
            [
                current_joints[0],
                current_joints[1],
                current_joints[2],
                0.0,
                current_joints[4],
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                current_joints[3],
                current_joints[4],
                current_joints[5],
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                1.5707963267948966,
                0.0,
            ],
            [
                atan2(y, x),
                current_joints[1],
                current_joints[2],
                0.0,
                1.5707963267948966,
                0.0,
            ],
        ]

        valid_solutions = []

        for seed_index, seed_joints in enumerate(seed_candidates):
            request = GetPositionIK.Request()

            request.ik_request.group_name = "arm"
            request.ik_request.ik_link_name = "link_6"
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout = Duration(
                seconds=0.75
            ).to_msg()

            request.ik_request.pose_stamped.header.frame_id = (
                "base_link"
            )

            pose = request.ik_request.pose_stamped.pose

            pose.position.x = (
                x - rotated_translation[0]
            )
            pose.position.y = (
                y - rotated_translation[1]
            )
            pose.position.z = (
                z - rotated_translation[2]
            )

            pose.orientation.x = flange_quaternion[0]
            pose.orientation.y = flange_quaternion[1]
            pose.orientation.z = flange_quaternion[2]
            pose.orientation.w = flange_quaternion[3]

            request.ik_request.robot_state.joint_state.name = (
                joint_names
            )
            request.ik_request.robot_state.joint_state.position = [
                float(value)
                for value in seed_joints
            ]

            future = self.compute_ik_client.call_async(
                request
            )

            start_time = monotonic()

            while not future.done():
                if monotonic() - start_time > 1.25:
                    print(
                        "RUNTIME IK SEED TIMEOUT:",
                        seed_index,
                        flush=True,
                    )
                    future = None
                    break

                sleep(0.02)

            if future is None:
                continue

            response = future.result()

            if (
                response is None
                or response.error_code.val != 1
            ):
                print(
                    "RUNTIME IK SEED REJECTED:",
                    "seed=",
                    seed_index,
                    "error_code=",
                    None if response is None else response.error_code.val,
                    "seed_joints=",
                    [round(value, 4) for value in seed_joints],
                    flush=True,
                )
                continue

            solution_map = dict(
                zip(
                    response.solution.joint_state.name,
                    response.solution.joint_state.position,
                )
            )

            try:
                joints = [
                    float(solution_map[name])
                    for name in joint_names
                ]
            except KeyError:
                continue

            joint_deltas = [
                abs(
                    atan2(
                        sin(solution - current),
                        cos(solution - current),
                    )
                )
                for solution, current in zip(
                    joints,
                    current_joints,
                )
            ]

            # Prefer the branch with the least overall movement.
            # Give the wrist joints a little extra weight because
            # ±180 degree wrist flips caused the large sweeping motions.
            score = (
                joint_deltas[0]
                + joint_deltas[1]
                + joint_deltas[2]
                + 1.5 * joint_deltas[3]
                + 1.25 * joint_deltas[4]
                + 1.5 * joint_deltas[5]
            )

            valid_solutions.append(
                (
                    score,
                    joints,
                    joint_deltas,
                    seed_index,
                )
            )

        if not valid_solutions:
            print(
                "RUNTIME IK FAILED:",
                "no valid solution for target=",
                target,
                flush=True,
            )
            return None

        valid_solutions.sort(
            key=lambda item: item[0]
        )

        best_score, best_joints, best_deltas, best_seed = (
            valid_solutions[0]
        )

        print(
            "RUNTIME IK SELECTED:",
            "target=",
            target,
            "seed=",
            best_seed,
            "score=",
            round(best_score, 4),
            "delta_deg=",
            [
                round(degrees(value), 1)
                for value in best_deltas
            ],
            "joints=",
            best_joints,
            flush=True,
        )

        return best_joints


    def execute_automatic_pose(
        self,
        target,
        motion_type="PTP",
        joint_source=None,
    ):
        temp_name = "__AUTO_JIG_TEMP_POSE__"

        previous_value = self.saved_poses.get(temp_name)

        target_joints = None

        if joint_source is not None:
            source_data = self.saved_poses.get(joint_source)
            if source_data is not None:
                target_joints = source_data.get("joints")

        if target_joints is None:
            target_joints = self.compute_runtime_ik(target)

            if target_joints is None:
                print(
                    "AUTOMATIC POSE FAILED: runtime IK could not solve target.",
                    target,
                    flush=True,
                )
                return False

        self.saved_poses[temp_name] = {
            "pose": list(target),
            "joints": target_joints,
            "saved_mode": "cartesian",
        }

        try:
            while True:
                success = self.run_blended_sequence(
                    [temp_name],
                    [0.0],
                    [motion_type],
                    finish_when_done=False,
                )

                if success:
                    return True

                if self.auto_pause_requested:
                    self.wait_if_automatic_paused()
                    continue

                return False

        finally:
            if previous_value is None:
                self.saved_poses.pop(temp_name, None)
            else:
                self.saved_poses[temp_name] = previous_value


    def wait_if_automatic_paused(self):
        if not self.auto_pause_requested:
            return

        self.auto_pause_active = True

        self.root.after(
            0,
            lambda: self.auto_operation_status_var.set(
                "Status: Automatic operation paused. Press RESUME to continue."
            ),
        )

        while self.auto_pause_requested:
            sleep(0.10)

        self.auto_pause_active = False

        self.root.after(
            0,
            lambda: self.auto_operation_status_var.set(
                "Status: Resuming automatic operation..."
            ),
        )


    def run_automatic_jig_placement(
        self,
        target_name,
        selected_model,
        selected_link,
    ):
        try:
            # ------------------------------------------------
            # Re-read the jig position immediately before move.
            # This acts like the final virtual-camera check.
            # ------------------------------------------------
            jig_position = self.get_entity_position(
                selected_model
            )

            if jig_position is None:
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: Could not locate {selected_model}.",
                )
                return

            target_data = self.placement_targets.get(
                target_name
            )

            if target_data is None:
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Placement target data unavailable.",
                )
                return

            jx, jy, jz = jig_position
            tx, ty, tz = target_data["position"]

            # ------------------------------------------------
            # Geometry calibrated from successful manual poses.
            #
            # Pickup:
            #   l1 / m1_pick_set ≈ Z 0.440
            #   l11 / m1_pick    ≈ Z 0.370
            #
            # Therefore the grasp TCP is ~18 mm above the jig
            # model centre and approach is ~70 mm above grasp.
            # ------------------------------------------------
            pickup_roll = 179.8
            pickup_pitch = -0.2
            pickup_yaw = 0.2

            pickup_z = 0.370
            pickup_approach_z = 0.500

            pickup_approach = (
                jx,
                jy,
                pickup_approach_z,
                pickup_roll,
                pickup_pitch,
                pickup_yaw,
            )

            pickup_pose = (
                jx,
                jy,
                pickup_z,
                pickup_roll,
                pickup_pitch,
                pickup_yaw,
            )

            if (
                target_name == "L1"
                and selected_model == "large_jig_1"
                and "l11" in self.saved_poses
            ):
                pickup_pose = tuple(
                    self.saved_poses["l11"]["pose"]
                )

            # ------------------------------------------------
            # Placement geometry calibrated from successful L1
            # and M1 approach orientations.
            # ------------------------------------------------
            place_roll = 179.8
            place_pitch = 0.0
            place_yaw = 90.0

            place_z = 0.371
            place_approach_z = 0.500

            place_approach = (
                tx,
                ty,
                place_approach_z,
                place_roll,
                place_pitch,
                place_yaw,
            )

            place_pose = (
                tx,
                ty,
                place_z,
                place_roll,
                place_pitch,
                place_yaw,
            )

            # ------------------------------------------------
            # STEP 1: Open gripper.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                "Opening gripper..."
            )

            gripper_success, gripper_message = (
                self.execute_sequence_gripper_action(
                    1.75,
                    "OPEN GRIPPER",
                )
            )

            if not gripper_success:
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: {gripper_message}",
                )
                return

            if self.motion_cancel_requested:
                self.finish_automatic_jig_placement(
                    False,
                    "Automatic placement stopped.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 2: Move above detected jig.
            # PTP-style planning for the larger travel.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Moving above {selected_model}..."
            )

            if not self.execute_automatic_pose(
                pickup_approach,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Could not reach jig approach position.",
                )
                return

            if self.motion_cancel_requested:
                self.finish_automatic_jig_placement(
                    False,
                    "Automatic placement stopped.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 3: Vertical descent to grasp.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Descending to {selected_model}..."
            )

            if not self.execute_automatic_pose(
                pickup_pose,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Could not reach jig pickup position.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 4: Close and attach.
            # Existing generic nearest-jig logic is reused.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Gripping {selected_model}..."
            )

            gripper_success, gripper_message = (
                self.execute_sequence_gripper_action(
                    0.04,
                    "CLOSE GRIPPER",
                )
            )

            if not gripper_success:
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: {gripper_message}",
                )
                return

            if self.attached_jig_model is None:
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Gripper closed but no jig was acquired.",
                )
                return

            if self.attached_jig_model != selected_model:
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: A different jig was attached.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 5: Vertical lift.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Lifting {selected_model}..."
            )

            if not self.execute_automatic_pose(
                pickup_approach,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Could not lift the jig safely.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 6: Travel above selected target.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Moving {selected_model} above {target_name}..."
            )

            if not self.execute_automatic_pose(
                place_approach,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: Could not reach approach above {target_name}.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 7: Vertical placement descent.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Lowering jig into {target_name}..."
            )

            if not self.execute_automatic_pose(
                place_pose,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: Could not reach placement pose at {target_name}.",
                )
                return

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 8: Open and early-detach.
            # Existing placement validation runs automatically.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Releasing jig at {target_name}..."
            )

            gripper_success, gripper_message = (
                self.execute_sequence_gripper_action(
                    1.75,
                    "OPEN GRIPPER",
                )
            )

            if not gripper_success:
                self.finish_automatic_jig_placement(
                    False,
                    f"FAILED: {gripper_message}",
                )
                return

            self.placed_jig_models.add(selected_model)
            self.filled_placement_targets.add(target_name)

            print(
                "AUTOMATIC JIG INVENTORY:",
                selected_model,
                "marked as placed.",
                "placed_jigs=",
                sorted(self.placed_jig_models),
                flush=True,
            )

            self.wait_if_automatic_paused()

            # ------------------------------------------------
            # STEP 9: Retract vertically from target.
            # ------------------------------------------------
            self.set_automatic_operation_status(
                f"Retracting from {target_name}..."
            )

            if not self.execute_automatic_pose(
                place_approach,
                "PTP",
            ):
                self.finish_automatic_jig_placement(
                    False,
                    "FAILED: Jig was released, but retract motion failed.",
                )
                return

            final_message = (
                self.last_placement_status
                if self.last_placement_status is not None
                else (
                    f"Automatic placement complete: "
                    f"{selected_model} → {target_name}"
                )
            )

            self.finish_automatic_jig_placement(
                True,
                final_message,
            )

        except Exception as error:
            self.finish_automatic_jig_placement(
                False,
                f"AUTOMATIC PLACEMENT ERROR: {error}",
            )


    def read_pose(self):
        transform = self.tf_buffer.lookup_transform(
            "base_link",
            "link_6",
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )

        flange_position = transform.transform.translation
        flange_rotation = transform.transform.rotation

        flange_quaternion = [
            flange_rotation.x,
            flange_rotation.y,
            flange_rotation.z,
            flange_rotation.w,
        ]

        tcp_translation = self.tcp_offset[:3]
        rotated_translation = rotate_vector(
            flange_quaternion,
            tcp_translation,
        )
        

        tcp_x = flange_position.x + rotated_translation[0]
        tcp_y = flange_position.y + rotated_translation[1]
        tcp_z = flange_position.z + rotated_translation[2]

        tcp_offset_quaternion = rpy_to_quaternion(
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        )
        tcp_quaternion = quaternion_multiply(
            flange_quaternion,
            tcp_offset_quaternion,
        )

        roll, pitch, yaw = quaternion_to_rpy(
            tcp_quaternion[0],
            tcp_quaternion[1],
            tcp_quaternion[2],
            tcp_quaternion[3],
        )

        return (
            tcp_x,
            tcp_y,
            tcp_z,
            roll,
            pitch,
            yaw,
        )

    def refresh_pose(self):
        try:
            self.current_pose = self.read_pose()

            names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

            for name, value in zip(names, self.current_pose):
                decimals = 3 if name in ["X", "Y", "Z"] else 1
                self.current_vars[name].set(f"{value:.{decimals}f}")

            if self.current_joint_positions is not None:
                for number, value in enumerate(
                    self.current_joint_positions,
                    start=1,
                ):
                    self.current_joint_vars[f"J{number}"].set(
                        f"{degrees(value):.1f}"
                    )
            if not self.target_initialized:
                self.copy_current_pose()
                self.target_initialized = True
                self.status_var.set("Robot connected and ready.")

        except Exception:
            if not self.busy:
                self.status_var.set("Waiting for robot pose...")

        self.root.after(500, self.refresh_pose)

    def copy_current_pose(self):
        if self.control_mode.get() == "joint":
            if self.current_joint_positions is None:
                return

            for number, value in enumerate(
                self.current_joint_positions,
                start=1,
            ):
                self.target_joint_vars[f"J{number}"].set(
                    f"{degrees(value):.1f}"
                )
            return

    
        if self.current_pose is None:
            return

        names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        for name, value in zip(names, self.current_pose):
            decimals = 3 if name in ["X", "Y", "Z"] else 1
            self.target_vars[name].set(f"{value:.{decimals}f}")

    def save_current_pose(self):
        if self.current_pose is None:
            messagebox.showerror(
                "Pose Not Available",
                "The current robot pose is not available yet.",
            )
            return

        pose_name = simpledialog.askstring(
            "Save Current Pose",
            "Enter a name for this pose:",
            parent=self.root,
        )

        if pose_name is None:
            return

        pose_name = pose_name.strip()

        if not pose_name:
            messagebox.showerror(
                "Invalid Name",
                "Please enter a name for the pose.",
            )
            return

        if pose_name in self.saved_poses:
            replace = messagebox.askyesno(
                "Replace Pose",
                f'A pose named "{pose_name}" already exists.\nReplace it?',
            )
            if not replace:
                return

        self.saved_poses[pose_name] = {
            "pose": list(self.current_pose),
            "joints": (
                list(self.current_joint_positions)
                if self.current_joint_positions is not None
                else None
            ),
            "saved_mode": self.control_mode.get(),
        }

        try:
            self.write_saved_poses()
        except OSError as error:
            messagebox.showerror("Save Failed", str(error))
            return

        self.pose_selector["values"] = sorted(self.saved_poses.keys())
        self.selected_pose.set(pose_name)
        self.status_var.set(f'Pose "{pose_name}" saved successfully.')

    def load_selected_pose(self):
        pose_name = self.selected_pose.get()

        if not pose_name or pose_name not in self.saved_poses:
            messagebox.showwarning(
                "No Position Selected",
                "Select a saved position first.",
            )
            return

        saved_data = self.saved_poses[pose_name]

        if self.control_mode.get() == "joint":
            joints = saved_data.get("joints")

            if joints is None:
                messagebox.showwarning(
                    "Joint Data Unavailable",
                    "This older saved position contains only Cartesian data.\n"
                    "Move to it in Cartesian mode, then save it again.",
                )
                return

            for number, value in enumerate(joints, start=1):
                self.target_joint_vars[f"J{number}"].set(
                    f"{degrees(value):.1f}"
                )
        else:
            pose = saved_data.get("pose")

            if pose is None:
                messagebox.showwarning(
                    "Pose Data Unavailable",
                    "This saved position has no Cartesian data.",
                )
                return

            names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

            for name, value in zip(names, pose):
                decimals = 3 if name in ["X", "Y", "Z"] else 1
                self.target_vars[name].set(f"{value:.{decimals}f}")

        self.status_var.set(
            f'Position "{pose_name}" loaded in '
            f'{self.control_mode.get()} mode.'
        )

    def go_to_selected_pose(self):
        if self.busy:
            return

        pose_name = self.selected_pose.get()

        if not pose_name or pose_name not in self.saved_poses:
            messagebox.showwarning(
                "No Pose Selected",
                "Select a saved pose first.",
            )
            return

        saved_data = self.saved_poses[pose_name]

        if (
            self.control_mode.get() == "joint"
            and saved_data.get("joints") is None
        ):
            self.load_selected_pose()
            return

        if (
            self.control_mode.get() == "cartesian"
            and saved_data.get("pose") is None
        ):
            self.load_selected_pose()
            return

        self.load_selected_pose()

        self.set_busy(True)
        self.status_var.set(
            f'Planning PTP move to "{pose_name}"...'
        )

        Thread(
            target=self.run_blended_sequence,
            args=([pose_name], [0.0], ["PTP"]),
            kwargs={"finish_when_done": True},
            daemon=True,
        ).start()

    def delete_selected_pose(self):
        pose_name = self.selected_pose.get()

        if not pose_name or pose_name not in self.saved_poses:
            messagebox.showwarning(
                "No Pose Selected",
                "Select a saved pose first.",
            )
            return

        confirmed = messagebox.askyesno(
            "Delete Pose",
            f'Delete the saved pose "{pose_name}"?',
        )

        if not confirmed:
            return

        del self.saved_poses[pose_name]

        try:
            self.write_saved_poses()
        except OSError as error:
            messagebox.showerror("Delete Failed", str(error))
            return

        self.pose_selector["values"] = sorted(self.saved_poses.keys())
        self.selected_pose.set("")
        self.status_var.set(f'Pose "{pose_name}" deleted.')

    def open_sequence_window(self):
        if (
            self.sequence_window is not None
            and self.sequence_window.winfo_exists()
        ):
            self.sequence_window.lift()
            return

        self.sequence_window = tk.Toplevel(self.root)
        self.sequence_window.title("Sequence Controller")
        self.sequence_window.geometry("720x650")
        self.sequence_window.minsize(720, 650)
        self.sequence_window.resizable(True, True)
        self.sequence_window.protocol(
            "WM_DELETE_WINDOW",
            self.close_sequence_window,
        )

        ttk.Label(
            self.sequence_window,
            text="Build a Robot Motion Sequence",
            font=("Arial", 14, "bold"),
        ).pack(pady=10)

        selection_frame = ttk.Frame(self.sequence_window)
        selection_frame.pack(fill="x", padx=15, pady=5)

        self.sequence_pose_var = tk.StringVar()

        self.sequence_pose_selector = ttk.Combobox(
            selection_frame,
            textvariable=self.sequence_pose_var,
            values=sorted(self.saved_poses.keys()),
            state="readonly",
            width=28,
        )
        self.sequence_pose_selector.grid(
            row=0,
            column=0,
            padx=5,
            pady=5,
        )

        ttk.Button(
            selection_frame,
            text="ADD WAYPOINT",
            command=self.add_pose_to_sequence,
        ).grid(row=0, column=1, padx=5, pady=5)


        self.sequence_rows_container = tk.Frame(
            self.sequence_window,
            background="white",
            bd=1,
            relief="sunken",
            height=280,
        )
        
        self.sequence_rows_container.pack(
            fill="both",
            expand=False,
            padx=20,
            pady=10,
        )

        self.sequence_canvas = tk.Canvas(
            self.sequence_rows_container,
            background="white",
            highlightthickness=0,
            height=260,
        )
        self.sequence_canvas.pack(
            side="left",
            fill="both",
            expand=True,
        )

        self.sequence_scrollbar = ttk.Scrollbar(
            self.sequence_rows_container,
            orient="vertical",
            command=self.sequence_canvas.yview,
        )
        self.sequence_scrollbar.pack(
            side="right",
            fill="y",
        )

        self.sequence_canvas.configure(
            yscrollcommand=self.sequence_scrollbar.set,
        )

        self.sequence_rows_frame = ttk.Frame(
            self.sequence_canvas,
        )

        self.sequence_canvas_window = self.sequence_canvas.create_window(
            (0, 0),
            window=self.sequence_rows_frame,
            anchor="nw",
        )

        self.sequence_canvas.bind(
            "<Configure>",
            lambda event: self.sequence_canvas.itemconfigure(
                self.sequence_canvas_window,
                width=event.width,
            ),
        )
        
        self.sequence_rows_frame.bind(
            "<Configure>",
            lambda event: self.sequence_canvas.configure(
                scrollregion=self.sequence_canvas.bbox("all")
            ),
        )

        ttk.Label(
            self.sequence_rows_frame,
            text="Waypoint",
            font=("Arial", 10, "bold"),
        ).grid(row=0, column=2, padx=5, pady=3)

        ttk.Label(
            self.sequence_rows_frame,
            text="Behavior",
            font=("Arial", 10, "bold"),
        ).grid(row=0, column=3, padx=5, pady=3)

        ttk.Label(
            self.sequence_rows_frame,
            text="Blend Radius",
            font=("Arial", 10, "bold"),
        ).grid(row=0, column=4, padx=5, pady=3)
        
        ttk.Label(
            self.sequence_rows_frame,
            text="Motion",
            font=("Arial", 10, "bold"),
        ).grid(row=0, column=5, padx=5, pady=3)
        
        
        mode_frame = ttk.LabelFrame(
            self.sequence_window,
            text="Waypoint Behavior",
            padding=8,
        )
        mode_frame.pack(fill="x", padx=20, pady=5)

        ttk.Radiobutton(
            mode_frame,
            text="STOP at waypoint",
            variable=self.waypoint_behavior,
            value="stop",
        ).grid(row=0, column=0, sticky="w", padx=5, pady=3)

        ttk.Radiobutton(
            mode_frame,
            text="CONTINUE through waypoint",
            variable=self.waypoint_behavior,
            value="continue",
        ).grid(row=1, column=0, sticky="w", padx=5, pady=3)

        ttk.Label(
            mode_frame,
            text="Blend Radius:",
        ).grid(row=2, column=0, sticky="e", padx=5, pady=3)

        ttk.Entry(
            mode_frame,
            textvariable=self.sequence_blend_radius,
            width=8,
        ).grid(row=2, column=1, sticky="w", padx=5, pady=3)

        ttk.Label(
            mode_frame,
            text="metres",
        ).grid(row=2, column=2, sticky="w", pady=3)

        ttk.Label(
            mode_frame,
            text="Choose waypoint behavior, then click ADD WAYPOINT.",
        ).grid(
            row=3,
            column=0,
            columnspan=3,
            sticky="w",
            padx=5,
            pady=3,
        )

        ttk.Label(
            mode_frame,
            text="Final waypoint always stops safely.",
            foreground="darkred",
        ).grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="w",
            padx=5,
            pady=3,
        )


        edit_frame = ttk.Frame(self.sequence_window)
        edit_frame.pack(pady=5)

        ttk.Button(
            edit_frame,
            text="REMOVE WAYPOINT",
            command=self.remove_sequence_pose,
        ).grid(row=0, column=0, padx=5)

        ttk.Button(
            edit_frame,
            text="CLEAR SEQUENCE",
            command=self.clear_sequence,
        ).grid(row=0, column=1, padx=5)

        run_frame = ttk.Frame(self.sequence_window)
        run_frame.pack(pady=10)

        self.run_sequence_button = ttk.Button(
            run_frame,
            text="RUN SEQUENCE",
            command=self.start_sequence,
        )
        self.run_sequence_button.grid(row=0, column=0, padx=10)

        self.pause_sequence_button = ttk.Button(
            run_frame,
            text="STOP",
            command=self.pause_sequence,
            state="disabled",
        )
        self.pause_sequence_button.grid(row=0, column=1, padx=10)

        self.resume_sequence_button = ttk.Button(
            run_frame,
            text="RESUME",
            command=self.resume_sequence,
            state="disabled",
        )
        self.resume_sequence_button.grid(row=0, column=2, padx=10)

        self.stop_sequence_button = ttk.Button(
            run_frame,
            text="CLOSE SEQUENCE",
            command=self.stop_sequence,
            state="disabled",
        )
        self.stop_sequence_button.grid(row=0, column=3, padx=10)

        self.refresh_sequence_list()

    def close_sequence_window(self):
        if self.sequence_running:
            messagebox.showwarning(
                "Sequence Is Running",
                "Stop the sequence before closing this window.",
            )
            return

        self.sequence_window.destroy()
        self.sequence_window = None
        self.sequence_listbox = None


    def select_sequence_row(self, index):
        self.selected_sequence_index = index

        for row_index, row_data in enumerate(
            self.sequence_row_widgets
        ):
            if row_index == index:
                row_data["frame"].configure(
                    background="lightblue",
                )
            else:
                row_data["frame"].configure(
                    background="white",
                )

    def move_sequence_row_up(self, index):
        self.sync_sequence_from_rows()

        if index <= 0 or index >= len(self.sequence):
            return

        self.sequence[index - 1], self.sequence[index] = (
            self.sequence[index],
            self.sequence[index - 1],
        )

        self.sequence_blend_radii[index - 1], self.sequence_blend_radii[index] = (
            self.sequence_blend_radii[index],
            self.sequence_blend_radii[index - 1],
        )

        self.sequence_motion_types[index - 1], self.sequence_motion_types[index] = (
            self.sequence_motion_types[index],
            self.sequence_motion_types[index - 1],
        )

        self.sequence_input_numbers[index - 1], self.sequence_input_numbers[index] = (
            self.sequence_input_numbers[index],
            self.sequence_input_numbers[index - 1],
        )

        self.sequence_input_states[index - 1], self.sequence_input_states[index] = (
            self.sequence_input_states[index],
            self.sequence_input_states[index - 1],
        )

        self.sequence_input_timings[index - 1], self.sequence_input_timings[index] = (
            self.sequence_input_timings[index],
            self.sequence_input_timings[index - 1],
        )

        self.sequence_output_timings[index - 1], self.sequence_output_timings[index] = (
            self.sequence_output_timings[index],
            self.sequence_output_timings[index - 1],
        )

        if hasattr(self, "sequence_output_numbers"):
            self.sequence_output_numbers[index - 1], self.sequence_output_numbers[index] = (
                self.sequence_output_numbers[index],
                self.sequence_output_numbers[index - 1],
            )

        if hasattr(self, "sequence_output_states"):
            self.sequence_output_states[index - 1], self.sequence_output_states[index] = (
                self.sequence_output_states[index],
                self.sequence_output_states[index - 1],
            )

        self.selected_sequence_index = index - 1
        self.refresh_sequence_list()


    def move_sequence_row_down(self, index):
        self.sync_sequence_from_rows()

        if index < 0 or index >= len(self.sequence) - 1:
            return

        self.sequence[index + 1], self.sequence[index] = (
            self.sequence[index],
            self.sequence[index + 1],
        )

        self.sequence_blend_radii[index + 1], self.sequence_blend_radii[index] = (
            self.sequence_blend_radii[index],
            self.sequence_blend_radii[index + 1],
        )
        self.sequence_motion_types[index + 1], self.sequence_motion_types[index] = (
            self.sequence_motion_types[index],
            self.sequence_motion_types[index + 1],
        )

        self.sequence_input_numbers[index + 1], self.sequence_input_numbers[index] = (
            self.sequence_input_numbers[index],
            self.sequence_input_numbers[index + 1],
        )

        self.sequence_input_states[index + 1], self.sequence_input_states[index] = (
            self.sequence_input_states[index],
            self.sequence_input_states[index + 1],
        )

        self.sequence_input_timings[index + 1], self.sequence_input_timings[index] = (
            self.sequence_input_timings[index],
            self.sequence_input_timings[index + 1],
        )

        self.sequence_output_timings[index + 1], self.sequence_output_timings[index] = (
            self.sequence_output_timings[index],
            self.sequence_output_timings[index + 1],
        )

        if hasattr(self, "sequence_output_numbers"):
            self.sequence_output_numbers[index + 1], self.sequence_output_numbers[index] = (
                self.sequence_output_numbers[index],
                self.sequence_output_numbers[index + 1],
            )

        if hasattr(self, "sequence_output_states"):
            self.sequence_output_states[index + 1], self.sequence_output_states[index] = (
                self.sequence_output_states[index],
                self.sequence_output_states[index + 1],
            )

        self.selected_sequence_index = index + 1
        self.refresh_sequence_list()

    def sync_sequence_from_rows(self):
        new_sequence = []
        new_blend_radii = []
        new_motion_types = []
        new_input_numbers = []
        new_input_states = []
        new_input_timings = []
        new_output_numbers = []
        new_output_states = []
        new_output_timings = []

        for row_data in self.sequence_row_widgets:
            pose_name = row_data["pose_var"].get()
            behavior = row_data["behavior_var"].get()

            if not pose_name:
                continue

            if behavior == "CONTINUE":
                try:
                    blend_radius = float(
                        row_data["radius_var"].get()
                    )
                except ValueError:
                    blend_radius = 0.001

                if blend_radius <= 0.0:
                    blend_radius = 0.001
            else:
                blend_radius = 0.0

            new_sequence.append(pose_name)
            new_blend_radii.append(blend_radius)
            new_motion_types.append(
                row_data["motion_var"].get()
            )
            new_input_numbers.append(
                row_data["input_number_var"].get()
            )
            new_input_states.append(
                row_data["input_state_var"].get()
            )
            new_input_timings.append(
                row_data["input_timing_var"].get()
            )
            new_output_numbers.append(
                row_data["output_number_var"].get()
            )
            new_output_states.append(
                row_data["output_state_var"].get()
            )
            new_output_timings.append(
                row_data["output_timing_var"].get()
            )

        self.sequence = new_sequence
        self.sequence_blend_radii = new_blend_radii
        self.sequence_motion_types = new_motion_types
        self.sequence_input_numbers = new_input_numbers
        self.sequence_input_states = new_input_states
        self.sequence_input_timings = new_input_timings
        self.sequence_output_numbers = new_output_numbers
        self.sequence_output_states = new_output_states
        self.sequence_output_timings = new_output_timings

        print("DEBUG SEQUENCE:", self.sequence, flush=True)
        print("DEBUG RADII:", self.sequence_blend_radii, flush=True)
        print("DEBUG MOTION TYPES:", self.sequence_motion_types, flush=True)
        print("DEBUG OUTPUTS:", self.sequence_output_numbers, flush=True)

    def validate_sequence_settings(self):
        sequence_length = len(self.sequence)

        sequence_lists = [
            self.sequence_blend_radii,
            self.sequence_motion_types,
            self.sequence_input_numbers,
            self.sequence_input_states,
            self.sequence_input_timings,
            self.sequence_output_numbers,
            self.sequence_output_states,
            self.sequence_output_timings,
        ]

        if any(
            len(values) != sequence_length
            for values in sequence_lists
        ):
            return False, "Sequence row data is inconsistent."

        for index, pose_name in enumerate(self.sequence):
            if pose_name not in self.saved_poses:
                return (
                    False,
                    f'Point {index + 1}: pose "{pose_name}" does not exist.',
                )

            if self.sequence_motion_types[index] not in ("PTP", "LIN"):
                return (
                    False,
                    f"Point {index + 1}: invalid motion type.",
                )

            if self.sequence_blend_radii[index] < 0.0:
                return (
                    False,
                    f"Point {index + 1}: blend radius cannot be negative.",
                )

            if self.sequence_input_states[index] not in ("TRUE", "FALSE"):
                return (
                    False,
                    f"Point {index + 1}: invalid input state.",
                )

            if self.sequence_output_states[index] not in ("TRUE", "FALSE"):
                return (
                    False,
                    f"Point {index + 1}: invalid output state.",
                )

            if self.sequence_input_timings[index] not in ("BEFORE", "AFTER"):
                return (
                    False,
                    f"Point {index + 1}: invalid input timing.",
                )

            if self.sequence_output_timings[index] not in ("BEFORE", "AFTER"):
                return (
                    False,
                    f"Point {index + 1}: invalid output timing.",
                )

        return True, ""

    def refresh_sequence_list(self):
        for row_data in self.sequence_row_widgets:
            row_data["frame"].destroy()

        self.sequence_row_widgets.clear()

        for index, (pose_name, blend_radius) in enumerate(
            zip(self.sequence, self.sequence_blend_radii),
            start=1,
        ):

            row_frame = tk.Frame(
                self.sequence_rows_frame,
                bd=1,
                relief="flat",
            )

            row_frame.grid(
                row=index,
                column=0,
                columnspan=6,
                sticky="ew",
                pady=3,
            )
            
            row_frame.bind(
                "<Button-1>",
                lambda event, row_index=index - 1:
                    self.select_sequence_row(row_index),
            )

            ttk.Label(
                row_frame,
                text=str(index),
                font=("Arial", 10, "bold"),
                anchor="center",
            ).grid(
                row=0,
                column=0,
                columnspan=2,
                padx=2,
                pady=(0, 2),
            )

            pose_var = tk.StringVar(
                value=pose_name
            )

            behavior_var = tk.StringVar(
                value=(
                    "CONTINUE"
                    if blend_radius > 0.0
                    else "STOP"
                )
            )

            radius_var = tk.StringVar(
                value=(
                    f"{blend_radius:.3f}"
                    if blend_radius > 0.0
                    else "0.001"
                )
            )
            
            motion_var = tk.StringVar(
                value=(
                    self.sequence_motion_types[index - 1]
                    if index - 1 < len(self.sequence_motion_types)
                    else "LIN"
                )
            )

            input_number_var = tk.StringVar(
                value=(
                    self.sequence_input_numbers[index - 1]
                    if index - 1 < len(self.sequence_input_numbers)
                    else "NONE"
                )
            )

            input_state_var = tk.StringVar(
                value=(
                    self.sequence_input_states[index - 1]
                    if index - 1 < len(self.sequence_input_states)
                    else "TRUE"
                )
            )

            input_timing_var = tk.StringVar(
                value=(
                    self.sequence_input_timings[index - 1]
                    if index - 1 < len(self.sequence_input_timings)
                    else "BEFORE"
                )
            )

            output_number_var = tk.StringVar(
                value=(
                    self.sequence_output_numbers[index - 1]
                    if (
                        hasattr(self, "sequence_output_numbers")
                        and index - 1 < len(self.sequence_output_numbers)
                    )
                    else "NONE"
                )
            )

            output_state_var = tk.StringVar(
                value=(
                    self.sequence_output_states[index - 1]
                    if (
                        hasattr(self, "sequence_output_states")
                        and index - 1 < len(self.sequence_output_states)
                    )
                    else "TRUE"
                )
            )

            output_timing_var = tk.StringVar(
                value=(
                    self.sequence_output_timings[index - 1]
                    if index - 1 < len(self.sequence_output_timings)
                    else "AFTER"
                )
            )
            
            pose_selector = ttk.Combobox(
                row_frame,
                textvariable=pose_var,
                values=sorted(self.saved_poses.keys()),
                state="readonly",
                width=20,
            )
            pose_selector.grid(
                row=0,
                column=2,
                padx=5,
            )

            behavior_selector = ttk.Combobox(
                row_frame,
                textvariable=behavior_var,
                values=["STOP", "CONTINUE"],
                state="readonly",
                width=12,
            )
            behavior_selector.grid(
                row=0,
                column=3,
                padx=5,
            )

            radius_entry = ttk.Entry(
                row_frame,
                textvariable=radius_var,
                width=10,
            )
            radius_entry.grid(
                row=0,
                column=4,
                padx=5,
            )
            
            
            motion_selector = ttk.Combobox(
                row_frame,
                textvariable=motion_var,
                values=["PTP", "LIN"],
                state="readonly",
                width=8,
            )
            motion_selector.grid(
                row=0,
                column=5,
                padx=5,
            )

            logic_frame = ttk.Frame(row_frame)
            logic_frame.grid(
                row=1,
                column=2,
                columnspan=8,
                sticky="w",
                padx=0,
                pady=(3, 2),
            )

            ttk.Label(
                logic_frame,
                text="IF",
                font=("Arial", 10, "bold"),
            ).pack(
                side="left",
                padx=(0, 3),
            )

            input_number_selector = ttk.Combobox(
                logic_frame,
                textvariable=input_number_var,
                values=[
                    "NONE",
                    "IN 1",
                    "IN 2",
                    "IN 3",
                    "IN 4",
                    "IN 5",
                    "IN 6",
                ],
                state="readonly",
                width=9,
            )
            input_number_selector.pack(
                side="left",
                padx=(0, 3),
            )

            ttk.Label(
                logic_frame,
                text="=",
                font=("Arial", 10, "bold"),
            ).pack(
                side="left",
                padx=(0, 3),
            )

            input_state_selector = ttk.Combobox(
                logic_frame,
                textvariable=input_state_var,
                values=["TRUE", "FALSE"],
                state=(
                    "disabled"
                    if input_number_var.get() == "NONE"
                    else "readonly"
                ),
                width=7,
            )
            input_state_selector.pack(
                side="left",
                padx=(0, 3),
            )

            input_timing_selector = ttk.Combobox(
                logic_frame,
                textvariable=input_timing_var,
                values=["BEFORE", "AFTER"],
                state=(
                    "disabled"
                    if input_number_var.get() == "NONE"
                    else "readonly"
                ),
                width=8,
            )
            input_timing_selector.pack(
                side="left",
                padx=(0, 10),
            )

            ttk.Label(
                logic_frame,
                text="OUT",
                font=("Arial", 10, "bold"),
            ).pack(
                side="left",
                padx=(0, 3),
            )

            output_number_selector = ttk.Combobox(
                logic_frame,
                textvariable=output_number_var,
                values=[
                    "NONE",
                    "OUT 1",
                    "OUT 2",
                    "OUT 3",
                    "OUT 4",
                    "OUT 5",
                    "OUT 6",
                    "OPEN GRIPPER",
                    "CLOSE GRIPPER",
                ],
                state="readonly",
                width=9,
            )
            output_number_selector.pack(
                side="left",
                padx=(0, 3),
            )

            ttk.Label(
                logic_frame,
                text="=",
                font=("Arial", 10, "bold"),
            ).pack(
                side="left",
                padx=(0, 3),
            )

            output_state_selector = ttk.Combobox(
                logic_frame,
                textvariable=output_state_var,
                values=["TRUE", "FALSE"],
                state=(
                    "disabled"
                    if output_number_var.get() == "NONE"
                    else "readonly"
                ),
                width=7,
            )
            output_state_selector.pack(
                side="left",
                padx=(0, 3),
            )

            output_timing_selector = ttk.Combobox(
                logic_frame,
                textvariable=output_timing_var,
                values=["BEFORE", "AFTER"],
                state=(
                    "disabled"
                    if output_number_var.get() == "NONE"
                    else "readonly"
                ),
                width=8,
            )
            output_timing_selector.pack(
                side="left",
            )

            def update_input_state_selector(
                event=None,
                input_number_var=input_number_var,
                input_state_selector=input_state_selector,
                input_timing_selector=input_timing_selector,
            ):
                if input_number_var.get() == "NONE":
                    input_state_selector.config(state="disabled")
                    input_timing_selector.config(state="disabled")
                else:
                    input_state_selector.config(state="readonly")
                    input_timing_selector.config(state="readonly")

            input_number_selector.bind(
                "<<ComboboxSelected>>",
                update_input_state_selector,
            )

            def update_output_state_selector(
                event=None,
                output_number_var=output_number_var,
                output_state_selector=output_state_selector,
                output_timing_selector=output_timing_selector,
            ):
                if output_number_var.get() == "NONE":
                    output_state_selector.config(state="disabled")
                    output_timing_selector.config(state="disabled")
                else:
                    output_state_selector.config(state="readonly")
                    output_timing_selector.config(state="readonly")

            output_number_selector.bind(
                "<<ComboboxSelected>>",
                update_output_state_selector,
            )

            up_button = ttk.Button(
                row_frame,
                text="↑",
                width=3,
                command=lambda row_index=index - 1:
                    self.move_sequence_row_up(row_index),
            )
            up_button.grid(
                row=1,
                column=0,
                padx=(5, 2),
                pady=(2, 0),
            )

            down_button = ttk.Button(
                row_frame,
                text="↓",
                width=3,
                command=lambda row_index=index - 1:
                    self.move_sequence_row_down(row_index),
            )
            down_button.grid(
                row=1,
                column=1,
                padx=(2, 5),
                pady=(2, 0),
            )          
            
            if index == 1:
                up_button.configure(state="disabled")

            if index == len(self.sequence):
                down_button.configure(state="disabled")
                
                
            def update_radius_state(
                event=None,
                behavior_var=behavior_var,
                radius_entry=radius_entry,
            ):
                if behavior_var.get() == "CONTINUE":
                    radius_entry.configure(state="normal")
                else:
                    radius_entry.configure(state="disabled")

            behavior_selector.bind(
                "<<ComboboxSelected>>",
                update_radius_state,
            )

            update_radius_state()
            
            
            for widget in (
                pose_selector,
                behavior_selector,
                radius_entry,
            ):
                widget.bind(
                    "<Button-1>",
                    lambda event, row_index=index - 1:
                        self.select_sequence_row(row_index),
                )

            self.sequence_row_widgets.append(
                {
                    "frame": row_frame,
                    "pose_var": pose_var,
                    "behavior_var": behavior_var,
                    "radius_var": radius_var,
                    "motion_var": motion_var,
                    "input_number_var": input_number_var,
                    "input_state_var": input_state_var,
                    "input_timing_var": input_timing_var,
                    "output_number_var": output_number_var,
                    "output_state_var": output_state_var,
                    "output_timing_var": output_timing_var,
                }
            )
            
            if self.selected_sequence_index == index - 1:
                row_frame.configure(
                    background="lightblue",
                )
            else:
                row_frame.configure(
                    background="white",
                )            

    def add_pose_to_sequence(self):
        pose_name = self.sequence_pose_var.get()
        self.sync_sequence_from_rows()
        
        if not pose_name or pose_name not in self.saved_poses:
            messagebox.showwarning(
                "No Pose Selected",
                "Select a saved pose to add.",
            )
            return

        blend_radius = 0.0

        if self.waypoint_behavior.get() == "continue":
            try:
                blend_radius = float(
                    self.sequence_blend_radius.get()
                )
            except ValueError:
                messagebox.showerror(
                    "Invalid Blend Radius",
                    "Enter a valid number such as 0.05.",
                )
                return

            if blend_radius <= 0.0:
                messagebox.showerror(
                    "Invalid Blend Radius",
                    "Continuous points need a blend radius greater than zero.",
                )
                return

        self.sequence.append(pose_name)
        self.sequence_blend_radii.append(blend_radius)
        self.sequence_motion_types.append("LIN")
        self.sequence_input_numbers.append("NONE")
        self.sequence_input_states.append("TRUE")
        self.sequence_input_timings.append("BEFORE")
        self.sequence_output_numbers.append("NONE")
        self.sequence_output_states.append("TRUE")
        self.sequence_output_timings.append("AFTER")
        self.refresh_sequence_list()


    def remove_sequence_pose(self):
        self.sync_sequence_from_rows()

        if self.selected_sequence_index is None:
            self.status_var.set(
                "Select a sequence row first."
            )
            return

        index = self.selected_sequence_index

        if 0 <= index < len(self.sequence):
            del self.sequence[index]
            del self.sequence_blend_radii[index]
            del self.sequence_motion_types[index]
            del self.sequence_input_numbers[index]
            del self.sequence_input_states[index]
            del self.sequence_input_timings[index]
            del self.sequence_output_numbers[index]
            del self.sequence_output_states[index]
            del self.sequence_output_timings[index]

        self.selected_sequence_index = None
        self.refresh_sequence_list()

    def clear_sequence(self):
        if self.sequence_running:
            return

        self.sequence.clear()
        self.sequence_blend_radii.clear()
        self.sequence_motion_types.clear()
        self.sequence_input_numbers.clear()
        self.sequence_input_states.clear()
        self.sequence_input_timings.clear()
        self.sequence_output_numbers.clear()
        self.sequence_output_states.clear()
        self.sequence_output_timings.clear()
        self.refresh_sequence_list()

    def start_sequence(self):
        if self.busy or self.sequence_running:
            return
        self.sync_sequence_from_rows()

        valid, validation_message = self.validate_sequence_settings()

        if not valid:
            messagebox.showerror(
                "Invalid Sequence",
                validation_message,
                parent=self.sequence_window,
            )
            return
        
        if not self.sequence:
            messagebox.showwarning(
                "Empty Sequence",
                "Add at least one saved pose to the sequence.",
            )
            return

        approved = messagebox.askyesno(
            "Run Sequence",
            "Run all poses in the displayed order?",
            parent=self.sequence_window,
        )

        if not approved:
            return

        self.sequence_running = True
        self.stop_sequence_requested = False
        self.sequence_paused = False
        self.sequence_resume_index = 0
        self.sequence_resume_base_index = 0
        self.set_busy(True)

        self.run_sequence_button.config(state="disabled")
        self.pause_sequence_button.config(state="normal")
        self.resume_sequence_button.config(state="disabled")
        self.stop_sequence_button.config(state="normal")
        self.status_var.set("Starting sequence...")

        sequence_to_run = list(self.sequence)
        blend_radii_to_run = list(self.sequence_blend_radii)
        motion_types_to_run = list(self.sequence_motion_types)
        input_numbers_to_run = list(self.sequence_input_numbers)
        input_states_to_run = list(self.sequence_input_states)

        self.active_sequence_input_timings = list(
            self.sequence_input_timings
        )

        self.active_sequence_output_numbers = list(
            self.sequence_output_numbers
        )
        self.active_sequence_output_states = list(
            self.sequence_output_states
        )
        self.active_sequence_output_timings = list(
            self.sequence_output_timings
        )

        Thread(
            target=self.run_sequence,

            args=(
                sequence_to_run,
                blend_radii_to_run,
                motion_types_to_run,
                input_numbers_to_run,
                input_states_to_run,
            ),

            daemon=True,
        ).start()

    def run_sequence(
        self,
        sequence_to_run,
        blend_radii_to_run,
        motion_types_to_run,
        input_numbers_to_run,
        input_states_to_run,
        sequence_offset=0,
    ):
        self.sequence_resume_base_index = sequence_offset

        sequence_end = sequence_offset + len(sequence_to_run)

        has_gripper_action = any(
            output_name in ("OPEN GRIPPER", "CLOSE GRIPPER")
            for output_name in self.active_sequence_output_numbers[
                sequence_offset:sequence_end
            ]
        )

        if has_gripper_action:
            self.run_sequence_with_gripper_actions(
                sequence_to_run,
                blend_radii_to_run,
                motion_types_to_run,
                input_numbers_to_run,
                input_states_to_run,
                sequence_offset,
            )
            return

        self.run_mixed_sequence(
            sequence_to_run,
            blend_radii_to_run,
            motion_types_to_run,
            input_numbers_to_run,
            input_states_to_run,
        )
        return
            
        completed_steps = 0
        final_message = "Sequence completed successfully."

        for index, pose_name in enumerate(sequence_to_run, start=1):
            if self.stop_sequence_requested:
                final_message = (
                    f"Sequence stopped after {completed_steps} completed step(s)."
                )
                break

            if pose_name not in self.saved_poses:
                final_message = f'FAILED: Pose "{pose_name}" no longer exists.'
                break

            saved_data = self.saved_poses[pose_name]
            target = tuple(saved_data["pose"])
            target_joints = saved_data.get("joints")
            saved_mode = saved_data.get(
                "saved_mode",
                "cartesian",
            )

            try:
                current = self.read_pose()
            except Exception as error:
                final_message = f"FAILED: Could not read robot pose: {error}"
                break

            x, y, z, roll, pitch, yaw = target
            cx, cy, cz, croll, cpitch, cyaw = current

            distance = sqrt(
                (x - cx) ** 2
                + (y - cy) ** 2
                + (z - cz) ** 2
            )

            angle_change = max(
                abs(angle_difference(roll, croll)),
                abs(angle_difference(pitch, cpitch)),
                abs(angle_difference(yaw, cyaw)),
            )

            if distance > MAXIMUM_STEP_DISTANCE:
                final_message = (
                    f'FAILED before "{pose_name}": '
                    f"movement is {distance:.3f} m, above the 0.10 m limit."
                )
                break

            if angle_change > MAXIMUM_ANGLE_CHANGE:
                final_message = (
                    f'FAILED before "{pose_name}": '
                    f"angle change is {angle_change:.1f}°, "
                    "above the 15° limit."
                )
                break

            self.root.after(
                0,
                lambda step=index, total=len(sequence_to_run),
                name=pose_name: self.status_var.set(
                    f'Sequence step {step}/{total}: Moving to "{name}"...'
                ),
            )

            if saved_mode == "joint" and target_joints is not None:
                move_success = self.execute_joint_positions(
                    tuple(target_joints)
                )
            else:
                move_success = self.execute_pose(target)

            if not move_success:
                final_message = (
                    f'FAILED at sequence step {index}: "{pose_name}".'
                )
                break

            completed_steps += 1

        self.root.after(
            0,
            lambda: self.finish_sequence(final_message),
        )
    def run_sequence_with_gripper_actions(
        self,
        sequence_to_run,
        blend_radii_to_run,
        motion_types_to_run,
        input_numbers_to_run,
        input_states_to_run,
        sequence_offset=0,
    ):
        self.last_placement_status = None
        segment_start = 0

        for index in range(len(sequence_to_run)):
            absolute_index = sequence_offset + index

            gripper_action = self.get_sequence_gripper_action(
                absolute_index
            )

            if gripper_action is None:
                continue

            output_timing = (
                self.active_sequence_output_timings[absolute_index]
                if (
                    hasattr(self, "active_sequence_output_timings")
                    and absolute_index < len(self.active_sequence_output_timings)
                )
                else "AFTER"
            )

            motion_end = (
                index
                if output_timing == "BEFORE"
                else index + 1
            )

            segment_names = sequence_to_run[
                segment_start:motion_end
            ]

            segment_radii = list(
                blend_radii_to_run[
                    segment_start:motion_end
                ]
            )

            segment_motion_types = list(
                motion_types_to_run[
                    segment_start:motion_end
                ]
            )

            segment_input_numbers = list(
                input_numbers_to_run[
                    segment_start:motion_end
                ]
            )

            segment_input_states = list(
                input_states_to_run[
                    segment_start:motion_end
                ]
            )

            # A gripper action is a hard barrier.
            if segment_radii:
                segment_radii[-1] = 0.0

            self.sequence_resume_base_index = (
                sequence_offset + segment_start
            )

            if segment_names:
                arm_success = self.run_mixed_sequence(
                    segment_names,
                    segment_radii,
                    segment_motion_types,
                    segment_input_numbers,
                    segment_input_states,
                    finish_when_done=False,
                )

                if not arm_success:
                    return False

            if self.stop_sequence_requested:
                self.root.after(
                    0,
                    lambda: self.finish_sequence(
                        "Sequence stopped."
                    ),
                )
                return False

            target_position, action_name = gripper_action
            pose_name = sequence_to_run[index]

            self.sequence_resume_index = absolute_index

            self.root.after(
                0,
                lambda name=pose_name,
                action=action_name:
                    self.status_var.set(
                        f'At "{name}" — executing {action}...'
                    ),
            )

            gripper_success, gripper_message = (
                self.execute_sequence_gripper_action(
                    target_position,
                    action_name,
                )
            )

            if not gripper_success:
                if self.stop_sequence_requested:
                    self.root.after(
                        0,
                        lambda: self.finish_sequence(
                            "Sequence stopped."
                        ),
                    )
                else:
                    self.root.after(
                        0,
                        lambda message=gripper_message,
                        point=absolute_index + 1:
                            self.finish_sequence(
                                f"FAILED at sequence point "
                                f"{point}: {message}"
                            ),
                    )

                return False

            self.sequence_resume_index = absolute_index + 1

            self.root.after(
                0,
                lambda message=gripper_message:
                    self.status_var.set(message),
            )

            segment_start = (
                index
                if output_timing == "BEFORE"
                else index + 1
            )

        if segment_start < len(sequence_to_run):
            self.sequence_resume_base_index = (
                sequence_offset + segment_start
            )

            remaining_names = sequence_to_run[
                segment_start:
            ]

            remaining_radii = list(
                blend_radii_to_run[
                    segment_start:
                ]
            )

            remaining_motion_types = list(
                motion_types_to_run[
                    segment_start:
                ]
            )

            remaining_input_numbers = list(
                input_numbers_to_run[
                    segment_start:
                ]
            )

            remaining_input_states = list(
                input_states_to_run[
                    segment_start:
                ]
            )

            arm_success = self.run_mixed_sequence(
                remaining_names,
                remaining_radii,
                remaining_motion_types,
                remaining_input_numbers,
                remaining_input_states,
                finish_when_done=False,
            )

            if not arm_success:
                return False

        if self.stop_sequence_requested:
            self.root.after(
                0,
                lambda: self.finish_sequence(
                    "Sequence stopped."
                ),
            )
            return False

        final_message = (
            self.last_placement_status
            if self.last_placement_status is not None
            else "Sequence completed successfully."
        )

        self.root.after(
            0,
            lambda message=final_message:
                self.finish_sequence(message),
        )

        return True
    def wait_for_sequence_input(self, input_name, input_state):
        if input_name == "NONE":
            return True

        input_number = int(input_name.split()[-1])
        required_state = input_state == "TRUE"

        while self.digital_inputs[input_number].get() != required_state:
            self.root.after(
                0,
                lambda number=input_number, state=input_state:
                    self.status_var.set(
                        f"Waiting for IN {number} = {state}..."
                    ),
            )

            if self.stop_sequence_requested:
                return False

            sleep(0.10)

        self.root.after(
            0,
            lambda number=input_number, state=input_state:
                self.status_var.set(
                    f"IN {number} = {state} — proceeding..."
                ),
        )

        return True


    def run_mixed_sequence(
        self,
        sequence_to_run,
        blend_radii_to_run,
        motion_types_to_run,
        input_numbers_to_run,
        input_states_to_run,
        finish_when_done=True,
    ):
        segment_start = 0

        for index, blend_radius in enumerate(blend_radii_to_run):
            is_final_pose = index == len(sequence_to_run) - 1

            input_condition = input_numbers_to_run[index]
            has_input_condition = input_condition != "NONE"

            absolute_index = self.sequence_resume_base_index + index
            input_timing = (
                self.active_sequence_input_timings[absolute_index]
                if (
                    hasattr(self, "active_sequence_input_timings")
                    and absolute_index < len(self.active_sequence_input_timings)
                )
                else "BEFORE"
            )

            output_name = (
                self.active_sequence_output_numbers[absolute_index]
                if (
                    hasattr(self, "active_sequence_output_numbers")
                    and absolute_index < len(self.active_sequence_output_numbers)
                )
                else "NONE"
            )
            has_output_action = output_name != "NONE"

            # Any I/O event is an intentional synchronization point.
            # Pure motion waypoints keep their existing blended behavior.
            if (
                blend_radius > 0.0
                and not is_final_pose
                and not has_input_condition
                and not has_output_action
            ):
                continue

            if has_input_condition and input_timing == "BEFORE":
                if segment_start < index:
                    precondition_names = sequence_to_run[
                        segment_start:index
                    ]

                    precondition_radii = list(
                        blend_radii_to_run[
                            segment_start:index
                        ]
                    )

                    if precondition_radii:
                        precondition_radii[-1] = 0.0

                    if len(precondition_names) == 1:
                        pose_name = precondition_names[0]
                        saved_data = self.saved_poses[pose_name]
                        target_joints = saved_data.get("joints")

                        if target_joints is not None:
                            success = self.execute_joint_positions(
                                tuple(target_joints)
                            )
                        else:
                            success = self.execute_pose(
                                tuple(saved_data["pose"])
                            )

                    else:
                        working_motion_types = [
                            "PTP"
                        ] * len(precondition_names)

                        success = self.run_blended_sequence(
                            precondition_names,
                            precondition_radii,
                            working_motion_types,
                            finish_when_done=False,
                        )

                    if not success:
                        if self.stop_sequence_requested:
                            self.root.after(
                                0,
                                lambda: self.finish_sequence(
                                    "Sequence stopped."
                                ),
                            )
                            return

                        self.root.after(
                            0,
                            lambda: self.finish_sequence(
                                "FAILED: Sequence segment could not be executed."
                            ),
                        )
                        return

                    segment_start = index

                if not self.wait_for_sequence_input(
                    input_condition,
                    input_states_to_run[index],
                ):
                    self.root.after(
                        0,
                        lambda: self.finish_sequence(
                            "Sequence stopped."
                        ),
                    )
                    return False

            if (
                hasattr(self, "active_sequence_output_timings")
                and absolute_index < len(self.active_sequence_output_timings)
                and self.active_sequence_output_timings[absolute_index] == "BEFORE"
            ):
                self.apply_sequence_output(absolute_index)

            segment_names = sequence_to_run[
                segment_start:index + 1
            ]

            segment_radii = blend_radii_to_run[
                segment_start:index + 1
            ]

            working_motion_types = [
                "PTP"
            ] * len(segment_names)
            
            print(
                "DEBUG MIXED SEGMENT:",
                "index=", index,
                "segment_start=", segment_start,
                "segment_names=", segment_names,
                "segment_radii=", segment_radii,
                "working_motion_types=", working_motion_types,
                flush=True,
            )
            success = self.run_blended_sequence(
                segment_names,
                segment_radii,
                working_motion_types,
                finish_when_done=False,
            )
                
            if not success:
                if self.stop_sequence_requested:
                    self.root.after(
                        0,
                        lambda: self.finish_sequence(
                            "Sequence stopped."
                        ),
                    )
                    return

                self.root.after(
                    0,
                    lambda: self.finish_sequence(
                        "FAILED: Sequence segment could not be executed."
                    ),
                )
                return

            if (
                has_input_condition
                and input_timing == "AFTER"
            ):
                if not self.wait_for_sequence_input(
                    input_condition,
                    input_states_to_run[index],
                ):
                    self.root.after(
                        0,
                        lambda: self.finish_sequence(
                            "Sequence stopped."
                        ),
                    )
                    return False

            output_timing = (
                self.active_sequence_output_timings[absolute_index]
                if (
                    hasattr(self, "active_sequence_output_timings")
                    and absolute_index < len(self.active_sequence_output_timings)
                )
                else "AFTER"
            )

            if output_timing == "AFTER":
                self.apply_sequence_output(absolute_index)
                
            if not is_final_pose:
                stop_name = sequence_to_run[index]

                self.root.after(
                    0,
                    lambda name=stop_name: self.status_var.set(
                        f'STOP at "{name}" — waiting '
                        f'{STOP_DWELL_SECONDS:.1f} s...'
                    ),
                )

                sleep(STOP_DWELL_SECONDS)

                if self.stop_sequence_requested:
                    self.root.after(
                        0,
                        lambda: self.finish_sequence(
                            "Sequence stopped."
                        ),
                    )
                    return

            segment_start = index + 1

        if finish_when_done:
            self.root.after(
                0,
                lambda: self.finish_sequence(
                    "Sequence completed successfully."
                ),
            )

        return True

    def saved_position_is_current(self, saved_data):
        pose_matches = False
        joint_matches = False

        if self.current_pose is not None:
            target_pose = saved_data["pose"]

            position_error = sqrt(
                (target_pose[0] - self.current_pose[0]) ** 2
                + (target_pose[1] - self.current_pose[1]) ** 2
                + (target_pose[2] - self.current_pose[2]) ** 2
            )

            angle_error = max(
                abs(angle_difference(target_pose[3], self.current_pose[3])),
                abs(angle_difference(target_pose[4], self.current_pose[4])),
                abs(angle_difference(target_pose[5], self.current_pose[5])),
            )

            pose_matches = (
                position_error < 0.01
                and angle_error < 3.0
            )

        target_joints = saved_data.get("joints")

        if (
            target_joints is not None
            and self.current_joint_positions is not None
        ):
            joint_error = max(
                abs(
                    degrees(
                        atan2(
                            sin(target - current),
                            cos(target - current),
                        )
                    )
                )
                for target, current in zip(
                    target_joints,
                    self.current_joint_positions,
                )
            )

            joint_matches = joint_error < 2.0

        print(
            "DEBUG CURRENT CHECK:",
            "position_error=",
            position_error if self.current_pose is not None else None,
            "angle_error=",
            angle_error if self.current_pose is not None else None,
            "joint_error=",
            joint_error
            if (
                target_joints is not None
                and self.current_joint_positions is not None
            )
            else None,
            "pose_matches=",
            pose_matches,
            "joint_matches=",
            joint_matches,
            flush=True,
        )

        return pose_matches or joint_matches


    def run_blended_sequence(
        self,
        sequence_to_run,
        blend_radii_to_run,
        motion_types_to_run,
        finish_when_done=True,
    ):
    
        sequence_to_run = list(sequence_to_run)
        blend_radii_to_run = list(blend_radii_to_run)
        motion_types_to_run = list(motion_types_to_run)        
        skipped_poses = []

        while sequence_to_run:
            first_name = sequence_to_run[0]
            first_data = self.saved_poses.get(first_name)

            if (
                first_name.lower() == "home"
                or first_data is None
                or not self.saved_position_is_current(first_data)
            ):
                break

            skipped_poses.append(sequence_to_run.pop(0))

            if blend_radii_to_run:
                blend_radii_to_run.pop(0)
            if motion_types_to_run:
                motion_types_to_run.pop(0)

        if not sequence_to_run:
            if finish_when_done:
                skipped_text = ", ".join(skipped_poses)
                self.root.after(
                    0,
                    lambda: self.finish_sequence(
                        f"Sequence complete: already at {skipped_text}."
                    ),
                )
            return True

  

        if not self.sequence_action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.root.after(
                0,
                lambda: self.finish_sequence(
                    "FAILED: Continuous sequence server is unavailable."
                ),
            )
            return

        goal = MoveGroupSequence.Goal()

        for index, pose_name in enumerate(sequence_to_run):
            if pose_name not in self.saved_poses:
                self.root.after(
                    0,
                    lambda name=pose_name: self.finish_sequence(
                        f'FAILED: Pose "{name}" no longer exists.'
                    ),
                )
                return

            saved_data = self.saved_poses[pose_name]
            motion_type = motion_types_to_run[index]
            target = tuple(saved_data["pose"])
            target_joints = saved_data.get("joints")

            request = MotionPlanRequest()
            request.group_name = "arm"
            request.pipeline_id = (
                "pilz_industrial_motion_planner"
            )
            request.planner_id = motion_type
            request.num_planning_attempts = 1
            request.allowed_planning_time = 10.0
            speed_scale = self.get_speed_scale()
            request.max_velocity_scaling_factor = speed_scale
            request.max_acceleration_scaling_factor = speed_scale
            if motion_type == "LIN":
                request.goal_constraints.append(
                    self.create_sequence_constraints(target)
                )
            elif target_joints is not None:
                request.goal_constraints.append(
                    self.create_joint_constraints(target_joints)
                )
            else:
                request.goal_constraints.append(
                    self.create_sequence_constraints(target)
                )


            item = MotionSequenceItem()
            item.req = request

            if index == len(sequence_to_run) - 1:
                item.blend_radius = 0.0
            else:
                item.blend_radius = blend_radii_to_run[index]

            print(
                "DEBUG SEND ITEM:",
                "index=", index,
                "pose=", pose_name,
                "motion=", motion_type,
                "target_joints=", target_joints,
                "blend_radius=", item.blend_radius,
                flush=True,
            )

            goal.request.items.append(item)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        self.root.after(
            0,
            lambda: self.status_var.set(
                "Planning blended sequence..."
            ),
        )

        send_future = self.sequence_action_client.send_goal_async(
            goal
        )

        send_wait_count = 0

        while not send_future.done():
            sleep(0.05)
            send_wait_count += 1

            if send_wait_count >= 1200:
                self.root.after(
                    0,
                    lambda: self.finish_sequence(
                        "Sequence finished, but MoveIt did not return the goal response."
                    ),
                )
                return

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.root.after(
                0,
                lambda: self.finish_sequence(
                    "FAILED: Continuous sequence was rejected."
                ),
            )
            return

        self.sequence_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()

        current_status_index = 0
        self.sequence_resume_index = self.sequence_resume_base_index

        self.root.after(
            0,
            lambda name=sequence_to_run[0],
            total=len(sequence_to_run): self.status_var.set(
                f'Moving to "{name}" (1/{total})...'
            ),
        )

        final_saved_data = self.saved_poses[
            sequence_to_run[-1]
        ]
        final_target = tuple(final_saved_data["pose"])
        final_target_joints = final_saved_data.get("joints")

        final_reached_count = 0
        motion_detected = False
        stationary_count = 0
        last_observed_pose = self.current_pose

        try:
            initial_pose = self.current_pose
            initial_distance = sqrt(
                (final_target[0] - initial_pose[0]) ** 2
                + (final_target[1] - initial_pose[1]) ** 2
                + (final_target[2] - initial_pose[2]) ** 2
            )
            final_was_left = initial_distance > 0.01
        except Exception:
            final_was_left = True

        while not result_future.done():

            if self.auto_jig_running and self.auto_pause_requested:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass

                try:
                    request = CancelGoal.Request()
                    self.controller_cancel_client.call_async(request)
                except Exception:
                    pass

                self.sequence_goal_handle = None

                print(
                    "AUTOMATIC MOTION CANCELLED FOR PAUSE",
                    flush=True,
                )

                return False

            if self.stop_sequence_requested:
                goal_handle.cancel_goal_async()
                self.sequence_goal_handle = None

                self.root.after(
                    0,
                    lambda: self.finish_sequence(
                        "Continuous sequence stopped."
                    ),
                )
                return

            try:
                current = self.current_pose

                if current_status_index < len(sequence_to_run) - 1:
                    status_target = self.saved_poses[
                        sequence_to_run[current_status_index]
                    ]["pose"]

                    status_position_error = sqrt(
                        (status_target[0] - current[0]) ** 2
                        + (status_target[1] - current[1]) ** 2
                        + (status_target[2] - current[2]) ** 2
                    )

                    status_angle_error = max(
                        abs(
                            angle_difference(
                                status_target[3],
                                current[3],
                            )
                        ),
                        abs(
                            angle_difference(
                                status_target[4],
                                current[4],
                            )
                        ),
                        abs(
                            angle_difference(
                                status_target[5],
                                current[5],
                            )
                        ),
                    )

                    if (
                        status_position_error < 0.02
                        and status_angle_error < 5.0
                    ):
                        self.apply_sequence_output(
                            self.sequence_resume_base_index
                            + current_status_index
                        )

                        current_status_index += 1
                        self.sequence_resume_index = (
                            self.sequence_resume_base_index
                            + current_status_index
                        )

                        self.root.after(
                            0,
                            lambda index=current_status_index,
                            name=sequence_to_run[current_status_index],
                            total=len(sequence_to_run):
                                self.status_var.set(
                                    f'Moving to "{name}" '
                                    f'({index + 1}/{total})...'
                                ),
                        )

                observed_position_change = sqrt(
                    (current[0] - last_observed_pose[0]) ** 2
                    + (current[1] - last_observed_pose[1]) ** 2
                    + (current[2] - last_observed_pose[2]) ** 2
                )

                observed_angle_change = max(
                    abs(angle_difference(current[3], last_observed_pose[3])),
                    abs(angle_difference(current[4], last_observed_pose[4])),
                    abs(angle_difference(current[5], last_observed_pose[5])),
                )

                if (
                    observed_position_change > 0.0005
                    or observed_angle_change > 0.1
                ):
                    motion_detected = True
                    stationary_count = 0
                elif motion_detected:
                    stationary_count += 1

                last_observed_pose = current
                position_error = sqrt(
                    (final_target[0] - current[0]) ** 2
                    + (final_target[1] - current[1]) ** 2
                    + (final_target[2] - current[2]) ** 2
                )

                angle_error = max(
                    abs(
                        angle_difference(
                            final_target[3],
                            current[3],
                        )
                    ),
                    abs(
                        angle_difference(
                            final_target[4],
                            current[4],
                        )
                    ),
                    abs(
                        angle_difference(
                            final_target[5],
                            current[5],
                        )
                    ),
                )

                cartesian_reached = (
                    position_error < 0.01
                    and angle_error < 3.0
                )

                joint_reached = False

                if (
                    final_target_joints is not None
                    and self.current_joint_positions is not None
                ):
                    joint_error = max(
                        abs(
                            degrees(
                                atan2(
                                    sin(target_value - current_value),
                                    cos(target_value - current_value),
                                )
                            )
                        )
                        for target_value, current_value in zip(
                            final_target_joints,
                            self.current_joint_positions,
                        )
                    )

                    joint_reached = joint_error < 2.0

                at_final_pose = (
                    cartesian_reached or joint_reached
                )
                watchdog_finished = (
                    motion_detected
                    and stationary_count >= 10
                    and position_error < 0.05
                    and angle_error < 10.0
                )

                if not at_final_pose:
                
                    final_was_left = True
                    final_reached_count = 0
                elif final_was_left:
                    final_reached_count += 1


            except Exception as error:
                print(
                    "COMPLETION CHECK ERROR:",
                    repr(error),
                    flush=True,
                )

            sleep(0.10)

        wrapped_result = result_future.result()
        self.sequence_goal_handle = None

        success = (
            wrapped_result is not None
            and wrapped_result.result.response.error_code.val == 1
        )

        if success:
            self.apply_sequence_output(
                self.sequence_resume_base_index
                + len(skipped_poses)
                + len(sequence_to_run)
                - 1
            )

        if finish_when_done:
            final_message = (
                "Continuous sequence completed successfully."
                if success
                else "FAILED: Continuous sequence could not be executed."
            )

            self.root.after(
                0,
                lambda message=final_message: self.finish_sequence(
                    message
                ),
            )

        return success
        

    def create_joint_constraints(self, joint_positions):
        constraints = Constraints()

        for joint_name, position in zip(
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            joint_positions,
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def create_sequence_constraints(self, target):

        x, y, z, roll, pitch, yaw = target

        tcp_quaternion = rpy_to_quaternion(
            roll,
            pitch,
            yaw,
        )
        tcp_offset_quaternion = rpy_to_quaternion(
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        )

        flange_quaternion = quaternion_multiply(
            tcp_quaternion,
            quaternion_conjugate(tcp_offset_quaternion),
        )

        rotated_translation = rotate_vector(
            flange_quaternion,
            self.tcp_offset[:3],
        )

        target_pose = Pose()
        target_pose.position.x = x - rotated_translation[0]
        target_pose.position.y = y - rotated_translation[1]
        target_pose.position.z = z - rotated_translation[2]
        target_pose.orientation.x = flange_quaternion[0]
        target_pose.orientation.y = flange_quaternion[1]
        target_pose.orientation.z = flange_quaternion[2]
        target_pose.orientation.w = flange_quaternion[3]
        tolerance_shape = SolidPrimitive()
        tolerance_shape.type = SolidPrimitive.SPHERE
        tolerance_shape.dimensions = [0.002]

        position_region = BoundingVolume()
        position_region.primitives.append(tolerance_shape)
        position_region.primitive_poses.append(target_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "link_6"
        position_constraint.constraint_region = position_region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "link_6"
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = radians(2.0)
        orientation_constraint.absolute_y_axis_tolerance = radians(2.0)
        orientation_constraint.absolute_z_axis_tolerance = radians(2.0)
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(
            orientation_constraint
        )

        return constraints

    def stop_active_motion(self):
        if not self.busy:
            return

        self.motion_cancel_requested = True
        self.status_var.set("Stopping active movement...")
        self.stop_motion_button.config(state="disabled")

        if self.gripper_motion_active:
            if self.gripper_goal_handle is not None:
                self.gripper_goal_handle.cancel_goal_async()
            return

        if self.sequence_running:
            self.stop_sequence_requested = True

            if self.sequence_goal_handle is not None:
                self.sequence_goal_handle.cancel_goal_async()
            return

        try:
            self.moveit2.cancel_execution()
        except Exception as error:
            self.status_var.set(f"STOP ERROR: {error}")

    def resume_sequence(self):
        if (
            self.busy
            or self.sequence_running
            or not self.sequence_paused
        ):
            return

        self.sync_sequence_from_rows()

        if self.sequence_resume_index >= len(self.sequence):
            return

        resume_index = self.sequence_resume_index

        sequence_to_run = list(
            self.sequence[resume_index:]
        )
        blend_radii_to_run = list(
            self.sequence_blend_radii[resume_index:]
        )
        motion_types_to_run = list(
            self.sequence_motion_types[resume_index:]
        )
        input_numbers_to_run = list(
            self.sequence_input_numbers[resume_index:]
        )
        input_states_to_run = list(
            self.sequence_input_states[resume_index:]
        )

        self.sequence_running = True
        self.sequence_paused = False
        self.stop_sequence_requested = False
        self.motion_cancel_requested = False
        self.set_busy(True)

        self.run_sequence_button.config(state="disabled")
        self.pause_sequence_button.config(state="normal")
        self.resume_sequence_button.config(state="disabled")
        self.stop_sequence_button.config(state="normal")

        self.status_var.set(
            f"Resuming sequence from point "
            f"{resume_index + 1}..."
        )

        Thread(
            target=self.run_sequence,
            args=(
                sequence_to_run,
                blend_radii_to_run,
                motion_types_to_run,
                input_numbers_to_run,
                input_states_to_run,
                resume_index,
            ),
            daemon=True,
        ).start()


    def pause_sequence(self):
        if not self.sequence_running:
            return

        self.sequence_paused = True
        self.stop_sequence_requested = True
        self.motion_cancel_requested = True

        self.status_var.set(
            f"Sequence stopped at point "
            f"{self.sequence_resume_index + 1}."
        )

        self.pause_sequence_button.config(state="disabled")

        if self.sequence_goal_handle is not None:
            try:
                self.sequence_goal_handle.cancel_goal_async()
            except Exception:
                pass

        try:
            request = CancelGoal.Request()
            self.controller_cancel_client.call_async(request)
        except Exception:
            pass

        if (
            self.sequence_window is not None
            and self.sequence_window.winfo_exists()
        ):
            self.run_sequence_button.config(state="disabled")
            self.pause_sequence_button.config(state="disabled")
            self.resume_sequence_button.config(state="disabled")
            self.stop_sequence_button.config(state="normal")


    def stop_sequence(self):
        if not self.sequence_running and not self.sequence_paused:
            return

        self.stop_sequence_requested = True
        self.motion_cancel_requested = True

        self.status_var.set("Stopping sequence...")
        self.stop_sequence_button.config(state="disabled")

        if self.sequence_goal_handle is not None:
            try:
                self.sequence_goal_handle.cancel_goal_async()
            except Exception:
                pass

        try:
            request = CancelGoal.Request()
            self.controller_cancel_client.call_async(request)
        except Exception:
            pass
        self.sequence_running = False
        self.sequence_paused = False
        self.sequence_resume_index = 0
        self.sequence_resume_base_index = 0
        self.set_busy(False)
        self.status_var.set("Sequence exited.")

        if (
            self.sequence_window is not None
            and self.sequence_window.winfo_exists()
        ):
            self.run_sequence_button.config(state="normal")
            self.pause_sequence_button.config(state="disabled")
            self.resume_sequence_button.config(state="disabled")
            self.stop_sequence_button.config(state="disabled")
    def finish_sequence(self, message):
        was_stopped = self.stop_sequence_requested

        self.sequence_running = False
        self.stop_sequence_requested = False
        self.set_busy(False)

        if self.sequence_paused:
            self.status_var.set(
                f"Sequence stopped at point "
                f"{self.sequence_resume_index + 1}."
            )

            if (
                self.sequence_window is not None
                and self.sequence_window.winfo_exists()
            ):
                self.run_sequence_button.config(state="normal")
                self.pause_sequence_button.config(state="disabled")
                self.resume_sequence_button.config(state="normal")
                self.stop_sequence_button.config(state="normal")

            return

        if was_stopped:
            message = "Sequence exited."

        self.sequence_resume_index = 0
        self.sequence_resume_base_index = 0
        self.status_var.set(message)

        if (
            self.sequence_window is not None
            and self.sequence_window.winfo_exists()
        ):
            self.run_sequence_button.config(state="normal")
            self.pause_sequence_button.config(state="disabled")
            self.resume_sequence_button.config(state="disabled")
            self.stop_sequence_button.config(state="disabled")


    def validate_joint_targets(self, target):
        for index, value in enumerate(target):
            value_degrees = degrees(value)
            minimum = JOINT_MINIMUM_DEGREES[index]
            maximum = JOINT_MAXIMUM_DEGREES[index]

            if value_degrees < minimum or value_degrees > maximum:
                messagebox.showwarning(
                    "Joint Limit Rejection",
                    f"J{index + 1} target is {value_degrees:.1f}°.\n"
                    f"Allowed range: {minimum:.1f}° to {maximum:.1f}°.",
                )
                return False

        return True

    def request_jog(self, axis, step):
        if self.control_mode.get() == "joint":
            if self.busy or self.current_joint_positions is None:
                return

            axis_order = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            joint_index = axis_order.index(axis)

            joint_step = (
                JOG_ANGLE_STEP
                if step > 0.0
                else -JOG_ANGLE_STEP
            )
            if joint_index == 2:
                joint_step = -joint_step

            target = list(self.current_joint_positions)
            target[joint_index] += radians(joint_step)
            if not self.validate_joint_targets(target):
                return

            for number, value in enumerate(target, start=1):
                self.target_joint_vars[f"J{number}"].set(
                    f"{degrees(value):.1f}"
                )

            self.set_busy(True)
            self.status_var.set(
                f"Jogging J{joint_index + 1} by {joint_step:+.1f}°..."
            )

            Thread(
                target=self.execute_joint_positions,
                args=(tuple(target),),
                daemon=True,
            ).start()
            return

        if self.busy or self.current_pose is None:
            return

        target = list(self.current_pose)
        frame_mode = self.jog_frame_mode.get()

        tcp_quaternion = rpy_to_quaternion(
            self.current_pose[3],
            self.current_pose[4],
            self.current_pose[5],
        )

        tcp_offset_quaternion = rpy_to_quaternion(
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        )

        flange_quaternion = quaternion_multiply(
            tcp_quaternion,
            quaternion_conjugate(tcp_offset_quaternion),
        )

        if axis in ["X", "Y", "Z"]:
            direction = {
                "X": [step, 0.0, 0.0],
                "Y": [0.0, step, 0.0],
                "Z": [0.0, 0.0, step],
            }[axis]

            if frame_mode == "TOOL":
                direction = rotate_vector(
                    tcp_quaternion,
                    direction,
                )
            elif frame_mode == "FLANGE":
                direction = rotate_vector(
                    flange_quaternion,
                    direction,
                )

            target[0] += direction[0]
            target[1] += direction[1]
            target[2] += direction[2]

        else:
            rotation_axis = {
                "Roll": "Y",
                "Pitch": "X",
                "Yaw": "Z",
            }[axis]

            delta_quaternion = axis_angle_quaternion(
                rotation_axis,
                step,
            )

            if frame_mode == "TOOL":
                target_quaternion = quaternion_multiply(
                    tcp_quaternion,
                    delta_quaternion,
                )
            elif frame_mode == "FLANGE":
                world_delta = quaternion_multiply(
                    quaternion_multiply(
                        flange_quaternion,
                        delta_quaternion,
                    ),
                    quaternion_conjugate(flange_quaternion),
                )
                target_quaternion = quaternion_multiply(
                    world_delta,
                    tcp_quaternion,
                )
            else:
                target_quaternion = quaternion_multiply(
                    delta_quaternion,
                    tcp_quaternion,
                )

            target[3], target[4], target[5] = quaternion_to_rpy(
                target_quaternion[0],
                target_quaternion[1],
                target_quaternion[2],
                target_quaternion[3],
            )

        names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]

        for name, value in zip(names, target):
            decimals = 3 if name in ["X", "Y", "Z"] else 1
            self.target_vars[name].set(f"{value:.{decimals}f}")

        self.set_busy(True)

        self.status_var.set(
            f"Jogging {axis} by {step:+.3f}..."
            if axis in ["X", "Y", "Z"]
            else f"Jogging {axis} by {step:+.1f}°..."
        )

        Thread(
            target=self.execute_pose,
            args=(tuple(target), True),
            daemon=True,
        ).start()


    def clear_all_digital_inputs(self):
        for input_variable in self.digital_inputs.values():
            input_variable.set(False)

        self.status_var.set("All digital inputs cleared.")

    def turn_all_digital_outputs_off(self):
        for number in range(1, 7):
            self.set_digital_output(number, False)

        self.status_var.set("All digital outputs turned OFF.")

    def get_digital_outputs(self):
        if not hasattr(self, "_digital_output_states"):
            self._digital_output_states = {
                number: False
                for number in range(1, 7)
            }

        return self._digital_output_states

    def set_digital_output(self, number, state):
        outputs = self.get_digital_outputs()

        if number not in outputs:
            return

        outputs[number] = bool(state)

        if hasattr(self, "output_lamps") and number in self.output_lamps:
            self.output_lamps[number].config(
                text="●",
                fg="green" if outputs[number] else "gray",
            )
    def get_digital_output(self, number):
        outputs = self.get_digital_outputs()
        return outputs.get(number, False)

    def apply_sequence_output(self, absolute_index):
        if not hasattr(self, "active_sequence_output_numbers"):
            return

        if absolute_index < 0:
            return

        if absolute_index >= len(self.active_sequence_output_numbers):
            return

        output_name = self.active_sequence_output_numbers[absolute_index]

        if output_name == "NONE":
            return

        if output_name in ("OPEN GRIPPER", "CLOSE GRIPPER"):
            return

        if not output_name.startswith("OUT "):
            return

        output_number = int(output_name.split()[-1])

        output_state = (
            self.active_sequence_output_states[absolute_index] == "TRUE"
        )

        self.root.after(
            0,
            lambda number=output_number, state=output_state:
                self.set_digital_output(number, state),
        )

    def get_sequence_gripper_action(self, absolute_index):
        if not hasattr(self, "active_sequence_output_numbers"):
            return None

        if absolute_index < 0:
            return None

        if absolute_index >= len(self.active_sequence_output_numbers):
            return None

        output_name = self.active_sequence_output_numbers[absolute_index]

        if output_name == "OPEN GRIPPER":
            return 1.75, "OPEN GRIPPER"

        if output_name == "CLOSE GRIPPER":
            return 0.04, "CLOSE GRIPPER"

        return None

    def set_busy(self, busy):
        self.busy = busy
        if busy:
            self.motion_cancel_requested = False

        self.stop_motion_button.config(
            state="normal" if busy else "disabled"
        )
        state = "disabled" if busy else "normal"
        self.mode_button.config(state=state)
        self.tcp_button.config(state=state)
        self.speed_selector.config(
            state="disabled" if busy else "readonly"
        )
        
        for button in self.jog_buttons:
            button.config(state=state)

        self.move_button.config(state=state)
        self.home_button.config(state=state)
        self.open_gripper_button.config(state=state)
        self.close_gripper_button.config(state=state)
        self.copy_button.config(state=state)
        self.save_pose_button.config(state=state)
        self.load_pose_button.config(state=state)
        self.delete_pose_button.config(state=state)
        self.go_pose_button.config(state=state)
        self.sequence_button.config(state=state)
        self.pose_selector.config(
            state="disabled" if busy else "readonly"
        )

    def request_pose_move(self):
        if self.control_mode.get() == "joint":
            if self.busy or self.current_joint_positions is None:
                return

            try:
                target = tuple(
                    radians(
                        -float(self.target_joint_vars[f"J{number}"].get())
                        if number == 3
                        else float(self.target_joint_vars[f"J{number}"].get())
                    )
                    for number in range(1, 7)
                )
                
            except ValueError:
                messagebox.showerror(
                    "Invalid Input",
                    "Enter valid angles for all six joints.",
                )
                return
            if not self.validate_joint_targets(target):
                return

            maximum_change = max(
                abs(degrees(target_value - current_value))
                for target_value, current_value in zip(
                    target,
                    self.current_joint_positions,
                )
            )

            approved = messagebox.askyesno(
                "Confirm Joint Movement",
                f"Maximum joint change: {maximum_change:.1f}°\n\n"
                "Move the robot?",
            )

            if not approved:
                return

            self.set_busy(True)
            self.status_var.set(
                "Planning and executing target joint angles..."
            )

            Thread(
                target=self.execute_joint_positions,
                args=(target,),
                daemon=True,
            ).start()
            return

        if self.busy or self.current_pose is None:
            return

        try:
            target = tuple(
                float(self.target_vars[name].get())
                for name in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            )
        except ValueError:
            messagebox.showerror(
                "Invalid Input",
                "Enter valid numbers in all six boxes.",
            )
            return

        x, y, z, roll, pitch, yaw = target
        cx, cy, cz, croll, cpitch, cyaw = self.current_pose

        distance = sqrt(
            (x - cx) ** 2
            + (y - cy) ** 2
            + (z - cz) ** 2
        )

        angle_change = max(
            abs(angle_difference(roll, croll)),
            abs(angle_difference(pitch, cpitch)),
            abs(angle_difference(yaw, cyaw)),
        )

        if distance > MAXIMUM_STEP_DISTANCE:
            messagebox.showwarning(
                "Safety Rejection",
                "The requested position movement is greater than 0.10 m.",
            )
            return

        if angle_change > MAXIMUM_ANGLE_CHANGE:
            messagebox.showwarning(
                "Safety Rejection",
                "The requested orientation change is greater than 15°.",
            )
            return

        approved = messagebox.askyesno(
            "Confirm Movement",
            f"Position movement: {distance:.3f} m\n"
            f"Orientation change: {angle_change:.1f}°\n\n"
            "Move the robot?",
        )

        if not approved:
            return

        self.set_busy(True)
        self.status_var.set("Planning and executing target pose...")

        Thread(
            target=self.execute_pose,
            args=(target,),
            daemon=True,
        ).start()

    def execute_joint_positions(self, target):
        success = False
        try:
            speed_scale = self.get_speed_scale()
            self.moveit2.max_velocity = speed_scale
            self.moveit2.max_acceleration = speed_scale

            trajectory = self.moveit2.plan(
                joint_positions=list(target),
                start_joint_state=list(self.current_joint_positions),
            )

            if trajectory is None:
                success = False
            else:
                self.moveit2.execute(trajectory)
                success = self.moveit2.wait_until_executed()

            if self.motion_cancel_requested:
                message = "Movement stopped by user."
            elif success:
                message = "SUCCESS: Target joint angles reached."
            else:
                message = (
                    "FAILED: Target joint angles could not be reached."
                )

        except Exception as error:
            message = f"ERROR: {error}"

        self.root.after(
            0,
            lambda: self.finish_movement(message),
        )
        return success

    def execute_pose(self, target, cartesian=False):

        x, y, z, roll, pitch, yaw = target

        tcp_quaternion = rpy_to_quaternion(
            roll,
            pitch,
            yaw,
        )
        tcp_offset_quaternion = rpy_to_quaternion(
            self.tcp_offset[3],
            self.tcp_offset[4],
            self.tcp_offset[5],
        )

        flange_quaternion = quaternion_multiply(
            tcp_quaternion,
            quaternion_conjugate(tcp_offset_quaternion),
        )

        rotated_translation = rotate_vector(
            flange_quaternion,
            self.tcp_offset[:3],
        )

        flange_position = [
            x - rotated_translation[0],
            y - rotated_translation[1],
            z - rotated_translation[2],
        ]

        move_success = False

        try:
            controller_success = False
            speed_scale = self.get_speed_scale()
            self.moveit2.max_velocity = speed_scale
            self.moveit2.max_acceleration = speed_scale

            trajectory = self.moveit2.plan(
                position=flange_position,
                quat_xyzw=flange_quaternion,
                start_joint_state=list(self.current_joint_positions),
                cartesian=cartesian,
            )

            if trajectory is None:
                controller_success = False
            else:
                self.moveit2.execute(trajectory)
                controller_success = self.moveit2.wait_until_executed()
            if self.motion_cancel_requested:
                self.root.after(
                    0,
                    lambda: self.finish_movement(
                        "Movement stopped by user."
                    ),
                )
                return False


            final_pose = self.read_pose()
            fx, fy, fz, froll, fpitch, fyaw = final_pose

            position_error = sqrt(
                (x - fx) ** 2
                + (y - fy) ** 2
                + (z - fz) ** 2
            )

            orientation_error = max(
                abs(angle_difference(roll, froll)),
                abs(angle_difference(pitch, fpitch)),
                abs(angle_difference(yaw, fyaw)),
            )

            measured_success = (
                position_error <= 0.005
                and orientation_error <= 1.0
            )

            move_success = controller_success or measured_success
            print(
                "LIN RESULT:",
                "controller_success =", controller_success,
                "position_error =", position_error,
                "orientation_error =", orientation_error,
                "measured_success =", measured_success,
                flush=True,
            )


            if move_success:
                message = (
                    "SUCCESS\n"
                    f"Position error: {position_error:.4f} m\n"
                    f"Orientation error: {orientation_error:.2f}°"
                )
            else:
                message = (
                    "FAILED: Target pose was not reached.\n"
                    f"Position error: {position_error:.4f} m\n"
                    f"Orientation error: {orientation_error:.2f}°"
                )

        except Exception as error:
            message = f"ERROR: {error}"

        self.root.after(
            0,
            lambda: self.finish_movement(message),
        )
        return move_success

    def request_home(self):
        if self.busy:
            return

        approved = messagebox.askyesno(
            "Return Home",
            "Return all six joints exactly to zero?",
        )

        if not approved:
            return

        self.set_busy(True)
        self.status_var.set("Returning to exact joint-zero HOME...")

        Thread(
            target=self.run_blended_sequence,
            args=(["home"], [0.0], ["PTP"]),
            kwargs={"finish_when_done": True},
            daemon=True,
        ).start()

    def execute_home(self):
        try:
            speed_scale = self.get_speed_scale()
            self.moveit2.max_velocity = speed_scale
            self.moveit2.max_acceleration = speed_scale

            self.moveit2.move_to_configuration(
                joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

            success = self.moveit2.wait_until_executed()

            if self.motion_cancel_requested:
                message = "Movement stopped by user."
            elif success:
                message = "SUCCESS: Robot returned to exact HOME."
            else:
                message = (
                    "The controller reported ABORTED. "
                    "Check the measured pose before trying again."
                )

        except Exception as error:
            message = f"ERROR: {error}"

        self.root.after(
            0,
            lambda: self.finish_home(message),
        )
        
    def finish_home(self, message):
        self.finish_movement(message)
        self.root.after(500, self.copy_current_pose)


    def finish_movement(self, message):
        self.status_var.set(message)

        if (
            not self.sequence_running
            and not self.auto_jig_running
        ):
            self.set_busy(False)

    def close(self):
        if self.busy:
            messagebox.showwarning(
                "Robot Is Moving",
                "Wait until the current movement finishes.",
            )
            return

        self.root.destroy()


def main():
    rclpy.init()

    node = Node("robot_arm_gui")
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

    moveit2.pipeline_id = "pilz_industrial_motion_planner"
    moveit2.planner_id = "PTP"
    moveit2.max_velocity = 0.10
    moveit2.max_acceleration = 0.10

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    executor = rclpy.executors.MultiThreadedExecutor(
        num_threads=6
    )

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )


    root = tk.Tk()
    app = RobotGUI(root, node, moveit2, tf_buffer)
    executor.add_node(node)
    executor_thread.start()
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
