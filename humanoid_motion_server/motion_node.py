"""Humanoid motion control node powered by Pinocchio kinematics and CLIK IK."""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from humanoid_motion_interfaces.action import MoveJ, MoveL, MoveP
from humanoid_motion_interfaces.msg import MotionOptions, Status

from .channel_config import ChannelConfig, load_channels
from .kinematics import PinocchioKinematics
from .solve_ik import solve_ik
from .tasks import FrameTask, JointTask
from .trajectory import CartesianTrajectory, JointTrajectory, TargetStore, TargetTransform


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class HumanoidMotionControlNode(Node):
    def __init__(self):
        super().__init__("humanoid_motion_control", automatically_declare_parameters_from_overrides=True)

        self.cb_group = ReentrantCallbackGroup()

        # Declare standard parameters with defaults
        self._declare_if_not_set("joint_state_endpoint", "/observation/joint_states")
        self._declare_if_not_set("joint_command_endpoint", "/control/motion/joint_targets")
        self._declare_if_not_set("output_permit_topic", "/control/motion/permit")
        self._declare_if_not_set("channel_config_file", "")
        self._declare_if_not_set("urdf_file", "")
        self._declare_if_not_set("sdk_config_file", "")
        self._declare_if_not_set("tool_config_file", "")
        self._declare_if_not_set("robot_api_file", "")
        self._declare_if_not_set("control_frequency_hz", 100.0)
        self._declare_if_not_set("servo_lease_ms", 100.0)
        self._declare_if_not_set("feedback_max_age_ms", 200.0)
        self._declare_if_not_set("default_move_timeout_s", 60.0)
        self._declare_if_not_set("joint_max_velocity_rad_s", 0.6)
        self._declare_if_not_set("move_j_position_tolerance_rad", 0.01)
        self._declare_if_not_set("cartesian_position_tolerance_m", 0.005)
        self._declare_if_not_set("cartesian_orientation_tolerance_rad", 0.02)
        self._declare_if_not_set("joint_group_names", ["left_arm", "right_arm", "whole_body"])

        # Retrieve parameters
        self.joint_state_endpoint = str(self.get_parameter("joint_state_endpoint").value)
        self.joint_command_endpoint = str(self.get_parameter("joint_command_endpoint").value)
        self.output_permit_topic = str(self.get_parameter("output_permit_topic").value)
        self.channel_config_file = str(self.get_parameter("channel_config_file").value)
        self.urdf_file = str(self.get_parameter("urdf_file").value)
        self.control_frequency_hz = float(self.get_parameter("control_frequency_hz").value)
        self.dt = 1.0 / max(1.0, self.control_frequency_hz)
        self.servo_lease_s = float(self.get_parameter("servo_lease_ms").value) / 1000.0
        self.feedback_max_age_s = float(self.get_parameter("feedback_max_age_ms").value) / 1000.0
        self.default_move_timeout_s = float(self.get_parameter("default_move_timeout_s").value)
        self.default_max_velocity = float(self.get_parameter("joint_max_velocity_rad_s").value)
        self.move_j_tolerance = float(self.get_parameter("move_j_position_tolerance_rad").value)
        self.pos_tolerance = float(self.get_parameter("cartesian_position_tolerance_m").value)
        self.rot_tolerance = float(self.get_parameter("cartesian_orientation_tolerance_rad").value)

        # Parse joint groups
        self.groups: Dict[str, List[str]] = {}
        group_names = self.get_parameter("joint_group_names").value
        if isinstance(group_names, (list, tuple)):
            for gname in group_names:
                param_name = f"groups.{gname}"
                if self.has_parameter(param_name):
                    joints = self.get_parameter(param_name).value
                    if isinstance(joints, (list, tuple)):
                        self.groups[gname] = list(map(str, joints))

        # Resolve URDF and channel configs from share directory if empty
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = Path(get_package_share_directory("humanoid_motion_server"))
        except Exception:
            share_dir = Path(__file__).resolve().parent.parent

        if not self.urdf_file or not Path(self.urdf_file).is_file():
            candidates = [
                share_dir / "urdf/test_humanoid.urdf",
                share_dir / "share/humanoid_motion_server/urdf/test_humanoid.urdf",
            ]
            for candidate in candidates:
                if candidate.is_file():
                    self.urdf_file = str(candidate)
                    break
            if not self.urdf_file:
                self.get_logger().error(f"urdf_file '{self.urdf_file}' not found")

        if not self.channel_config_file or not Path(self.channel_config_file).is_file():
            candidates = [
                share_dir / "config/channels.yaml",
                share_dir / "share/humanoid_motion_server/config/channels.yaml",
            ]
            for candidate in candidates:
                if candidate.is_file():
                    self.channel_config_file = str(candidate)
                    break

        self.channels: List[ChannelConfig] = []
        if self.channel_config_file and Path(self.channel_config_file).is_file():
            self.channels = load_channels(self.channel_config_file)
            self.get_logger().info(f"Loaded {len(self.channels)} channels from {self.channel_config_file}")

        # Gather all free joints
        free_joints_set = set()
        for g_joints in self.groups.values():
            free_joints_set.update(g_joints)
        for ch in self.channels:
            if ch.group in self.groups:
                free_joints_set.update(self.groups[ch.group])

        # If still empty, inspect URDF for scalar joints
        if not free_joints_set and self.urdf_file and Path(self.urdf_file).is_file():
            temp_model = pin.buildModelFromUrdf(self.urdf_file)
            for i in range(1, len(temp_model.joints)):
                joint = temp_model.joints[i]
                if joint.nq == 1 and joint.nv == 1:
                    free_joints_set.add(temp_model.names[i])

        self.free_joint_names = sorted(list(free_joints_set))
        if not self.free_joint_names:
            raise ValueError("No free joints configured or found in URDF")

        # Initialize Pinocchio Kinematics
        self.kinematics = PinocchioKinematics(self.urdf_file, self.free_joint_names)
        self.current_q = self.kinematics.copy_q()
        self.command_q = self.kinematics.copy_q()
        self.have_feedback = False
        self.last_feedback_time = float("-inf")

        # Group mappings to column indices
        self.group_indices: Dict[str, List[int]] = {}
        for gname, jnames in self.groups.items():
            indices = [self.free_joint_names.index(j) for j in jnames if j in self.free_joint_names]
            self.group_indices[gname] = indices

        # Target store for IK
        self.targets = TargetStore()
        self.tasks: Dict[str, List] = {}

        # Tracking state
        self.servo_p_targets: Dict[str, Tuple[str, str, TargetTransform, float]] = {}  # group -> (base, tip, target, stamp)
        self.servo_j_targets: Dict[str, Tuple[Dict[str, float], float]] = {}  # group -> (joint_dict, stamp)
        self.active_actions: Dict[str, any] = {}  # group -> active GoalHandle/context

        # Permit & authorization
        self.permit_key = ""
        self.permit_source = "motion"  # "vr" or "motion"
        self.permit_time = float("-inf")

        # QoS definitions
        latest_reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Joint command publisher
        self.command_pub = self.create_publisher(
            JointState, self.joint_command_endpoint, latest_reliable_qos
        )

        # Joint state subscriber
        self.state_sub = self.create_subscription(
            JointState, self.joint_state_endpoint, self._joint_state_cb, qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        # Permit subscriber
        self.permit_sub = self.create_subscription(
            Header, self.output_permit_topic, self._permit_cb, latest_reliable_qos,
            callback_group=self.cb_group,
        )

        # Create FK publishers, servo subscribers, and action servers for each channel
        self.fk_pubs: Dict[str, any] = {}
        self.servo_subs: Dict[str, any] = {}
        self.action_servers: Dict[str, any] = {}

        for ch in self.channels:
            self._setup_channel(ch)

        # Control loop timer
        self.timer = self.create_timer(self.dt, self._tick, callback_group=self.cb_group)
        self.get_logger().info(
            f"HumanoidMotionControlNode started with {len(self.free_joint_names)} joints, "
            f"rate={self.control_frequency_hz}Hz, URDF={self.urdf_file}"
        )

    def _declare_if_not_set(self, name: str, default_val):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_val)

    def _setup_channel(self, ch: ChannelConfig):
        # FK publisher
        if ch.fk_pose_topic and ch.fk_pose_topic not in self.fk_pubs:
            self.fk_pubs[ch.fk_pose_topic] = (
                self.create_publisher(PoseStamped, ch.fk_pose_topic, 1),
                ch.base_frame,
                ch.tip_frame,
            )

        # ServoP subscriber
        if ch.kind == "servo_p":
            sub = self.create_subscription(
                PoseStamped,
                ch.endpoint,
                lambda msg, c=ch: self._servo_p_cb(msg, c),
                qos_profile_sensor_data,
                callback_group=self.cb_group,
            )
            self.servo_subs[ch.endpoint] = sub

        # ServoJ subscriber
        elif ch.kind == "servo_j":
            sub = self.create_subscription(
                JointState,
                ch.endpoint,
                lambda msg, c=ch: self._servo_j_cb(msg, c),
                qos_profile_sensor_data,
                callback_group=self.cb_group,
            )
            self.servo_subs[ch.endpoint] = sub

        # MoveJ ActionServer
        elif ch.kind == "move_j":
            server = ActionServer(
                self,
                MoveJ,
                ch.endpoint,
                execute_callback=lambda goal_handle, c=ch: self._execute_move_j(goal_handle, c),
                goal_callback=lambda goal_request: self._goal_callback(goal_request, ch),
                cancel_callback=self._cancel_callback,
                callback_group=self.cb_group,
            )
            self.action_servers[ch.endpoint] = server

        # MoveP ActionServer
        elif ch.kind == "move_p":
            server = ActionServer(
                self,
                MoveP,
                ch.endpoint,
                execute_callback=lambda goal_handle, c=ch: self._execute_move_p(goal_handle, c),
                goal_callback=lambda goal_request: self._goal_callback(goal_request, ch),
                cancel_callback=self._cancel_callback,
                callback_group=self.cb_group,
            )
            self.action_servers[ch.endpoint] = server

        # MoveL ActionServer
        elif ch.kind == "move_l":
            server = ActionServer(
                self,
                MoveL,
                ch.endpoint,
                execute_callback=lambda goal_handle, c=ch: self._execute_move_p(goal_handle, c),
                goal_callback=lambda goal_request: self._goal_callback(goal_request, ch),
                cancel_callback=self._cancel_callback,
                callback_group=self.cb_group,
            )
            self.action_servers[ch.endpoint] = server

    def _permit_cb(self, msg: Header):
        fid = msg.frame_id.strip()
        self.permit_key = fid
        self.permit_time = time.monotonic()
        if fid.startswith("vr:") or "#vr#" in fid:
            self.permit_source = "vr"
        elif fid.startswith("motion:") or "#motion#" in fid:
            self.permit_source = "motion"
        elif "hc1:" in fid:
            parts = fid.split("#")
            if len(parts) >= 2 and parts[1] == "grant":
                self.permit_source = "vr" if "vr" in parts[0] else "motion"
            elif len(parts) >= 2 and parts[1] in ("revoke", "off"):
                self.permit_source = "motion"

    def _is_channel_permitted(self, ch: ChannelConfig) -> bool:
        # Check if caller matches current permit
        is_vr_channel = "/vr" in ch.endpoint or ch.priority >= 100 or "teleop" in ch.endpoint or "teleop" in ch.name
        now = time.monotonic()
        has_fresh_permit = (now - self.permit_time) <= 0.5
        if has_fresh_permit:
            if self.permit_source == "vr":
                return is_vr_channel
            else:
                return not is_vr_channel
        return True

    def _joint_state_cb(self, msg: JointState):
        try:
            self.kinematics.update_named_joints(msg.name, msg.position)
            self.current_q = self.kinematics.copy_q()
            if not self.have_feedback:
                self.command_q = self.current_q.copy()
                self.have_feedback = True
            self.last_feedback_time = time.monotonic()
        except Exception as e:
            self.get_logger().warning(f"Error updating joint states: {e}")

    def _servo_p_cb(self, msg: PoseStamped, ch: ChannelConfig):
        if not self._is_channel_permitted(ch):
            return
        p = msg.pose.position
        o = msg.pose.orientation
        rot = quat_to_matrix(o.x, o.y, o.z, o.w)
        trans = np.array([p.x, p.y, p.z], dtype=float)
        target = TargetTransform(trans, rot)
        now = time.monotonic()
        self.servo_p_targets[ch.group] = (ch.base_frame, ch.tip_frame, target, now)

    def _servo_j_cb(self, msg: JointState, ch: ChannelConfig):
        if not self._is_channel_permitted(ch):
            return
        if len(msg.name) != len(msg.position):
            return
        mapping = {n: float(v) for n, v in zip(msg.name, msg.position) if math.isfinite(v)}
        now = time.monotonic()
        self.servo_j_targets[ch.group] = (mapping, now)

    def _goal_callback(self, goal_request, ch: ChannelConfig):
        if not self._is_channel_permitted(ch):
            self.get_logger().warning(f"Rejected goal on {ch.endpoint}: channel not permitted by active source")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_move_j(self, goal_handle, ch: ChannelConfig):
        goal = goal_handle.request
        group = ch.group or goal.group_name
        target_msg = goal.target
        target_dict = dict(zip(target_msg.name, target_msg.position))

        group_joints = self.groups.get(group, self.free_joint_names)
        indices = [self.free_joint_names.index(j) for j in group_joints if j in self.free_joint_names]
        if not indices:
            goal_handle.abort()
            result = MoveJ.Result()
            result.status.code = Status.INVALID_REQUEST
            result.status.message = f"Unknown group or joints: {group}"
            return result

        current_positions = np.array([self.command_q[self.kinematics.joints[i].q_index] for i in indices])
        target_positions = np.array([target_dict.get(self.free_joint_names[i], current_positions[k]) for k, i in enumerate(indices)])

        scale = float(goal.options.velocity_scale) if goal.options.velocity_scale > 0.0 else 1.0
        trajectory = JointTrajectory(
            current_positions, target_positions,
            max_velocity=self.default_max_velocity,
            scale=scale,
        )

        timeout = float(goal.options.timeout_sec) if goal.options.timeout_sec > 0.0 else self.default_move_timeout_s
        start_time = time.monotonic()

        self.get_logger().info(f"Executing MoveJ on group '{group}' duration={trajectory.duration:.2f}s")
        rate = self.create_rate(self.control_frequency_hz)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                res = MoveJ.Result()
                res.status.code = Status.CANCELED
                res.status.message = "Goal canceled"
                return res

            if time.monotonic() - start_time > timeout:
                goal_handle.abort()
                res = MoveJ.Result()
                res.status.code = Status.TIMEOUT
                res.status.message = "MoveJ timeout exceeded"
                return res

            pos, progress, done = trajectory.step(self.dt)
            for k, idx in enumerate(indices):
                self.command_q[self.kinematics.joints[idx].q_index] = pos[k]

            feedback = MoveJ.Feedback()
            feedback.progress = progress
            feedback.actual_joint_state = JointState(
                header=Header(stamp=self.get_clock().now().to_msg()),
                name=[self.free_joint_names[i] for i in indices],
                position=[float(self.current_q[self.kinematics.joints[i].q_index]) for i in indices],
            )
            goal_handle.publish_feedback(feedback)

            if done:
                # Check convergence
                err = np.max(np.abs(pos - target_positions))
                if err <= self.move_j_tolerance:
                    break
                else:
                    break

            rate.sleep()

        goal_handle.succeed()
        res = MoveJ.Result()
        res.status.code = Status.OK
        res.status.message = "Success"
        res.final_joint_state = JointState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            name=[self.free_joint_names[i] for i in indices],
            position=[float(self.command_q[self.kinematics.joints[i].q_index]) for i in indices],
        )
        return res

    def _execute_move_p(self, goal_handle, ch: ChannelConfig):
        goal = goal_handle.request
        group = ch.group or goal.group_name
        base_frame = ch.base_frame
        tip_frame = ch.tip_frame or goal.tip_frame

        p = goal.target_pose.pose.position
        o = goal.target_pose.pose.orientation
        rot_target = quat_to_matrix(o.x, o.y, o.z, o.w)
        trans_target = np.array([p.x, p.y, p.z], dtype=float)
        target_se3 = pin.SE3(rot_target, trans_target)

        current_se3 = self.kinematics.relative_pose(base_frame, tip_frame, self.command_q)
        scale = float(goal.options.velocity_scale) if goal.options.velocity_scale > 0.0 else 1.0
        trajectory = CartesianTrajectory(current_se3, target_se3, scale=scale)

        timeout = float(goal.options.timeout_sec) if goal.options.timeout_sec > 0.0 else self.default_move_timeout_s
        start_time = time.monotonic()

        rate = self.create_rate(self.control_frequency_hz)
        group_joints = self.groups.get(group, self.free_joint_names)
        indices = [self.free_joint_names.index(j) for j in group_joints if j in self.free_joint_names]

        task = FrameTask(root=base_frame, frame=tip_frame, position_cost=5.0, orientation_cost=1.0)
        self.get_logger().info(f"Executing MoveP on group '{group}' duration={trajectory.duration:.2f}s")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                res = MoveP.Result()
                res.status.code = Status.CANCELED
                return res

            if time.monotonic() - start_time > timeout:
                goal_handle.abort()
                res = MoveP.Result()
                res.status.code = Status.TIMEOUT
                return res

            des_se3, progress, done = trajectory.step(self.dt)
            self.targets.set_target(base_frame, tip_frame, TargetTransform(des_se3.translation, des_se3.rotation))

            ik_res = solve_ik(self.kinematics, self.command_q, [task], self.targets, dt=self.dt, damping=1e-4)
            self.command_q = self.kinematics.integrate_free(self.command_q, ik_res.velocity, self.dt)
            self.command_q = self.kinematics.clip_configuration(self.command_q)

            feedback = MoveP.Feedback()
            feedback.progress = progress
            cur_p = self.kinematics.relative_pose(base_frame, tip_frame, self.current_q)
            q_cur = pin.Quaternion(cur_p.rotation)
            feedback.actual_pose = PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=base_frame),
                pose=pin_to_pose_msg(cur_p),
            )
            goal_handle.publish_feedback(feedback)

            if done:
                break
            rate.sleep()

        goal_handle.succeed()
        res = MoveP.Result()
        res.status.code = Status.OK
        final_p = self.kinematics.relative_pose(base_frame, tip_frame, self.command_q)
        res.final_pose = PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=base_frame),
            pose=pin_to_pose_msg(final_p),
        )
        return res

    def _tick(self):
        if not self.have_feedback:
            return

        now = time.monotonic()

        # 1. Publish FK for all configured channels
        self._publish_fk()

        # 2. Check if feedback is stale
        if now - self.last_feedback_time > self.feedback_max_age_s:
            # Feedback stale, hold position
            self._publish_command()
            return

        # 3. Process ServoP and ServoJ for each group
        for group, indices in self.group_indices.items():
            servo_p = self.servo_p_targets.get(group)
            servo_j = self.servo_j_targets.get(group)

            # Check ServoP lease
            if servo_p is not None and (now - servo_p[3]) <= self.servo_lease_s:
                base_frame, tip_frame, target, _ = servo_p
                self.targets.set_target(base_frame, tip_frame, target)
                task = FrameTask(root=base_frame, frame=tip_frame, position_cost=5.0, orientation_cost=1.0)
                ik_res = solve_ik(self.kinematics, self.command_q, [task], self.targets, dt=self.dt, damping=1e-4)
                self.command_q = self.kinematics.integrate_free(self.command_q, ik_res.velocity, self.dt)
                self.command_q = self.kinematics.clip_configuration(self.command_q)

            # Check ServoJ lease
            elif servo_j is not None and (now - servo_j[1]) <= self.servo_lease_s:
                target_dict, _ = servo_j
                for idx in indices:
                    name = self.free_joint_names[idx]
                    if name in target_dict:
                        target_val = target_dict[name]
                        q_idx = self.kinematics.joints[idx].q_index
                        cur_val = self.command_q[q_idx]
                        max_step = self.default_max_velocity * self.dt
                        delta = target_val - cur_val
                        step = max(-max_step, min(max_step, delta))
                        self.command_q[q_idx] += step

        # 4. Enforce clipping and publish command
        self.command_q = self.kinematics.clip_configuration(self.command_q)
        self._publish_command()

    def _publish_fk(self):
        now_msg = self.get_clock().now().to_msg()
        for topic, (pub, base, tip) in self.fk_pubs.items():
            try:
                pose = self.kinematics.relative_pose(base, tip, self.current_q)
                msg = PoseStamped()
                msg.header.stamp = now_msg
                msg.header.frame_id = base
                msg.pose.position.x = float(pose.translation[0])
                msg.pose.position.y = float(pose.translation[1])
                msg.pose.position.z = float(pose.translation[2])
                q = pin.Quaternion(pose.rotation)
                msg.pose.orientation.x = float(q.x)
                msg.pose.orientation.y = float(q.y)
                msg.pose.orientation.z = float(q.z)
                msg.pose.orientation.w = float(q.w)
                pub.publish(msg)
            except Exception as e:
                pass

    def _publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.free_joint_names)
        msg.position = [float(self.command_q[j.q_index]) for j in self.kinematics.joints]
        self.command_pub.publish(msg)


def pin_to_pose_msg(se3_pose: pin.SE3):
    from geometry_msgs.msg import Pose
    msg = Pose()
    msg.position.x = float(se3_pose.translation[0])
    msg.position.y = float(se3_pose.translation[1])
    msg.position.z = float(se3_pose.translation[2])
    q = pin.Quaternion(se3_pose.rotation)
    msg.orientation.x = float(q.x)
    msg.orientation.y = float(q.y)
    msg.orientation.z = float(q.z)
    msg.orientation.w = float(q.w)
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidMotionControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
