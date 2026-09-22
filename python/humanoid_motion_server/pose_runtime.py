"""Pose/teleop action runtime, independent of the web UI and robot hardware type."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import threading
import time

import yaml

from .poses import resolve_initial_pose, resolve_joint_jog, validate_initial_poses
from .pose_execution import execute_pose_with_teleop, receiver_control


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    options, ros_args = parser.parse_known_args(args)
    configuration = yaml.safe_load(Path(options.config).read_text())
    document = configuration['pose_document']
    document['initial_poses'] = validate_initial_poses(document['initial_poses'], document['resources'])
    home_id = configuration.get('home_pose_id', '')
    if home_id:
        resolve_initial_pose(document, home_id)
    motion = document['resources']['motion_params']['humanoid_motion_control']['ros__parameters']
    receiver_expected = configuration.get('receiver_present', False)
    identity = {key: configuration.get(key, '') for key in ('robot_id', 'revision')}
    stop_topic = configuration.get('stop_topic', '/teleop/emergency_stop')

    class PoseRuntime(Node):
        def __init__(self):
            super().__init__('humanoid_pose_runtime')
            self.seen = deque(maxlen=128)
            self.feedback_at = self.receiver_at = float('-inf')
            self.positions = {}
            self.enabled = False
            self.future = self.control_future = None
            self.blocked = False
            self.stop_before_ns = 0
            self.cancel_motion = threading.Event()
            self.worker = ThreadPoolExecutor(max_workers=2, thread_name_prefix='pose_runtime')
            self.results = self.create_publisher(String, '/motion/pose_results', 10)
            self.events = self.create_publisher(String, '/hc_teleop_recv/events', 10)
            self.status = self.create_publisher(String, '/motion/pose_status', 10)
            self.create_subscription(String, '/motion/pose_requests', self.web_request, 10)
            self.create_subscription(String, '/hc_teleop_recv/actions', self.teleop_request, 10)
            self.create_subscription(String, '/hc_teleop_recv/status', self.receiver, 10)
            self.create_subscription(JointState, motion['joint_state_endpoint'], self.feedback,
                                     qos_profile_sensor_data)
            self.create_subscription(Bool, stop_topic,
                                     lambda msg: self.stop() if msg.data else None, 10)
            self.create_timer(.02, self.tick)
            self.create_timer(.2, self.publish_status)
            self.get_logger().info('Pose runtime ready; no web process is required')

        def receiver(self, message):
            try:
                data = json.loads(message.data)
                current = data['configuration']
                if (current['sha256'] == configuration.get('configuration_sha256') and
                        current['robot_id'] == identity['robot_id']):
                    self.receiver_at = time.monotonic()
                    self.enabled = bool(data.get('enabled'))
            except (ValueError, KeyError, TypeError):
                pass

        def feedback(self, message):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if (not -0.01 <= age < .25 or len(message.name) != len(message.position) or
                    len(set(message.name)) != len(message.name) or
                    not all(math.isfinite(v) for v in message.position)):
                return
            self.positions = dict(zip(message.name, message.position))
            self.feedback_at = time.monotonic() - max(0., age)

        def fresh(self):
            now = time.monotonic()
            return now - self.feedback_at < .25 and (not receiver_expected or now - self.receiver_at < 1.)

        def stop(self):
            self.stop_before_ns = time.time_ns()
            self.cancel_motion.set()

        def publish_status(self):
            state = 'blocked' if self.blocked else 'running' if self.future or self.control_future else 'idle'
            payload = {**identity, 'state': state, 'ready': self.fresh(), 'home_pose_id': home_id,
                       'configuration_sha256': configuration.get('configuration_sha256', ''),
                       'stamp_ns': time.time_ns(),
                       'poses': [{'id': p['id'], 'name': p['name'], 'timeout_sec': p['timeout_sec']}
                                 for p in document['initial_poses']]}
            self.status.publish(String(data=json.dumps(payload)))

        def event(self, request, ok, message, **extra):
            payload = {**identity, 'id': request['id'], 'action': request['action'],
                       'kind': 'teleop_action', 'ok': ok, 'message': message, **extra}
            encoded = String(data=json.dumps(payload))
            self.results.publish(encoded)
            self.events.publish(encoded)
            self.get_logger().info(message)

        def teleop_request(self, message):
            self.request(message, headset=True)

        def web_request(self, message):
            self.request(message, headset=False)

        def request(self, message, headset):
            if len(message.data) > 8192:
                return
            try:
                request = json.loads(message.data)
                if not isinstance(request, dict):
                    return
                # Recording remains a separate data workflow; never claim its requests.
                if request.get('action') not in {'home', 'execute_pose', 'jog_joint', 'pause', 'resume', 'reset_reference'}:
                    return
                valid = (isinstance(request['id'], str) and len(request['id']) == 32 and
                         request['id'] not in self.seen and request['robot_id'] == identity['robot_id'] and
                         type(request['stamp_ns']) is int and abs(time.time_ns() - request['stamp_ns']) < 2e9)
                if headset:
                    valid = valid and receiver_expected and request.get('configuration_sha256') == configuration.get('configuration_sha256')
                else:
                    valid = valid and request.get('revision') == identity['revision']
            except (ValueError, KeyError, TypeError):
                return
            if not valid:
                return
            self.seen.append(request['id'])
            action = request['action']
            if headset and action in {'execute_pose', 'jog_joint'}:
                self.event(request, False, '手柄只允许执行已配置的回位姿态')
                return
            if action == 'pause':
                self.stop()
            elif self.future or self.control_future or self.blocked:
                self.event(request, False, '回位仍在执行或等待停止确认')
                return
            if action in {'pause', 'resume', 'reset_reference'}:
                if not receiver_expected:
                    self.event(request, action == 'pause', '遥操作接收器未启用')
                elif self.control_future:
                    self.event(request, False, '控制请求仍在执行')
                elif time.monotonic() - self.receiver_at >= 1.:
                    self.event(request, False, '遥操作状态已过期')
                else:
                    self.control_request = request
                    self.control_future = self.worker.submit(receiver_control, action, self.context.get_domain_id())
                return
            ident = home_id if action == 'home' else request.get('pose_id', '')
            if request['stamp_ns'] <= self.stop_before_ns:
                self.event(request, False, '回位请求已被后续暂停取消')
                return
            if action == 'home' and request.get('pose_id', ident) != ident:
                self.event(request, False, '回位姿态与运行配置不一致')
                return
            self.active_metadata = {}
            try:
                if action == 'jog_joint':
                    if self.enabled:
                        raise ValueError('请先停止遥操作使能，再执行点动')
                    jog = resolve_joint_jog(document, request.get('joint_name'), request.get('delta_rad'))
                    values = [self.positions[n] for n in jog['joint_names']]
                    index = jog['joint_names'].index(jog['joint_name'])
                    start = values[index]
                    values[index] += jog['delta_rad']
                    if not jog['lower_limits'][index] <= values[index] <= jog['upper_limits'][index]:
                        raise ValueError('点动目标超出关节限位')
                    self.active_metadata = {'joint_name': jog['joint_name'], 'delta_rad': jog['delta_rad'],
                                            'initial_position_rad': start, 'target_position_rad': values[index]}
                    pose = {'id': 'joint_jog', 'name': '关节点动', 'goals': [{**jog, 'positions_rad': values}]}
                else:
                    pose = resolve_initial_pose(document, ident)
            except (ValueError, KeyError) as error:
                self.event(request, False, str(error))
                return
            if not self.fresh() or any(n not in self.positions for g in pose['goals'] for n in g['joint_names']):
                self.event(request, False, '关节反馈或遥操作状态未就绪')
                return
            self.active_request, self.active_pose = request, pose
            self.cancel_motion.clear()
            self.future = self.worker.submit(
                execute_pose_with_teleop, pose['goals'], self.context.get_domain_id(), self.cancel_motion,
                receiver_present=receiver_expected, resume_after=self.enabled,
                servo_lease_ms=motion.get('servo_lease_ms', 100), stop_topic=stop_topic)

        def tick(self):
            if self.control_future and self.control_future.done():
                try:
                    self.event(self.control_request, True, self.control_future.result())
                except Exception as error:
                    self.event(self.control_request, False, str(error))
                self.control_future = None
            if self.future is None:
                return
            if not self.fresh():
                self.cancel_motion.set()
            if self.future.done():
                try:
                    results = self.future.result()
                    message = '点动已完成' if self.active_request['action'] == 'jog_joint' else '已到达初始姿态'
                    self.event(self.active_request, True, message, results=results,
                               pose_id=self.active_pose['id'], pose_name=self.active_pose['name'],
                               **self.active_metadata)
                except Exception as error:
                    self.blocked = bool(getattr(error, 'motion_may_be_active', False))
                    self.event(self.active_request, False, str(error), motion_may_be_active=self.blocked)
                self.future = None

        def close(self):
            self.cancel_motion.set()
            self.worker.shutdown(wait=True)

    rclpy.init(args=ros_args)
    node = PoseRuntime()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
