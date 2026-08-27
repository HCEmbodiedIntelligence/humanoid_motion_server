#!/usr/bin/env python3
"""End-to-end verifier for the installed mock bringup stack."""

import copy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from humanoid_motion_interfaces.action import MoveJ, MoveL, MoveP
from humanoid_motion_interfaces.msg import Status
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class VerificationError(RuntimeError):
    pass


class Verifier(Node):
    def __init__(self):
        super().__init__('verify_humanoid_mock_stack')
        self._lock = threading.Lock()
        self._state = None
        self._state_sequence = 0
        self._commands = []
        self._watchdog_stops = 0
        self._checks = []
        self.create_subscription(
            JointState, '/hc_teleop/joint_states', self._state_cb, 10)
        self.create_subscription(
            JointState, '/hc_teleop/joint_cmd', self._command_cb, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostic_cb, 10)

        self._runtime_parameters = self.create_client(
            SetParameters, '/humanoid_motion_control/set_parameters')
        self._move_j = ActionClient(self, MoveJ, '/normal/move_j')
        self._left_move_j = ActionClient(self, MoveJ, '/normal/left_arm/move_j')
        self._right_move_j = ActionClient(self, MoveJ, '/normal/right_arm/move_j')
        self._imitation_move_j = ActionClient(self, MoveJ, '/imitation/move_j')
        self._move_l = ActionClient(self, MoveL, '/normal/move_l')
        self._move_p = ActionClient(self, MoveP, '/imitation/move_p')
        self._low_servo_j = self.create_publisher(
            JointState, '/imitation/servo_j', qos_profile_sensor_data)
        self._servo_j = self.create_publisher(
            JointState, '/teleop/servo_j', qos_profile_sensor_data)
        self._servo_p = self.create_publisher(
            PoseStamped, '/teleop/servo_p', qos_profile_sensor_data)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _pass(self, description):
        self._checks.append(description)
        self.get_logger().info(f'PASS [{len(self._checks):02d}]: {description}')

    def _state_cb(self, message):
        with self._lock:
            self._state = copy.deepcopy(message)
            self._state_sequence += 1

    def _command_cb(self, message):
        with self._lock:
            self._commands.append((time.monotonic(), copy.deepcopy(message)))
            if len(self._commands) > 4000:
                del self._commands[:2000]

    def _diagnostic_cb(self, message):
        count = None
        for status in message.status:
            for value in status.values:
                if value.key == 'watchdog_stops':
                    count = int(value.value)
        if count is not None:
            with self._lock:
                self._watchdog_stops = max(self._watchdog_stops, count)

    def _state_copy(self):
        with self._lock:
            return copy.deepcopy(self._state)

    def _state_counter(self):
        with self._lock:
            return self._state_sequence

    def _watchdog_count(self):
        with self._lock:
            return self._watchdog_stops

    def _command_copy(self):
        with self._lock:
            return copy.deepcopy(self._commands)

    @staticmethod
    def _wait_future(future, timeout, description):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            raise VerificationError(f'{description} timed out')
        return future.result()

    @staticmethod
    def _require_ok(status, description):
        if status.code != Status.OK:
            raise VerificationError(
                f'{description} failed: code={status.code} message={status.message}')

    @staticmethod
    def _wait_until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.01)
        raise VerificationError(f'{description} timed out')

    def _wait_ready(self):
        self._wait_until(
            self._state_copy, 15.0, 'real /hc_teleop/joint_states feedback')
        if not self._runtime_parameters.wait_for_service(timeout_sec=10.0):
            raise VerificationError('unified runtime parameter services unavailable')
        for client, name in (
                (self._move_j, 'whole-body MoveJ'),
                (self._left_move_j, 'left MoveJ'),
                (self._right_move_j, 'right MoveJ'),
                (self._imitation_move_j, 'imitation MoveJ'),
                (self._move_l, 'MoveL'), (self._move_p, 'MoveP')):
            if not client.wait_for_server(timeout_sec=10.0):
                raise VerificationError(f'{name} action unavailable')
        self._pass('real 100 Hz driver feedback reached the public JointState boundary')

    def _lookup(self, base, tip):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                return self._tf_buffer.lookup_transform(base, tip, Time())
            except TransformException:
                time.sleep(0.05)
        raise VerificationError(f'TF {base} -> {tip} unavailable')

    def _current_pose(self, base='base_link', tip='left_tool0'):
        transform = self._lookup(base, tip)
        pose = PoseStamped()
        pose.header = copy.deepcopy(transform.header)
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
        return pose

    def _send_goal(self, client, goal, name, feedback=None):
        goal_handle = self._wait_future(
            client.send_goal_async(goal, feedback_callback=feedback),
            5.0, f'{name} acceptance')
        if not goal_handle.accepted:
            raise VerificationError(f'{name} goal rejected')
        return goal_handle

    def _result(self, goal_handle, timeout, name):
        return self._wait_future(goal_handle.get_result_async(), timeout, f'{name} result')

    def _expect_success(self, goal_handle, timeout, name):
        wrapped = self._result(goal_handle, timeout, name)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise VerificationError(f'{name} terminal action status was {wrapped.status}')
        self._require_ok(wrapped.result.status, name)
        return wrapped

    def _expect_abort(self, goal_handle, code, timeout, name):
        wrapped = self._result(goal_handle, timeout, name)
        if wrapped.status != GoalStatus.STATUS_ABORTED:
            raise VerificationError(f'{name} did not abort: status={wrapped.status}')
        if wrapped.result.status.code != code:
            raise VerificationError(
                f'{name} returned code={wrapped.result.status.code}, expected={code}')
        return wrapped

    def _move_j_goal(self, group, names, positions, timeout=5.0):
        goal = MoveJ.Goal()
        goal.group_name = group
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.name = list(names)
        goal.target.position = list(positions)
        goal.options.timeout_sec = timeout
        return goal

    def _joint_position(self, name):
        state = self._state_copy()
        return state.position[state.name.index(name)]

    def _wait_command(self, name, value=None, after=0.0, timeout=3.0, tolerance=1.0e-6):
        def matching():
            for received, message in self._command_copy():
                if received < after or name not in message.name:
                    continue
                index = message.name.index(name)
                if value is None or abs(message.position[index] - value) <= tolerance:
                    return received, message
            return None
        return self._wait_until(matching, timeout, f'command for {name}')

    def _publish_joint_servo(self, publisher, name, value):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [name]
        message.position = [value]
        publisher.publish(message)

    def _pause_feedback(self, paused):
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='test_pause_driver_feedback',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=paused))]
        response = self._wait_future(
            self._runtime_parameters.call_async(request),
            5.0, 'feedback test-control parameter')
        if not response.results or not response.results[0].successful:
            raise VerificationError('feedback test-control parameter was rejected')

    def _verify_five_interfaces(self):
        feedback = []
        current = self._joint_position('left_shoulder_pitch')
        handle = self._send_goal(
            self._move_j,
            self._move_j_goal('whole_body', ['left_shoulder_pitch'], [current + 0.10]),
            'MoveJ', lambda message: feedback.append(message))
        self._expect_success(handle, 8.0, 'MoveJ')
        if not feedback:
            raise VerificationError('MoveJ produced no real-feedback progress')
        self._pass('MoveJ action completed from real feedback')

        pose = self._current_pose()
        pose.header.stamp = self.get_clock().now().to_msg()
        move_l = MoveL.Goal()
        move_l.group_name = 'left_arm'
        move_l.tip_frame = 'left_tool0'
        move_l.target_pose = pose
        move_l.options.timeout_sec = 5.0
        self._expect_success(
            self._send_goal(self._move_l, move_l, 'MoveL'), 8.0, 'MoveL')
        self._pass('MoveL action completed from JointState-derived FK feedback')

        pose.header.stamp = self.get_clock().now().to_msg()
        move_p = MoveP.Goal()
        move_p.group_name = 'left_arm'
        move_p.tip_frame = 'left_tool0'
        move_p.target_pose = pose
        move_p.options.timeout_sec = 5.0
        self._expect_success(
            self._send_goal(self._move_p, move_p, 'MoveP'), 8.0, 'MoveP')
        self._pass('MoveP action completed from JointState-derived FK feedback')

        start = time.monotonic()
        for _ in range(100):
            self._publish_joint_servo(
                self._servo_j, 'right_shoulder_pitch', 0.20)
            time.sleep(0.01)
        self._wait_command('right_shoulder_pitch', after=start)
        samples = [
            received for received, message in self._command_copy()
            if received >= start and 'right_shoulder_pitch' in message.name
        ]
        if len(samples) < 20:
            raise VerificationError(f'only {len(samples)} ServoJ commands were observed')
        rate = (len(samples) - 1) / (samples[-1] - samples[0])
        if rate < 80.0 or rate > 120.0:
            raise VerificationError(f'JointState command output rate was {rate:.1f} Hz')
        if self._joint_position('right_shoulder_pitch') < 0.10:
            raise VerificationError('ServoJ did not move measured feedback toward its target')
        self._pass(
            f'ServoJ passed final RTC and moved measured feedback at {rate:.1f} Hz')

        servo_p = self._current_pose()
        start = time.monotonic()
        for _ in range(20):
            servo_p.header.stamp = self.get_clock().now().to_msg()
            self._servo_p.publish(servo_p)
            time.sleep(0.01)
        self._wait_command('left_shoulder_pitch', after=start)
        self._pass('ServoP reached the final-RTC JointState command boundary')
        time.sleep(0.20)

    def _verify_move_terminals(self):
        current = self._joint_position('left_shoulder_pitch')
        first = self._send_goal(
            self._move_j,
            self._move_j_goal('whole_body', ['left_shoulder_pitch'], [current + 0.5]),
            'preempted MoveJ')
        time.sleep(0.02)
        second = self._send_goal(
            self._move_j,
            self._move_j_goal('whole_body', ['left_shoulder_pitch'], [current - 0.05]),
            'preempting MoveJ')
        self._expect_abort(first, Status.PREEMPTED, 5.0, 'preempted MoveJ')
        self._expect_success(second, 8.0, 'preempting MoveJ')
        self._pass('same-priority latest Move preempts with structured PREEMPTED status')

        cancel = self._send_goal(
            self._move_j,
            self._move_j_goal('whole_body', ['left_shoulder_pitch'], [current]),
            'cancel MoveJ')
        cancel_response = self._wait_future(
            cancel.cancel_goal_async(), 5.0, 'MoveJ cancel response')
        if not cancel_response.goals_canceling:
            raise VerificationError('MoveJ cancel was not accepted')
        canceled = self._result(cancel, 5.0, 'canceled MoveJ')
        if canceled.status != GoalStatus.STATUS_CANCELED or \
                canceled.result.status.code != Status.CANCELED:
            raise VerificationError('MoveJ cancel lacked CANCELED terminal semantics')
        self._pass('Action cancel propagates CANCELED terminal semantics')

        timeout = self._send_goal(
            self._move_j,
            self._move_j_goal(
                'whole_body', ['left_shoulder_pitch'], [current], timeout=0.02),
            'timeout MoveJ')
        self._expect_abort(timeout, Status.TIMEOUT, 5.0, 'timeout MoveJ')
        self._pass('Move timeout aborts with structured TIMEOUT status')
        # The public timeout sends an asynchronous backend cancel. Let the
        # mock action consume it before starting the next arbitration case.
        time.sleep(0.20)

    def _verify_preemption_and_no_resume(self):
        start_position = self._joint_position('left_shoulder_pitch')
        old_target = min(start_position + 1.0, 2.0)
        imitation = self._send_goal(
            self._imitation_move_j,
            self._move_j_goal(
                'left_arm', ['left_shoulder_pitch'], [old_target], timeout=5.0),
            'imitation MoveJ')
        time.sleep(0.025)
        hold_position = self._joint_position('left_shoulder_pitch')
        pose = self._current_pose()
        for _ in range(5):
            pose.header.stamp = self.get_clock().now().to_msg()
            self._servo_p.publish(pose)
            time.sleep(0.02)
        self._expect_abort(imitation, Status.PREEMPTED, 5.0, 'imitation MoveJ')
        self._pass('teleop ServoP preempts an overlapping imitation Move')

        time.sleep(0.30)
        after_expiry = self._joint_position('left_shoulder_pitch')
        if abs(after_expiry - hold_position) > 0.15 or abs(after_expiry - old_target) < 0.30:
            raise VerificationError('preempted imitation Move resumed after teleop expiry')
        self._pass('preempted Move remains terminal and does not resume')

    def _verify_servo_lease_and_fallback(self):
        stop = threading.Event()

        def publish_low():
            while not stop.is_set():
                self._publish_joint_servo(
                    self._low_servo_j, 'right_shoulder_pitch', -0.25)
                time.sleep(0.025)

        thread = threading.Thread(target=publish_low, daemon=True)
        thread.start()
        try:
            self._wait_until(
                lambda: self._joint_position('right_shoulder_pitch') < -0.05,
                3.0, 'low-priority Servo movement')
            low_position = self._joint_position('right_shoulder_pitch')
            high_started = time.monotonic()
            for _ in range(20):
                self._publish_joint_servo(
                    self._servo_j, 'right_shoulder_pitch', 0.35)
                time.sleep(0.02)
            self._wait_command('right_shoulder_pitch', after=high_started)
            self._wait_until(
                lambda: self._joint_position('right_shoulder_pitch') > low_position + 0.03,
                3.0, 'high-priority Servo takeover')
            high_position = self._joint_position('right_shoulder_pitch')
            high_stopped = time.monotonic()
            time.sleep(0.08)
            if self._joint_position('right_shoulder_pitch') < high_position - 0.02:
                raise VerificationError('teleop lease expired before 100 ms')
            self._wait_until(
                lambda: self._joint_position('right_shoulder_pitch') < high_position - 0.03,
                3.0, 'low-priority Servo fallback')
            if time.monotonic() - high_stopped < 0.09:
                raise VerificationError('teleop fallback occurred before lease expiry')
        finally:
            stop.set()
            thread.join(timeout=1.0)
        self._pass('100 ms teleop lease expires and fresh low-priority Servo falls back')
        time.sleep(0.20)

    def _verify_group_arbitration(self):
        left = self._joint_position('left_shoulder_pitch')
        right = self._joint_position('right_shoulder_pitch')
        start = time.monotonic()
        left_goal = self._send_goal(
            self._left_move_j,
            self._move_j_goal('left_arm', ['left_shoulder_pitch'], [left + 0.15]),
            'left-arm MoveJ')
        right_goal = self._send_goal(
            self._right_move_j,
            self._move_j_goal('right_arm', ['right_shoulder_pitch'], [right - 0.15]),
            'right-arm MoveJ')

        self._wait_command('left_shoulder_pitch', after=start)
        self._wait_command('right_shoulder_pitch', after=start)
        self._expect_success(left_goal, 8.0, 'left-arm MoveJ')
        self._expect_success(right_goal, 8.0, 'right-arm MoveJ')
        self._pass('non-overlapping left/right arms execute concurrently across the runtime boundary')

        left = self._joint_position('left_shoulder_pitch')
        right = self._joint_position('right_shoulder_pitch')
        left_goal = self._send_goal(
            self._left_move_j,
            self._move_j_goal('left_arm', ['left_shoulder_pitch'], [left + 0.8]),
            'left-arm conflict MoveJ')
        right_goal = self._send_goal(
            self._right_move_j,
            self._move_j_goal('right_arm', ['right_shoulder_pitch'], [right + 0.8]),
            'right-arm conflict MoveJ')
        time.sleep(0.02)
        torso = self._joint_position('torso_yaw')
        whole = self._send_goal(
            self._move_j,
            self._move_j_goal('whole_body', ['torso_yaw'], [torso + 0.05]),
            'whole-body MoveJ')
        self._expect_abort(left_goal, Status.PREEMPTED, 5.0, 'left whole-body conflict')
        self._expect_abort(right_goal, Status.PREEMPTED, 5.0, 'right whole-body conflict')
        self._expect_success(whole, 8.0, 'whole-body MoveJ')
        self._pass('whole_body conflicts with and preempts both arm claims')

    def _verify_stale_feedback(self):
        sequence = self._state_counter()
        current = self._joint_position('left_shoulder_pitch')
        stale = self._send_goal(
            self._move_j,
            self._move_j_goal(
                'whole_body', ['left_shoulder_pitch'], [current + 1.0], timeout=3.0),
            'stale-feedback MoveJ')
        time.sleep(0.02)
        self._pause_feedback(True)
        try:
            self._expect_abort(stale, Status.STATE_STALE, 3.0, 'stale-feedback MoveJ')
            stopped_sequence = self._state_counter()
            time.sleep(0.15)
            if self._state_counter() <= stopped_sequence:
                raise VerificationError(
                    'independent driver JointState stopped with the motion feedback consumer')
            self._pass(
                'stale motion feedback aborts with STATE_STALE while the driver topic stays live')
        finally:
            self._pause_feedback(False)
        self._wait_until(
            lambda: self._state_counter() > sequence, 3.0, 'JointState feedback recovery')

    def _verify_invalid_inputs(self):
        cases = [
            (['unknown_joint'], [0.0]),
            (['left_shoulder_pitch', 'left_shoulder_pitch'], [0.0, 0.1]),
            (['left_shoulder_pitch'], [math.nan]),
            (['left_shoulder_pitch'], [math.inf]),
            (['left_shoulder_pitch', 'left_elbow'], [0.0]),
        ]
        for index, (names, positions) in enumerate(cases):
            goal = self._move_j_goal('left_arm', names, positions)
            handle = self._send_goal(self._left_move_j, goal, f'invalid MoveJ {index}')
            self._expect_abort(
                handle, Status.INVALID_REQUEST, 3.0, f'invalid MoveJ {index}')

        self._pass('ROS boundary rejects unknown, duplicate, NaN, Inf, and length errors')

    def _verify_driver_watchdog(self):
        baseline = self._watchdog_count()
        start = time.monotonic()
        for _ in range(8):
            self._publish_joint_servo(
                self._servo_j, 'right_shoulder_pitch', 0.10)
            time.sleep(0.025)
        self._wait_command('right_shoulder_pitch', after=start)
        self._wait_until(
            lambda: self._watchdog_count() > baseline, 3.0,
            'driver command watchdog increment')
        self._pass('driver command watchdog stops the plugin after command expiry')

    def verify(self):
        self._wait_ready()
        def visible_graph_nodes():
            return {
                f'{namespace.rstrip("/")}/{name}' if namespace != '/' else f'/{name}'
                for name, namespace in self.get_node_names_and_namespaces()
                if not name.startswith('_')
            }
        graph_nodes = self._wait_until(
            lambda: visible_graph_nodes()
            if '/humanoid_motion_control' in visible_graph_nodes() else None,
            3.0, 'control node graph discovery')
        expected_nodes = {
            '/humanoid_driver_runtime',
            '/humanoid_motion_control',
            '/verify_humanoid_mock_stack',
        }
        if graph_nodes != expected_nodes:
            raise VerificationError(f'unexpected ROS nodes during verification: {graph_nodes}')
        self._pass(
            'the graph contains separate motion and driver runtime nodes plus this verifier')
        service_names = {
            name for name, _ in self.get_service_names_and_types()
        }
        unexpected_kinematics = {
            '/kinematics/fk', '/kinematics/ik'
        } & service_names
        if unexpected_kinematics:
            raise VerificationError(
                f'public kinematics services must be absent: {unexpected_kinematics}')
        self._pass('FK/IK remain internal and are not exposed as ROS services')
        publishers = self.get_publishers_info_by_topic('/hc_teleop/joint_cmd')
        if len(publishers) != 1:
            raise VerificationError(
                f'/hc_teleop/joint_cmd must have one publisher, found {len(publishers)}')
        self._pass('/hc_teleop/joint_cmd has one authoritative publisher')
        self._verify_five_interfaces()
        self._verify_move_terminals()
        self._verify_preemption_and_no_resume()
        self._verify_servo_lease_and_fallback()
        self._verify_group_arbitration()
        self._verify_stale_feedback()
        self._verify_driver_watchdog()
        self._verify_invalid_inputs()
        self.get_logger().info(
            f'PASS: all {len(self._checks)} mock integration checks completed')


def main(args=None):
    rclpy.init(args=args)
    node = Verifier()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    exit_code = 0
    try:
        node.verify()
    except Exception as error:  # noqa: BLE001 - report every integration failure.
        node.get_logger().error(f'FAIL: {error}')
        exit_code = 1
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
