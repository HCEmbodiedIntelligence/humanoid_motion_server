"""Standalone runtime against mock ROS peers; no manager, vendor driver or robot."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from humanoid_motion_interfaces.action import MoveJ
from humanoid_motion_interfaces.msg import Status
from humanoid_motion_server.pose_client import request_pose_action
from humanoid_motion_server.pose_execution import MotionCommandError


def test_standalone_runtime_owns_named_pose_and_cancellation(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    transport = tmp_path / 'dds.xml'
    transport.write_text('''<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <transport_descriptors><transport_descriptor><transport_id>loopback_udp</transport_id>
        <type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
      </transport_descriptor></transport_descriptors>
      <participant profile_name="loopback" is_default_profile="true"><rtps>
        <userTransports><transport_id>loopback_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps></participant></profiles>''')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', str(transport))
    domain = 224
    motion = {'joint_state_endpoint': '/pose_test/feedback', 'servo_lease_ms': 1,
              'groups.right': ['r1', 'r2'], 'group_lower_limits.right': [-2., -2.],
              'group_upper_limits.right': [2., 2.]}
    pose = dict(id='right_home', name='Right only', velocity_scale=.12,
                acceleration_scale=.23, jerk_scale=.34, timeout_sec=5.,
                targets=[dict(channel='right_move', positions_rad=[.25, -.5])])
    cfg = dict(robot_id='test_robot', revision='test_revision', receiver_present=True,
               configuration_sha256='test_hash', home_pose_id='right_home',
               pose_document={'initial_poses': [pose], 'resources': {
                   'motion_params': {'humanoid_motion_control': {'ros__parameters': motion}},
                   'channel_config': {'channels': [dict(name='right_move', kind='move_j',
                                                        group='right', endpoint='/pose_test/right')]}}})
    path = tmp_path / 'runtime.yaml'
    path.write_text(yaml.safe_dump(cfg))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = rclpy.create_node('pose_runtime_test_peers', context=context)
    executor = MultiThreadedExecutor(context=context, num_threads=4)
    executor.add_node(node)
    received, controls, statuses = [], [], []
    mode = {'value': 'success', 'enabled': True, 'inhibited': False}
    started, stopped = threading.Event(), threading.Event()
    feedback = node.create_publisher(JointState, '/pose_test/feedback', 10)
    receiver = node.create_publisher(String, '/hc_teleop_recv/status', 10)

    def publish():
        feedback.publish(JointState(header=__import__('std_msgs.msg', fromlist=['Header']).Header(
            stamp=node.get_clock().now().to_msg()), name=['l1', 'r1', 'r2'], position=[.7, 0., 0.]))
        receiver.publish(String(data=json.dumps({'enabled': mode['enabled'],
            'configuration': {'robot_id': 'test_robot', 'sha256': 'test_hash'}})))
    timer = node.create_timer(.02, publish)
    subscription = node.create_subscription(String, '/motion/pose_status',
                                            lambda m: statuses.append(json.loads(m.data)), 10)

    def own(req, res):
        controls.append('acquire' if req.data else 'release')
        res.success = not (req.data and mode['inhibited'])
        if res.success:
            mode['inhibited'], mode['enabled'] = req.data, False
        return res

    def enable(req, res):
        controls.append('resume' if req.data else 'pause')
        res.success = not (req.data and mode['inhibited'])
        if res.success:
            mode['enabled'] = req.data
        return res

    services = [node.create_service(SetBool, '/hc_teleop_recv/set_motion_active', own),
                node.create_service(SetBool, '/hc_teleop_recv/set_enabled', enable)]

    def execute(handle):
        received.append(handle.request)
        started.set()
        result = MoveJ.Result()
        if mode['value'] == 'fail':
            handle.abort()
            result.status.code = Status.SDK_ERROR
        elif mode['value'] == 'cancel':
            until = time.monotonic() + 6
            while not handle.is_cancel_requested and time.monotonic() < until:
                time.sleep(.01)
            assert handle.is_cancel_requested
            handle.canceled()
            result.status.code = Status.CANCELED
        else:
            handle.succeed()
            result.status.code = Status.OK
            result.final_joint_state = handle.request.target
        stopped.set()
        return result

    server = ActionServer(node, MoveJ, '/pose_test/right', execute_callback=execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log = (tmp_path / 'runtime.log').open('w')
    process = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parents[1] / 'scripts/pose_runtime'),
                                '--config', str(path)], stdout=log, stderr=subprocess.STDOUT,
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
             'ROS_LOG_DIR': str(tmp_path / 'logs')})

    def request(action, pose_id=None, **kwargs):
        return request_pose_action(action, 'test_robot', 'test_revision', domain, pose_id, **kwargs)

    def wait(predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, (tmp_path / 'runtime.log').read_text()
            assert time.monotonic() < end, (tmp_path / 'runtime.log').read_text()
            time.sleep(.02)

    try:
        wait(lambda: statuses and statuses[-1]['ready'])
        assert request('home')['pose_id'] == 'right_home'
        assert controls == ['acquire', 'release', 'resume']
        goal = received[-1]
        assert goal.group_name == 'right' and goal.target.name == ['r1', 'r2']
        assert list(goal.target.position) == [.25, -.5]
        assert (goal.options.velocity_scale, goal.options.acceleration_scale,
                goal.options.jerk_scale, goal.options.timeout_sec) == (.12, .23, .34, 5.)
        before = len(received)
        with pytest.raises(MotionCommandError, match='姿态与运行配置不一致'):
            request('home', 'other_home')
        assert len(received) == before

        mode['value'] = 'fail'; controls.clear()
        with pytest.raises(MotionCommandError):
            request('home')
        assert controls == ['acquire', 'release'] and not mode['enabled']
        request('resume')
        wait(lambda: mode['enabled'])
        time.sleep(.1)  # Publish the new enable state before requesting another pose.
        mode['value'] = 'cancel'; controls.clear(); started.clear(); stopped.clear()
        with ThreadPoolExecutor(max_workers=1) as pool:
            running = pool.submit(request, 'home')
            assert started.wait(5), (tmp_path / 'runtime.log').read_text()
            with pytest.raises(MotionCommandError, match='仍在执行'):
                request('resume')
            with pytest.raises(MotionCommandError, match='仍在执行'):
                request('jog_joint', joint_name='r1', delta_rad=.1)
            assert request('pause')['ok']
            with pytest.raises(MotionCommandError):
                running.result(timeout=8)
        assert stopped.is_set() and not mode['enabled'] and not mode['inhibited']
        assert 'resume' not in controls

        # A request queued by a UI before a later pause cannot start afterward.
        stamp = time.time_ns()
        request('pause')
        mode['value'] = 'success'; before = len(received)
        with pytest.raises(MotionCommandError, match='后续暂停取消'):
            request('home', request_stamp_ns=stamp)
        assert len(received) == before
        result = request('jog_joint', joint_name='r1', delta_rad=.1)
        assert result['initial_position_rad'] == 0. and result['target_position_rad'] == .1
        assert received[-1].target.name == ['r1', 'r2']
        assert list(received[-1].target.position) == [.1, 0.]
        with pytest.raises(MotionCommandError, match='步长'):
            request('jog_joint', joint_name='r1', delta_rad=.3)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5)
        executor.shutdown(timeout_sec=8)
        thread.join(timeout=8)
        server.destroy()
        node.destroy_node(); context.shutdown(); log.close()
