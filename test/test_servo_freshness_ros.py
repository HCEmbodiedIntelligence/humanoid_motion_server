"""Real node/SDK test with repository fixtures and synthetic feedback only."""

import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import yaml


def run(node_binary):
    # Never join the robot's ROS domain or start a driver. Unique topic names
    # also isolate this test from other tests sharing the localhost domain.
    os.environ['ROS_DOMAIN_ID'] = '229'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    root = Path(tempfile.mkdtemp(prefix='servo-freshness-test-'))
    os.environ['ROS_LOG_DIR'] = str(root / 'roslogs')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from rclpy.qos import HistoryPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.action import ActionClient
    from humanoid_motion_interfaces.action import MoveJ
    from humanoid_motion_interfaces.msg import Status
    from action_msgs.msg import GoalStatus

    source = Path(__file__).resolve().parents[1]
    prefix = '/latest_' + uuid.uuid4().hex[:12]
    params = yaml.safe_load((source / 'config/motion_control.yaml').read_text())[
        'humanoid_motion_control']['ros__parameters']
    params.update({
        'joint_state_endpoint': prefix + '/feedback',
        'joint_command_endpoint': prefix + '/command',
        'sdk_config_file': str(source / 'config/robo_manip.test_humanoid.yaml'),
        'urdf_file': str(source / 'urdf/test_humanoid.urdf'),
        'tool_config_file': str(source / 'config/tools.yaml'),
        'channel_config_file': str(root / 'channels.yaml'),
    })
    channels = []
    for side in ('left', 'right'):
        channels.extend([
            dict(name=side + '_servo', kind='servo_j', group=side + '_arm',
                 priority=100, endpoint=prefix + '/' + side),
            dict(name=side + '_fk', kind='servo_p', group=side + '_arm', priority=100,
                 endpoint=prefix + '/' + side + '_pose', base_frame='base_link',
                 tip_frame=side + '_tool0', fk_pose_topic=prefix + '/' + side + '_fk'),
        ])
    channels.append(dict(name='left_move', kind='move_j', group='left_arm',
                         priority=100, endpoint=prefix + '/move'))
    (root / 'channels.yaml').write_text(yaml.safe_dump({'channels': channels}))
    (root / 'params.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))

    rclpy.init()
    observer = rclpy.create_node('servo_freshness_observer_' + uuid.uuid4().hex[:8])
    move_client = ActionClient(observer, MoveJ, prefix + '/move')
    # Match the driver's reliable writer with a deeper history. The receiver
    # must consume newest feedback continuously without waiting for old samples.
    feedback = observer.create_publisher(JointState, prefix + '/feedback', 10)
    targets = {s: observer.create_publisher(JointState, prefix + '/' + s, 1)
               for s in ('left', 'right')}
    positions = dict(torso_yaw=0., neck_yaw=0., left_shoulder_pitch=0., left_elbow=-.4,
                     right_shoulder_pitch=0., right_elbow=-.4)
    commands = {'left': [], 'right': []}
    fk_samples = []
    feedback_stamps = set()
    last_feedback = None
    last_targets = {}

    def command(message):
        side = 'left' if message.name[0].startswith('left') else 'right'
        commands[side].append((time.monotonic(), list(message.position)))
        positions.update(zip(message.name, message.position))

    def fk(message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        fk_samples.append((time.monotonic(), stamp))

    subscriptions = [observer.create_subscription(
        JointState, prefix + '/command', command, qos_profile_sensor_data)]
    subscriptions.extend(observer.create_subscription(
        PoseStamped, prefix + '/' + s + '_fk', fk, qos_profile_sensor_data)
        for s in ('left', 'right'))

    def publish_targets(value=.25, sides=('left', 'right'), replay=False):
        for side in sides:
            if replay:
                # Altering the pose without advancing its stamp must not replace
                # the accepted target or extend that target's lease.
                message = copy.deepcopy(last_targets[side])
                message.position[0] = -.25
            else:
                message = JointState()
                message.header.stamp = observer.get_clock().now().to_msg()
                message.name = [side + '_shoulder_pitch', side + '_elbow']
                message.position = [value, -.4]
                last_targets[side] = copy.deepcopy(message)
            targets[side].publish(message)

    proc = None

    def pump(duration, send=None, feedback_mode='fresh'):
        nonlocal last_feedback
        until = time.monotonic() + duration
        next_sample = 0.
        while time.monotonic() < until:
            if proc.poll() is not None:
                raise AssertionError('motion node exited:\n' + (root / 'node.log').read_text())
            now = time.monotonic()
            if now >= next_sample:
                next_sample = now + .01
                if feedback_mode == 'frozen':
                    message = copy.deepcopy(last_feedback)
                else:
                    message = JointState()
                    message.header.stamp = observer.get_clock().now().to_msg()
                    if feedback_mode == 'old':
                        message.header.stamp.sec -= 1
                    message.name = list(positions)
                    message.position = list(positions.values())
                    message.velocity = [0.] * len(positions)
                    if feedback_mode == 'fresh':
                        last_feedback = copy.deepcopy(message)
                stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                feedback_stamps.add(stamp)
                feedback.publish(message)
                if send:
                    send()
            rclpy.spin_once(observer, timeout_sec=.001)

    report = {'log_directory': str(root)}
    try:
        with (root / 'node.log').open('w') as log:
            proc = subprocess.Popen([str(node_binary), '--ros-args', '--params-file',
                                     str(root / 'params.yaml')], stdout=log, stderr=log)
        pump(2.)
        assert fk_samples, 'no FK feedback'
        for topic in (prefix + '/feedback', prefix + '/left', prefix + '/right'):
            info = observer.get_subscriptions_info_by_topic(topic)
            assert info, topic
            # Fast DDS on Humble does not advertise history/depth in discovery.
            # Check it when supplied; freshness is tested behaviorally below.
            for item in info:
                assert item.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT, (topic, str(item))
                if item.qos_profile.history != HistoryPolicy.UNKNOWN:
                    assert item.qos_profile.depth == 1, (topic, str(item))

        pump(.7, publish_targets)
        report['continuous_commands'] = {s: len(v) for s, v in commands.items()}
        assert all(len(v) >= 30 for v in commands.values()), report
        report['max_continuous_gap_ms'] = {
            s: max(b[0] - a[0] for a, b in zip(v, v[1:])) * 1000
            for s, v in commands.items()}
        assert all(v < 100 for v in report['max_continuous_gap_ms'].values()), report

        stopped_at = time.monotonic()
        pump(.35, lambda: publish_targets(replay=True))
        assert all(not any(t > stopped_at + .15 for t, _ in v)
                   for v in commands.values()), 'replayed targets renewed Servo lease'

        before = {s: len(v) for s, v in commands.items()}
        pump(.5, lambda: publish_targets(-.2))
        assert all(len(commands[s]) > before[s] + 20 for s in commands), 'fresh targets did not resume'

        frozen_at = time.monotonic()
        pump(.35, publish_targets, 'frozen')
        assert all(not any(t > frozen_at + .15 for t, _ in v)
                   for v in commands.values()), 'frozen feedback allowed continued commands'
        assert not any(t > frozen_at + .15 for t, _ in fk_samples), 'frozen feedback renewed FK'
        before = {s: len(v) for s, v in commands.items()}
        pump(.2, publish_targets, 'old')
        assert all(len(commands[s]) == before[s] for s in commands), 'stale feedback accepted'

        pump(.5, publish_targets)
        assert all(len(commands[s]) > before[s] + 20 for s in commands), 'fresh feedback did not recover'
        right_stopped = time.monotonic()
        left_before = len(commands['left'])
        pump(.35, lambda: publish_targets(sides=('left',)))
        assert not any(t > right_stopped + .15 for t, _ in commands['right']), 'right arm did not expire'
        assert len(commands['left']) > left_before + 20, 'right expiry stopped the left arm'
        assert all(stamp in feedback_stamps for _, stamp in fk_samples), 'FK changed source timestamp'

        # Owned Move workers must deliver timeout/cancel results, release their
        # wait registrations, and finish when shutdown occurs during a Move.
        pump(.2)
        assert move_client.wait_for_server(timeout_sec=1.)

        def wait_future(future):
            until = time.monotonic() + 2.
            while not future.done() and time.monotonic() < until:
                pump(.01)
            assert future.done(), 'Move worker failed to deliver a result'
            return future.result()

        def start_move(timeout):
            goal = MoveJ.Goal()
            goal.group_name = 'left_arm'
            goal.target.name = ['left_shoulder_pitch', 'left_elbow']
            goal.target.position = [-.8, -.4]
            goal.options.velocity_scale = .05
            goal.options.timeout_sec = timeout
            handle = wait_future(move_client.send_goal_async(goal))
            assert handle.accepted
            return handle

        for _ in range(8):
            handle = start_move(.04)
            result = wait_future(handle.get_result_async())
            assert result.result.status.code == Status.TIMEOUT, str(result)
        handle = start_move(10.)
        wait_future(handle.cancel_goal_async())
        result = wait_future(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED, str(result)
        start_move(10.)  # Still active when the node receives SIGINT below.
        report['move_timeout_cancel_and_active_shutdown'] = True
        report['passed'] = True
    finally:
        if proc is not None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                raise AssertionError('motion node did not join workers during shutdown')
        observer.destroy_node()
        rclpy.shutdown()
        (root / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--node', required=True, type=Path)
    run(parser.parse_args().node)
