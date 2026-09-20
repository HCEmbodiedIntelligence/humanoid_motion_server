#!/usr/bin/env python3
"""Explicit integration check: starts/stops its own headless simulation.

Source the built workspace, then run this script. Requires hc_teleop_recv and
the OpenArmX deployment, or --profile with another UDP teleop configuration.
Uses domain 223 and UDP 15105/15106, never the user's running simulation.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default=str(Path(__file__).resolve().parents[1] / 'config/simulation/openarmx.yaml'))
    parser.add_argument('--domain-id', type=int, default=223)
    parser.add_argument('--pose-port', type=int, default=15105)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from humanoid_motion_interfaces.action import MoveJ
    from humanoid_motion_server.simulation.configuration import load_configuration
    from humanoid_motion_server.simulation.home import home_targets
    from hc_teleop_recv.protocol import PACKET_FORMAT
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool

    configuration = load_configuration(get_package_share_directory('humanoid_motion_server'), args.profile)
    rclpy.init()
    node = rclpy.create_node('verify_simulation_home')
    log = tempfile.NamedTemporaryFile(prefix='simulation-home-', suffix='.log', delete=False)
    process = None
    latest, events, requests, samples, servo = {}, [], [], [], []
    controls = dict(outside=False, a=False, grip=False, sequence=0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def state(message):
        latest['state'] = dict(zip(message.name, message.position))
        samples.append((time.monotonic(), latest['state']))

    def status(key, message):
        latest[key] = json.loads(message.data)

    subscriptions = [node.create_subscription(JointState, configuration['motion']['joint_state_endpoint'], state, qos_profile_sensor_data),
        node.create_subscription(String, '/simulation/status', lambda m: status('simulation', m), 10),
        node.create_subscription(String, '/hc_teleop_recv/status', lambda m: status('teleop', m), 10),
        node.create_subscription(String, '/hc_teleop_recv/events', lambda m: events.append(json.loads(m.data)), 10),
        node.create_subscription(String, '/hc_teleop_recv/actions', lambda m: requests.append(json.loads(m.data)), 10)]
    for channel in configuration['channels']:
        if channel['kind'] == 'servo_p':
            subscriptions.append(node.create_subscription(PoseStamped, channel['endpoint'],
                                  lambda m: servo.append(time.monotonic()), qos_profile_sensor_data))
    # Use the same resolved stop endpoint as the frontend.
    from hc_teleop_recv.config import load_config
    teleop_config = load_config(configuration['resources']['hc_teleop_config'])
    stop_topic = teleop_config.emergency_stop_topic
    stop = node.create_publisher(Bool, stop_topic, 10)
    action = node.create_publisher(String, '/hc_teleop_recv/actions', 10)
    enabled = node.create_client(SetBool, '/hc_teleop_recv/set_enabled')

    def packet():
        controls['sequence'] += 1
        values = [b'PICO', 2, controls['sequence'], time.monotonic(), 7]
        for _ in range(3):
            values.extend([0., 0., 0., 0., 0., 0., 1.])
        for side in ('left', 'right'):
            held = (1 if side == 'right' and controls['a'] else 0) | (4 if controls['grip'] else 0)
            axis = (-1. if side == 'left' else 1.) if controls['outside'] else 0.
            values.extend([held, 0, 0, 0., float(controls['grip']), axis, 0., 0., 0.])
        sock.sendto(struct.pack(PACKET_FORMAT, *values), ('127.0.0.1', args.pose_port))

    def pump(duration):
        end, next_packet = time.monotonic() + duration, 0
        while time.monotonic() < end:
            if time.monotonic() >= next_packet:
                packet()
                next_packet = time.monotonic() + 1 / 60
            rclpy.spin_once(node, timeout_sec=.003)
            if process is not None:
                assert process.poll() is None, Path(log.name).read_text()[-8000:]

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (latest.get('teleop'), events, log.name)
            pump(.02)

    def offset():
        controls.update(a=False, grip=False, outside=False)
        future = enabled.call_async(SetBool.Request(data=False))
        wait(future.done)
        assert future.result().success
        pump(.4)
        goals = []
        for target in home_targets(configuration):
            client = ActionClient(node, MoveJ, target['endpoint'])
            assert client.wait_for_server(timeout_sec=5)
            goal = MoveJ.Goal()
            goal.group_name = target['group']
            goal.target.name = target['joint_names']
            goal.target.position = target['positions'].copy()
            name = goal.target.name[-1]
            limits = configuration['joints'][name]
            direction = 1 if goal.target.position[-1] < (limits['lower'] + limits['upper']) / 2 else -1
            goal.target.position[-1] += direction * min(.3, (limits['upper'] - limits['lower']) / 4)
            goal.options.timeout_sec = 20.
            future = client.send_goal_async(goal)
            wait(future.done)
            assert future.result().accepted
            goals.append((client, future.result().get_result_async()))
        for client, future in goals:
            wait(future.done, 25)
            assert future.result().result.status.code == 0
            client.destroy()
        pump(.2)
        assert max(abs(latest['state'][n] - v) for n, v in configuration['initial'].items()) > .05

    report = {}
    try:
        pump(.5)
        assert 'simulation' not in latest and 'teleop' not in latest, 'Test domain is already occupied'
        process = subprocess.Popen(['ros2', 'launch', 'humanoid_motion_server', 'simulation.launch.py',
            'profile:=' + args.profile, 'meshcat:=false', 'start_teleop:=true',
            f'domain_id:={args.domain_id}', f'pose_port:={args.pose_port}',
            f'discovery_port:={args.pose_port + 1}', 'simulation_python:=/usr/bin/python3'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait(lambda: all(k in latest for k in ('simulation', 'state', 'teleop')))
        assert latest['simulation']['mode'] == 'kinematic_simulation'
        pump(1)
        offset()
        home_samples = len(samples)
        controls.update(a=True, grip=True)
        wait(lambda: latest['teleop']['enabled'])
        pump(.3)
        controls.update(a=False, outside=True)
        wait(lambda: latest['teleop']['motion_active'])
        lock_at = time.monotonic()
        controls['a'] = True
        future = enabled.call_async(SetBool.Request(data=True))
        wait(future.done)
        assert not future.result().success, 'Resume must be rejected during home'
        wait(lambda: bool(events), 25)
        assert events[-1]['ok'], events
        pump(.5)
        assert not latest['teleop']['enabled'] and not latest['teleop']['motion_active']
        error = max(abs(latest['state'][n] - v) for n, v in configuration['initial'].items())
        assert error < .011, error
        assert not any(t > lock_at + .1 for t in servo), 'Servo target leaked during home'
        assert len([r for r in requests if r['action'] == 'home']) == 1, 'Held gesture repeated'
        for target in home_targets(configuration):
            moved_joint = target['joint_names'][-1]
            assert len({round(s[moved_joint], 4) for _, s in samples[home_samples:]}) > 10
        report.update(home='passed', maximum_home_error_rad=error, held_gesture_requests=1,
                      resume_during_home='rejected', servo_during_home=0)
        controls.update(a=False, outside=False)
        pump(.2)
        controls['a'] = True
        wait(lambda: latest['teleop']['enabled'])
        pump(.6)
        assert max(abs(latest['state'][n] - v) for n, v in configuration['initial'].items()) < .03
        report['resume_reference'] = 'no old-target jump'

        offset()
        before = len(events)
        controls['outside'] = True
        wait(lambda: latest['teleop']['motion_active'])
        pump(.4)
        stop.publish(Bool(data=True))
        wait(lambda: len(events) > before)
        assert not events[-1]['ok'], events[-1]
        pump(.4)
        assert not latest['teleop']['motion_active'] and not latest['teleop']['enabled']
        stopped = latest['state'].copy()
        pump(.4)
        assert max(abs(stopped[n] - latest['state'][n]) for n in stopped) < 1e-8
        report['cancel'] = 'stopped, teleop disabled, ownership released'

        before = len(events)
        for stale, identity in ((True, latest['teleop']['configuration']['sha256']), (False, 'wrong')):
            action.publish(String(data=json.dumps(dict(id=uuid.uuid4().hex, action='home',
                robot_id=teleop_config.robot_id, pose_id='simulation_initial',
                configuration_sha256=identity, stamp_ns=time.time_ns() - (10_000_000_000 if stale else 0)))))
        pump(.5)
        assert len(events) == before and not latest['teleop']['motion_active']
        report['stale_and_wrong_identity'] = 'ignored'
        report['log'] = log.name
        print(json.dumps(report, indent=2))
    finally:
        if process is not None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        sock.close()
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()
