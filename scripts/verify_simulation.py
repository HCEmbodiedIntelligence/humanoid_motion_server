#!/usr/bin/env python3
"""Exercise MoveJ, ServoP, fresh feedback and watchdog on a running simulation.

Run in the same dedicated domain as simulation.launch.py. Refuses to send goals
unless /simulation/status identifies this profile's kinematic simulation.
"""
import argparse
import json
import math
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='')
    parser.add_argument('--robot-id', default='')
    parser.add_argument('--domain-id', type=int, default=199)
    parser.add_argument('--expect-viewer', action='store_true')
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import numpy as np
    import pinocchio as pin
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from humanoid_motion_interfaces.action import MoveJ
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from humanoid_motion_server.simulation.configuration import load_configuration, read_yaml

    configuration = load_configuration(get_package_share_directory('humanoid_motion_server'),
                                       args.profile, args.robot_id)
    rclpy.init()
    node = rclpy.create_node('verify_offline_simulation')
    latest, commands, subscriptions = {}, [], []
    def save(key, message):
        latest[key] = message

    def subscribe(kind, topic, key):
        subscriptions.append(node.create_subscription(kind, topic, lambda m: save(key, m), qos_profile_sensor_data))

    motion = configuration['motion']
    subscribe(String, '/simulation/status', 'status')
    subscribe(String, '/simulation/viewer_status', 'viewer')
    subscribe(JointState, motion['joint_state_endpoint'], 'state')
    subscribe(JointState, '/simulation/joint_states', 'display')
    subscriptions.append(node.create_subscription(JointState, motion['joint_command_endpoint'],
                                                  lambda m: commands.append((time.monotonic(), m)), qos_profile_sensor_data))
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(.005, end - time.monotonic())))

    def wait(predicate, timeout=15):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('Timed out waiting for simulation')
            pump(.02)

    report = {}
    try:
        wait(lambda: 'status' in latest and 'state' in latest)
        status = json.loads(latest['status'].data)
        assert status['mode'] == 'kinematic_simulation' and status['robot_id'] == configuration['robot_id'], status
        if args.expect_viewer:
            wait(lambda: 'viewer' in latest and json.loads(latest['viewer'].data)['feedback_fresh'])
        servo_channels = {channel['group']: channel for channel in configuration['channels']
                          if channel['kind'] == 'servo_p'}
        move_channels = {channel['group']: channel for channel in configuration['channels']
                         if channel['kind'] == 'move_j' and channel['group'] in servo_channels}
        assert move_channels and servo_channels, 'Profile needs MoveJ and ServoP channels'
        initial = dict(zip(latest['state'].name, latest['state'].position))
        goals = []
        for group, channel in move_channels.items():
            client = ActionClient(node, MoveJ, channel['endpoint'])
            assert client.wait_for_server(timeout_sec=10), channel['endpoint']
            goal = MoveJ.Goal()
            goal.group_name = group
            goal.target.name = motion[f'groups.{group}']
            values = [initial[n] for n in goal.target.name]
            # Move the last joint towards the interior of its permitted range.
            name = goal.target.name[-1]
            limits = configuration['joints'][name]
            direction = 1 if values[-1] <= (limits['lower'] + limits['upper']) / 2 else -1
            values[-1] += .08 * direction
            values[-1] = min(max(values[-1], limits['lower']), limits['upper'])
            goal.target.position = values
            goal.options.timeout_sec = 20.0
            future = client.send_goal_async(goal)
            wait(future.done)
            handle = future.result()
            assert handle.accepted, channel['endpoint']
            goals.append((group, client, handle.get_result_async(), values))
        for group, client, future, values in goals:
            wait(future.done, 25)
            result = future.result().result
            assert result.status.code == 0, f'{group}: {result.status}'
            report[f'{group}_move_j'] = 'passed'
            client.destroy()
        pump(.3)
        actual = dict(zip(latest['state'].name, latest['state'].position))
        assert max(abs(actual[n] - initial[n]) for n in actual) > .03
        model = pin.buildModelFromUrdf(configuration['resources']['urdf'])
        data = model.createData()
        neutral = pin.neutral(model)
        def set_joint(q, name, value):
            joint = model.joints[model.getJointId(name)]
            if joint.nq == 1:
                q[joint.idx_q] = value
            else:
                q[joint.idx_q:joint.idx_q + 2] = [math.cos(value), math.sin(value)]
        for name, value in zip(latest['display'].name, latest['display'].position):
            set_joint(neutral, name, value)
        tools = {item['child_frame']: item for item in read_yaml(configuration['resources']['tool_config'])['tools']}
        publishers = {group: node.create_publisher(PoseStamped, c['endpoint'], 1)
                      for group, c in servo_channels.items()}
        pump(.3)
        commands.clear()
        begin = time.monotonic()
        while time.monotonic() - begin < 3.0:
            elapsed = time.monotonic() - begin
            for group, channel in servo_channels.items():
                q = neutral.copy()
                name = motion[f'groups.{group}'][-1]
                limits = configuration['joints'][name]
                amplitude = max(0.0, min(.01, (limits['upper'] - actual[name]) / 2,
                                         (actual[name] - limits['lower']) / 2))
                set_joint(q, name, actual[name] + amplitude * math.sin(elapsed * 2))
                pin.framesForwardKinematics(model, data, q)
                tool = tools[channel['tip_frame']]
                x, y, z, w = tool['rotation_xyzw']
                offset = pin.SE3(pin.Quaternion(w, x, y, z).matrix(), np.array(tool['translation_m']))
                transform = (data.oMf[model.getFrameId(channel['base_frame'])].inverse() *
                             data.oMf[model.getFrameId(tool['parent_frame'])] * offset)
                pose = PoseStamped()
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.header.frame_id = channel['base_frame']
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, transform.translation)
                quat = pin.Quaternion(transform.rotation).coeffs()
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quat)
                publishers[group].publish(pose)
            pump(1 / 60)
        last_input = time.monotonic()
        pump(.5)
        for group in servo_channels:
            names = set(motion[f'groups.{group}'])
            times = [t for t, message in commands if names.intersection(message.name)]
            assert len(times) > 100, (group, len(times))
            max_gap = max(b - a for a, b in zip(times, times[1:]))
            assert max_gap < .1, (group, max_gap)
            assert times[-1] - last_input < .15, (group, times[-1] - last_input)
            report[f'{group}_servo_p'] = {'commands': len(times), 'max_gap_ms': round(max_gap * 1000, 2),
                                         'stop_after_input_ms': round((times[-1] - last_input) * 1000, 2)}
        before = list(latest['state'].position)
        pump(.3)
        assert max(abs(a - b) for a, b in zip(before, latest['state'].position)) < 1e-9
        gripper_publishers, gripper_states = [], {}
        def receive_grippers(message):
            gripper_states.update(zip(message.name, message.position))
        for gripper in configuration['grippers']:
            subscriptions.append(node.create_subscription(JointState, gripper['feedback_topic'],
                                                          receive_grippers, qos_profile_sensor_data))
            gripper_publishers.append((node.create_publisher(JointState, gripper['command_topic'], 10), gripper))
        if gripper_publishers:
            wait(lambda: all(g['name'] in gripper_states for _, g in gripper_publishers))
            gripper_before = dict(gripper_states)
            for _ in range(10):
                for publisher, gripper in gripper_publishers:
                    command = JointState()
                    command.header.stamp = node.get_clock().now().to_msg()
                    command.name = [gripper['name']]
                    target = gripper['lower'] if gripper['initial'] != gripper['lower'] else gripper['upper']
                    command.position = [target]
                    publisher.publish(command)
                pump(.05)
            pump(.35)
            positions = dict(gripper_states)
            display = dict(zip(latest['display'].name, latest['display'].position))
            for _, gripper in gripper_publishers:
                name = gripper['name']
                assert abs(gripper_before[name] - positions[name]) > min(
                    .005, gripper['speed'] * .1, (gripper['upper'] - gripper['lower']) * .1), (name, positions)
                binding = configuration['gripper_joints'].get(name)
                if binding:
                    assert abs(display[binding['joint']] - positions[name] * binding.get('scale', 1.0) - binding.get('offset', 0.0)) < 1e-8
            report['grippers'] = positions
        if args.expect_viewer:
            report['viewer'] = json.loads(latest['viewer'].data)
            assert report['viewer']['frames'] > 30 and report['viewer']['feedback_fresh']
        report['simulation'] = json.loads(latest['status'].data)
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
