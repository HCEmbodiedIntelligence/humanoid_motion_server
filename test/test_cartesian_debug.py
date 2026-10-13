"""Keyboard and marker regressions without opening a ROS graph or terminal."""

import copy
import math
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from humanoid_motion_server.cartesian_debug import pose_error, quaternion, step_pose, transform
from humanoid_motion_server.keyboard_servo import KeyboardServoNode, select_channel
from humanoid_motion_server.simulation.pose_markers import PoseMarkers, segment_transform


CHANNEL = dict(name='custom_servo', kind='servo_p', group='custom_arm',
               endpoint='/custom/target', fk_pose_topic='/custom/fk', base_frame='custom_base', tip_frame='tcp')


def pose(frame='custom_base', position=(0.3, -0.2, 0.5), xyzw=(0., 0., 0., 1.), stamp=10):
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = stamp
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    q = message.pose.orientation
    q.x, q.y, q.z, q.w = xyzw
    return message


@pytest.mark.parametrize('local,expected', [(False, (0.5, -0.5, 0.5, 0.5)),
                                          (True, (0.5, 0.5, 0.5, 0.5))])
def test_reference_and_tool_rotations_differ_at_nonidentity_pose(local, expected):
    initial = pose(xyzw=(0., 0., math.sqrt(.5), math.sqrt(.5)))
    result = step_pose(initial, 0, math.pi/2, angular=True, local=local)
    assert quaternion(result.pose) == pytest.approx(expected)
    assert result.pose.position == initial.pose.position
    assert quaternion(initial.pose) == pytest.approx((0, 0, math.sqrt(.5), math.sqrt(.5)))


@pytest.mark.parametrize('local,expected', [(False, (.31, -.2, .5)), (True, (.3, -.19, .5))])
def test_translation_follows_selected_reference_or_target_tool_axis(local, expected):
    initial = pose(xyzw=(0., 0., math.sqrt(.5), math.sqrt(.5)))
    result = step_pose(initial, 0, .01, local=local)
    assert transform(result)[:3, 3] == pytest.approx(expected)


def test_quaternion_sign_does_not_change_pose_error():
    assert pose_error(transform(pose()), transform(pose(xyzw=(0., 0., 0., -1.)))) == (0., 0.)


def test_channel_selection_uses_configuration_and_rejects_ambiguity():
    assert select_channel([CHANNEL], group='custom_arm') == CHANNEL
    with pytest.raises(ValueError, match='多个'):
        select_channel([CHANNEL, {**CHANNEL, 'name': 'another'}])
    with pytest.raises(ValueError, match='没有匹配'):
        select_channel([CHANNEL], group='left_arm')


@pytest.fixture
def keyboard(monkeypatch):
    from humanoid_motion_server import keyboard_servo
    clock = [10.0]
    monkeypatch.setattr(keyboard_servo.time, 'monotonic', lambda: clock[0])
    samples = []
    state = JointState(name=['custom_joint'], position=[.3], velocity=[0.])
    state.header.stamp.sec = 10
    node = NS(configuration={'robot_id': 'custom_robot'}, channel=CHANNEL,
              names={'custom_joint'}, args=NS(allow_hardware=False, step=.005, angle_step=2.,
                                             max_offset=.1, max_angle=60.),
              simulation={'mode': 'kinematic_simulation', 'robot_id': 'custom_robot'},
              simulation_seen=10., feedback_age=.1, lease=.1, pose=pose(), state=state,
              armed=False, local=False, done=False, scale=1., target=None, last_publish=None,
              notice='', get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(clock[0]*1e9),
                                                             to_msg=lambda: state.header.stamp)),
              publisher=NS(get_subscription_count=lambda: 1,
                           publish=lambda m: samples.append(copy.deepcopy(m))))
    for method in ('feedback_reason', 'pause', 'rebase', 'handle_key', 'publish_target'):
        setattr(node, method, MethodType(getattr(KeyboardServoNode, method), node))
    return node, clock, samples


def test_start_paused_rebase_move_pause_and_quit(keyboard):
    node, _, samples = keyboard
    node.handle_key('w')
    node.publish_target()
    assert not samples and not node.armed
    node.handle_key('\n')
    node.handle_key('w')
    node.handle_key('o')
    node.publish_target()
    assert len(samples) == 1
    assert samples[-1].pose.position.x == pytest.approx(.305)
    assert pose_error(transform(node.pose), transform(samples[-1]))[1] == pytest.approx(math.radians(2))
    node.handle_key(' ')
    node.publish_target()
    assert len(samples) == 1 and not node.armed
    node.pose.pose.position.x = .7
    node.handle_key('\n')
    assert node.target.pose.position.x == .7  # Never resume the old target.
    node.handle_key('q')
    assert node.done and not node.armed


@pytest.mark.parametrize('fault', ['stale_fk', 'stale_simulation', 'wrong_robot', 'wrong_frame', 'invalid_quat', 'gap'])
def test_feedback_or_publish_failure_requires_manual_rearm(keyboard, fault):
    node, clock, samples = keyboard
    node.handle_key('\n')
    if fault == 'stale_fk':
        node.pose.header.stamp.sec = 9
    elif fault == 'stale_simulation':
        node.simulation_seen = 1.
    elif fault == 'wrong_robot':
        node.simulation['robot_id'] = 'other'
    elif fault == 'wrong_frame':
        node.pose.header.frame_id = 'other_frame'
    elif fault == 'invalid_quat':
        node.pose.pose.orientation.w = math.nan
    elif fault == 'gap':
        clock[0] += .101
    node.publish_target()
    assert not samples and not node.armed


def test_keyboard_rejects_target_ahead_of_feedback_limit(keyboard):
    node, _, _ = keyboard
    node.handle_key('\n')
    node.args.max_offset = .006
    node.handle_key('w')
    node.handle_key('w')
    assert node.target.pose.position.x == pytest.approx(.305)


def test_terminal_disconnect_pauses_before_servicing_another_publish(monkeypatch):
    from humanoid_motion_server import keyboard_servo
    calls = []
    monkeypatch.setattr(keyboard_servo.select, 'poll', lambda: NS(
        register=lambda *args: None, poll=lambda timeout: [(0, keyboard_servo.select.POLLHUP)]))
    monkeypatch.setattr(keyboard_servo.curses, 'curs_set', lambda value: None)
    monkeypatch.setattr(keyboard_servo.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(keyboard_servo.rclpy, 'spin_once', lambda *args, **kwargs: pytest.fail('published after disconnect'))
    screen = NS(nodelay=lambda value: None, keypad=lambda value: None)
    node = NS(done=False, pause=lambda reason: calls.append(reason))
    keyboard_servo.terminal(screen, node)
    assert len(calls) == 1 and '终端已断开' in calls[0]


class Scene:
    def __init__(self):
        self.nodes = {}

    def __getitem__(self, path):
        if path not in self.nodes:
            node = NS(properties={})
            node.set_property = lambda key, value: node.properties.update({key: value})
            node.set_transform = lambda matrix: setattr(node, 'matrix', np.array(matrix))
            node.set_object = lambda *args: setattr(node, 'object', args)
            self.nodes[path] = node
        return self.nodes[path]


class Geometry:
    def __getattr__(self, name):
        return lambda *args, **kwargs: NS(kind=name, args=args, kwargs=kwargs)


def test_marker_axes_are_positive_xyz_and_target_is_an_asymmetric_box():
    scene = Scene()
    PoseMarkers(scene, Geometry(), [CHANNEL])
    for index, axis in enumerate('XYZ'):
        matrix = scene[f'pose_debug/custom_servo/actual_FK/axes/{axis}/shaft'].matrix
        assert matrix[:3, :3] @ np.array([0., 1., 0.]) == pytest.approx(np.eye(3)[index])
    box = scene['pose_debug/custom_servo/target_ServoP/box']
    assert box.object[0].kind == 'Box'
    assert box.matrix[0, 3] > 0


@pytest.mark.parametrize('end', [(1., 2., 3.), (0., 1., 0.), (-1., 0., 0.)])
def test_error_line_survives_threejs_rotation_scale_decomposition(end):
    start = np.array([.1, -.2, .3])
    matrix = segment_transform(start, end)
    basis = matrix[:3, :3]
    scales = np.linalg.norm(basis, axis=0)
    rotation = basis / scales
    assert rotation.T @ rotation == pytest.approx(np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1.)
    assert matrix @ np.array([0., 0., 0., 1.]) == pytest.approx([*start, 1.])
    assert matrix @ np.array([1., 0., 0., 1.]) == pytest.approx([*end, 1.])
    assert segment_transform(start, start) is None


def test_marker_world_transform_handles_different_pose_frames_and_staleness():
    scene = Scene()
    markers = PoseMarkers(scene, Geometry(), [CHANNEL])
    base = transform(pose(position=(1., 2., 3.), xyzw=(0., 0., math.sqrt(.5), math.sqrt(.5))))
    actual = pose(position=(.1, 0., 0.))
    target = pose(frame='world', position=(1., 2.1, 3.), xyzw=(0., 0., math.sqrt(.5), math.sqrt(.5)))
    markers.receive(CHANNEL['name'], 'actual', actual, 10_000_000_000)
    markers.receive(CHANNEL['name'], 'target', target, 10_000_000_000)
    lookup = lambda frame: base if frame == 'custom_base' else np.eye(4)
    markers.render(10_010_000_000, lookup, True)
    status = markers.status()['custom_servo']
    assert status['position_error_m'] < 1e-10
    assert status['orientation_error_deg'] < 1e-6
    assert status['actual_world_xyz_m'] == pytest.approx((1., 2.1, 3.))
    assert status['actual_xyz_m'] == pytest.approx((.1, 0., 0.))
    assert scene['pose_debug/custom_servo/target_ServoP/box'].properties['color'] == [1., 170/255, .2, .35]
    markers.render(10_400_000_000, lookup, True)
    status = markers.status()['custom_servo']
    assert not status['actual_visible']
    assert status['target_visible'] and not status['target_fresh']
    assert scene['pose_debug/custom_servo/target_ServoP/box'].properties['color'] == [.53, .53, .53, .18]
    markers.render(10_410_000_000, lookup, False)
    assert not markers.status()['custom_servo']['target_visible']


def test_unknown_reference_frame_is_hidden_not_assumed_to_be_world():
    markers = PoseMarkers(Scene(), Geometry(), [CHANNEL])
    markers.receive('custom_servo', 'target', pose(frame='unknown'), 10_000_000_000)

    def lookup(frame):
        raise ValueError('unknown frame')

    markers.render(10_000_000_000, lookup, True)
    status = markers.status()['custom_servo']
    assert not status['target_visible'] and status['target_error'] == 'unknown frame'


def test_invalid_quaternion_and_clock_reset_clear_cached_markers():
    markers = PoseMarkers(Scene(), Geometry(), [CHANNEL])
    markers.receive('custom_servo', 'target', pose(), 10_000_000_000)
    markers.render(10_000_000_000, lambda _: np.eye(4), True)
    markers.receive('custom_servo', 'actual', pose(xyzw=(0., 0., 0., 0.)), 10_000_000_000)
    assert markers.entries['custom_servo']['actual'] is None
    markers.render(1_000_000_000, lambda _: np.eye(4), True)
    assert not markers.status()['custom_servo']['target_visible']
