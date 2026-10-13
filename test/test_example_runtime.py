"""Failure-path regressions for demos; no ROS graph or hardware connection."""
import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from sensor_msgs.msg import JointState


SPEC = importlib.util.spec_from_file_location(
    'motion_demo_runtime', Path(__file__).resolve().parents[1] / 'examples' / '_common.py')
common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(common)


@pytest.mark.parametrize('velocity', [[], [math.nan], [math.inf], [0.0, 0.0]])
def test_feedback_must_have_complete_finite_velocities(velocity):
    state = JointState(name=['j1'], position=[0.0], velocity=velocity)
    node = NS(state=state, names=['j1'], fresh=lambda message: True)
    assert not common.DemoNode.valid_state(node)
    state.velocity = [0.0]
    assert common.DemoNode.valid_state(node)


def test_joint_offset_overflow_is_rejected_before_command_creation():
    node = NS(state=JointState(name=['j1'], position=[1e308], velocity=[0.0]),
              names=['j1'], wait=lambda condition: None, valid_state=lambda: True)
    with pytest.raises(RuntimeError, match='有限数值'):
        common.DemoNode.joint_target(node, NS(positions=None, joint=1, delta=1e308))


@pytest.mark.parametrize('bad_field', ['position_overflow', 'quaternion_overflow', 'empty_frame'])
def test_invalid_pose_arithmetic_is_rejected(bad_field):
    pose = PoseStamped()
    pose.header.frame_id = 'base'
    pose.pose.orientation.w = 1.0
    args = common.parser('test').parse_args([])
    if bad_field == 'position_overflow':
        pose.pose.position.x = args.dx = 1e308
    elif bad_field == 'quaternion_overflow':
        q = pose.pose.orientation
        q.x = q.y = q.z = q.w = 1e308
    else:
        pose.header.frame_id = ' '
    node = NS(pose=pose, fresh=lambda message: True, wait=lambda condition: None)
    with pytest.raises(RuntimeError, match='位姿'):
        common.DemoNode.pose_target(node, args)


@pytest.fixture
def stream(monkeypatch):
    clock = [0.0]
    samples = []
    pose = PoseStamped()
    pose.header.frame_id = 'base'
    pose.pose.orientation.w = 1.0
    node = NS(wait=lambda condition: None, pose=pose, fresh=lambda message: True,
              valid_state=lambda: True,
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: pose.header.stamp)))
    publisher = NS(get_subscription_count=lambda: 1,
                   publish=lambda message: samples.append(copy.deepcopy(message)))
    monkeypatch.setattr(common.time, 'monotonic', lambda: clock[0])

    def spin(node, timeout_sec):
        clock[0] += max(timeout_sec, 0.0001)

    monkeypatch.setattr(common.rclpy, 'spin_once', spin)
    target = JointState(name=['j1'], position=[0.0])

    def run(make_message=None, **kwargs):
        common.DemoNode.stream(node, publisher, make_message or (lambda fraction: target),
                              NS(duration=0.05, rate=100.0), **kwargs)

    return NS(clock=clock, samples=samples, node=node, publisher=publisher, target=target, run=run)


def test_stall_past_deadline_is_not_reported_as_success(stream):
    def publish(message):
        stream.samples.append(message)
        stream.clock[0] = 1.0

    stream.publisher.publish = publish
    with pytest.raises(RuntimeError, match='发布间隔'):
        stream.run(hold=0.2)
    assert len(stream.samples) == 1


def test_slow_target_generation_does_not_publish_after_lease_gap(stream):
    def make_message(fraction):
        if stream.samples:
            stream.clock[0] += 0.2
        return stream.target

    with pytest.raises(RuntimeError, match='发布间隔'):
        stream.run(make_message)
    assert len(stream.samples) == 1


def test_fk_corruption_during_stream_stops_further_commands(stream):
    def publish(message):
        stream.samples.append(message)
        stream.node.pose.pose.orientation.x = math.nan

    stream.publisher.publish = publish
    with pytest.raises(RuntimeError, match='位姿'):
        stream.run(require_pose=True)
    assert len(stream.samples) == 1


@pytest.mark.parametrize('field', ['position', 'velocity'])
def test_invalid_generated_command_never_reaches_publisher(stream, field):
    setattr(stream.target, field, [math.nan])
    with pytest.raises(RuntimeError, match='关节指令'):
        stream.run()
    assert not stream.samples


def test_stale_feedback_and_lost_subscriber_stop_stream(stream):
    stream.node.valid_state = lambda: False
    with pytest.raises(RuntimeError, match='反馈'):
        stream.run()
    stream.node.valid_state = lambda: True
    stream.publisher.get_subscription_count = lambda: 0
    with pytest.raises(RuntimeError, match='订阅者'):
        stream.run()
    assert not stream.samples


def test_joint_cmd_reaches_final_position_with_zero_velocity(stream):
    spec = importlib.util.spec_from_file_location(
        'motion_joint_demo', Path(__file__).resolve().parents[1] / 'examples' / 'joint_cmd.py')
    module = importlib.util.module_from_spec(spec)
    # This example imports its adjacent helper as _common when run directly.
    import sys
    from unittest.mock import patch
    with patch.dict(sys.modules, {'_common': common}):
        spec.loader.exec_module(module)
    node = stream.node
    node.names, node.group = ['j1'], 'left_arm'
    node.create_publisher = lambda *args: stream.publisher
    node.joint_target = lambda args: ([0.1], [0.2])
    node.stream = lambda publisher, make, args, **kwargs: common.DemoNode.stream(
        node, publisher, make, args, **kwargs)
    module.demo(node, NS(duration=0.05, rate=100.0))
    assert stream.samples[0].position == pytest.approx([0.1])
    assert stream.samples[0].velocity == pytest.approx([0.0])
    assert stream.samples[-1].position == pytest.approx([0.2])
    assert stream.samples[-1].velocity == pytest.approx([0.0])
    assert len(stream.samples) > 20


def resolved(value=None, error=None):
    future = Future()
    if error is None:
        future.set_result(value)
    else:
        future.set_exception(error)
    return future


@pytest.mark.parametrize('failure', ['result_error', 'interrupt', 'timeout', 'success', 'rejected'])
def test_action_cleanup_depends_on_confirmed_terminal_result(failure):
    calls = []
    terminal = NS(status=GoalStatus.STATUS_SUCCEEDED, result=NS(status=NS(code=0, message='ok')))
    responses = [resolved(terminal)]
    if failure == 'result_error':
        responses.insert(0, resolved(error=RuntimeError('transport failed')))
    handle = NS(accepted=failure != 'rejected',
                get_result_async=lambda: responses.pop(0),
                cancel_goal_async=lambda: calls.append('cancel') or resolved(NS(goals_canceling=[1])))
    pending = resolved(handle)
    client = NS(send_goal_async=lambda goal: pending)
    interrupted = False

    def future_result(future, timeout):
        nonlocal interrupted
        if future is not pending and failure in ('interrupt', 'timeout') and not interrupted:
            interrupted = True
            if failure == 'interrupt':
                raise KeyboardInterrupt()
            raise RuntimeError('timeout')
        return future.result()

    node = NS(future_result=future_result)
    args = NS(scale=0.1, timeout=1.0)
    goal = NS(options=NS())
    if failure == 'success':
        common.DemoNode.execute(node, client, goal, args)
    else:
        expected = KeyboardInterrupt if failure == 'interrupt' else RuntimeError
        with pytest.raises(expected):
            common.DemoNode.execute(node, client, goal, args)
    assert calls == ([] if failure in ('success', 'rejected') else ['cancel'])


def test_node_creation_failure_shuts_down_ros(monkeypatch):
    calls = []
    monkeypatch.setattr(common.rclpy, 'init', lambda **kwargs: calls.append('init'))
    monkeypatch.setattr(common.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(common.rclpy, 'shutdown', lambda: calls.append('shutdown'))

    def fail(*args):
        raise RuntimeError('node creation failed')

    monkeypatch.setattr(common, 'DemoNode', fail)
    assert common.run('demo', NS(arm='left'), lambda *args: None) == 1
    assert calls == ['init', 'shutdown']
