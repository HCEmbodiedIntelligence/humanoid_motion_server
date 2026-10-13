"""Cartesian demo targets, without creating a ROS node or publishing commands."""
import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped


SPEC = importlib.util.spec_from_file_location(
    'motion_demo_common', Path(__file__).resolve().parents[1] / 'examples' / '_common.py')
common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(common)
HALF_SQRT = math.sqrt(0.5)


def make_target(argv, quaternion=(0.0, 0.0, 0.0, 1.0), *, stream=True):
    feedback = PoseStamped()
    feedback.header.frame_id = 'base_link'
    feedback.header.stamp.sec = 123
    feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = 0.3, -0.2, 0.6
    q = feedback.pose.orientation
    q.x, q.y, q.z, q.w = quaternion
    original = copy.deepcopy(feedback)

    def wait(condition):
        assert condition()

    node = SimpleNamespace(pose=feedback, wait=wait, fresh=lambda message: True)
    args = common.parser('test', stream=stream).parse_args(argv)
    target = common.DemoNode.pose_target(node, args)
    assert feedback == original  # A new command must never modify measured FK.
    assert target.header == feedback.header
    return target.pose


def xyzw(pose):
    q = pose.orientation
    return q.x, q.y, q.z, q.w


@pytest.mark.parametrize('option,expected', [
    ('--droll', (HALF_SQRT, 0.0, 0.0, HALF_SQRT)),
    ('--dpitch', (0.0, HALF_SQRT, 0.0, HALF_SQRT)),
    ('--dyaw', (0.0, 0.0, HALF_SQRT, HALF_SQRT)),
])
@pytest.mark.parametrize('sign', [-1, 1])
def test_rotation_only_preserves_tcp_position(option, expected, sign):
    pose = make_target([option, str(sign * math.pi / 2)])
    expected = tuple(sign * value for value in expected[:3]) + (expected[3],)
    assert xyzw(pose) == pytest.approx(expected)
    assert (pose.position.x, pose.position.y, pose.position.z) == (0.3, -0.2, 0.6)


def test_increment_uses_reference_axes_with_nonidentity_fk():
    # After an initial 90-degree Z rotation, a fixed-X turn has negative q.y.
    # Right multiplication (a tool-X turn) would instead produce positive q.y.
    pose = make_target(['--droll', str(math.pi / 2)], (0.0, 0.0, HALF_SQRT, HALF_SQRT))
    assert xyzw(pose) == pytest.approx((0.5, -0.5, 0.5, 0.5))


@pytest.mark.parametrize('stream', [False, True])
def test_combined_xyz_and_rpy_have_fixed_xyz_rotation_order(stream):
    quarter_turn = str(math.pi / 2)
    pose = make_target(['--dx', '0.01', '--dy', '-0.02', '--dz', '0.03',
                        '--droll', quarter_turn, '--dpitch', quarter_turn,
                        '--dyaw', quarter_turn], stream=stream)
    assert (pose.position.x, pose.position.y, pose.position.z) == pytest.approx((0.31, -0.22, 0.63))
    assert xyzw(pose) == pytest.approx((0.0, HALF_SQRT, 0.0, HALF_SQRT))
    assert math.hypot(*xyzw(pose)) == pytest.approx(1.0)


def test_default_keeps_current_orientation_and_position_only_keeps_orientation():
    quaternion = (0.0, 0.0, 2.0, 2.0)
    expected = (0.0, 0.0, HALF_SQRT, HALF_SQRT)
    assert xyzw(make_target([], quaternion)) == pytest.approx(expected)
    assert xyzw(make_target(['--dx', '0.01'], quaternion)) == pytest.approx(expected)


@pytest.mark.parametrize('option', ['--droll', '--dpitch', '--dyaw'])
@pytest.mark.parametrize('value', ['nan', 'inf', '-inf'])
def test_angles_must_be_finite(option, value):
    with pytest.raises(SystemExit) as error:
        common.parser('test', stream=True).parse_args([f'{option}={value}'])
    assert error.value.code == 2
