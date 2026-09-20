"""Deterministic plant and configuration regressions; no ROS graph or Meshcat needed."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from humanoid_motion_server.simulation.configuration import load_configuration, normalize_parameters
from humanoid_motion_server.simulation.home import home_targets
from humanoid_motion_server.simulation.model import Joint, expand_mimics, resource_path, visual_model
from humanoid_motion_server.simulation.plant import KinematicPlant


SHARE = Path(__file__).resolve().parents[1]


def test_home_uses_complete_simulation_initial_pose_without_overlapping_groups():
    config = load_configuration(SHARE)
    config['initial']['left_elbow'] = -.99
    targets = home_targets(config)
    names = [name for target in targets for name in target['joint_names']]
    assert len(names) == len(set(names)) == len(config['initial'])
    actual = {name: value for target in targets
              for name, value in zip(target['joint_names'], target['positions'])}
    assert actual == config['initial']
    config['channels'] = [c for c in config['channels'] if c['group'] == 'left_arm']
    with pytest.raises(ValueError, match='covering all'):
        home_targets(config)


def test_home_group_selection_backtracks_when_largest_group_cannot_cover_all():
    config = {'initial': dict(a=0, b=1, c=2, d=3), 'motion': {
        'groups.large': ['a', 'b', 'c'], 'groups.left': ['a', 'b'], 'groups.right': ['c', 'd']},
        'channels': [{'kind': 'move_j', 'group': group} for group in ('large', 'left', 'right')]}
    assert [t['group'] for t in home_targets(config)] == ['left', 'right']


def plant():
    result = KinematicPlant({'left': Joint(-2, 2, 1, 'revolute'),
                             'right': Joint(-2, 2, 2, 'revolute')}, {'left': 0, 'right': 0})
    result.step(10.0)
    return result


def test_partial_commands_merge_and_newest_target_overwrites():
    p = plant()
    assert p.command(['left'], [1], 1_000_000_000, 1_000_000_000, 10)
    assert p.command(['right'], [1], 1_000_000_000, 1_000_000_000, 10)
    assert p.command(['left'], [-1], 1_000_000_001, 1_000_000_001, 10)
    p.step(10.01)
    assert p.positions == pytest.approx({'left': -0.01, 'right': 0.02})


@pytest.mark.parametrize('stamp', [0, 900_000_000, 899_999_999, 1_011_000_000])
def test_rejects_zero_stale_or_future_source(stamp):
    p = plant()
    assert not p.command(['left'], [1], stamp, 1_000_000_000, 10)
    p.step(10.01)
    assert p.positions['left'] == 0


def test_replay_cannot_renew_watchdog_and_arms_expire_independently():
    p = plant()
    assert p.command(['left', 'right'], [1, 1], 1_000_000_000, 1_000_000_000, 10)
    p.step(10.04)
    assert not p.command(['left'], [1], 1_000_000_000, 1_040_000_000, 10.04)
    assert p.command(['right'], [1], 1_050_000_000, 1_050_000_000, 10.05)
    p.step(10.08)
    before = p.positions.copy()
    p.step(10.11)
    assert p.positions['left'] == before['left']
    assert p.positions['right'] > before['right']


def test_network_age_consumes_watchdog():
    p = plant()
    assert p.command(['left'], [1], 920_000_000, 1_000_000_000, 10)
    p.step(10.01)
    before = p.positions['left']
    p.step(10.03)
    assert p.positions['left'] == before


@pytest.mark.parametrize('names,values', [(['missing'], [0]), (['left'], [3]),
                                         (['left'], [float('nan')]), (['left'], [float('inf')]),
                                         (['left', 'left'], [0, 1]), (['left'], [])])
def test_malformed_or_out_of_limit_message_is_atomic(names, values):
    p = plant()
    assert not p.command(names, values, 1_000_000_000, 1_000_000_000, 10)
    assert p.targets == {'left': 0, 'right': 0}


def test_stall_holds_without_large_jump():
    p = plant()
    assert p.command(['left'], [1], 1_000_000_000, 1_000_000_000, 10)
    p.step(20)
    assert p.positions['left'] == 0
    assert p.velocities['left'] == 0


def test_clock_rewind_resets_order_and_holds_old_target():
    p = plant()
    assert p.command(['left'], [1], 1_000_000_000, 1_000_000_000, 10)
    p.observe_clock(500_000_000)
    p.step(10.01)
    assert p.positions['left'] == 0
    assert p.command(['left'], [-1], 510_000_000, 510_000_000, 10.01)
    p.step(10.02)
    assert p.positions['left'] < 0


def test_mimic_chains_and_cycles():
    assert expand_mimics({'a': .02}, {'c': ('b', 2, .01), 'b': ('a', -1, 0)}) == pytest.approx(
        {'a': .02, 'b': -.02, 'c': -.03})
    with pytest.raises(ValueError, match='mimic'):
        expand_mimics({}, {'a': ('b', 1, 0), 'b': ('a', 1, 0)})


def test_visual_overlay_preserves_authoritative_limits(tmp_path):
    original = SHARE / 'urdf/test_humanoid.urdf'
    root = ET.parse(original).getroot()
    root.find('joint/limit').set('upper', '999')
    ET.SubElement(ET.SubElement(root.find('link'), 'visual'), 'geometry').append(
        ET.Element('sphere', {'radius': '0.1'}))
    overlay = tmp_path / 'visual.urdf'
    ET.ElementTree(root).write(overlay)
    merged = ET.fromstring(visual_model(original, overlay))
    assert len(merged.findall('.//visual')) == 1
    assert merged.find('joint/limit').get('upper') == '1.2'
    root.find('joint/origin').set('xyz', '1 2 3')
    ET.ElementTree(root).write(overlay)
    with pytest.raises(ValueError, match='kinematics'):
        visual_model(original, overlay)


def test_builtin_profile_does_not_require_manager_or_hardware():
    config = load_configuration(SHARE)
    assert config['robot_id'] == 'test_humanoid'
    assert len(config['joints']) == 6
    assert not config['environment']
    assert not any('driver' in key for key in config['resources'])
    assert config['motion']['use_sim_time'] is False


def test_initial_state_limits_and_unknown_profile_keys(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump({'initial_positions': {'left_elbow': 1}}))
    with pytest.raises(ValueError, match='outside limits'):
        load_configuration(SHARE, path)
    path.write_text('hardware_driver: vendor/RealDriver\n')
    with pytest.raises(ValueError, match='Unknown simulation profile'):
        load_configuration(SHARE, path)


def test_saved_integer_limits_are_typed_as_ros_doubles():
    values = normalize_parameters({'joint_max_velocity_rad_s': 1,
                                   'group_lower_limits.arm': [-1, 0.1], 'feedback_max_age_ms': 100})
    assert type(values['joint_max_velocity_rad_s']) is float
    assert all(type(v) is float for v in values['group_lower_limits.arm'])
    assert type(values['feedback_max_age_ms']) is int


def test_deployed_asset_prefix_and_headless_without_visual_package(tmp_path):
    index = tmp_path / 'prefix/share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    (index / 'private_robot').touch()
    asset = tmp_path / 'prefix/share/private_robot/mesh.stl'
    asset.parent.mkdir()
    asset.write_text('mesh fixture')
    assert resource_path('package://private_robot/mesh.stl', tmp_path,
                         [str(tmp_path / 'prefix')]) == asset
    profile = tmp_path / 'headless.yaml'
    profile.write_text('visual_urdf: package://not_installed/robot.urdf\n')
    assert load_configuration(SHARE, profile)['joints']
