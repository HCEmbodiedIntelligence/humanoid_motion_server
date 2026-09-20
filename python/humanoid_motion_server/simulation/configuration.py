"""Build a simulation snapshot from ordinary motion resources or a deployment.

No driver configuration, vendor launch or hardware plugin is executed. Manager
integration is imported only for robot_id; a file profile works without manager.
"""

from dataclasses import asdict
import math
import os
from pathlib import Path

import yaml

from .model import Joint, joint_metadata, resource_path


RESOURCE_KEYS = ('motion_params', 'sdk_config', 'channel_config', 'tool_config', 'urdf')


def read_yaml(path):
    with Path(path).open(encoding='utf-8') as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f'Expected YAML mapping: {path}')
    return result


def _number(value, label, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{label} must be a finite {"positive " if positive else ""}number')
    return float(value)


def normalize_parameters(parameters):
    """Saved JSON may spell declared ROS doubles as integers."""
    doubles = {
        'control_frequency_hz', 'input_stamp_max_age_s', 'input_stamp_future_tolerance_s',
        'default_move_timeout_s', 'stable_duration_s', 'move_j_position_tolerance_rad',
        'stopped_velocity_tolerance_rad_s', 'cartesian_position_tolerance_m',
        'cartesian_orientation_tolerance_rad', 'feedback_limit_recovery_margin_rad',
        'feedback_rebase_tolerance_rad',
    }
    result = dict(parameters)
    for key, value in result.items():
        if key in doubles or key.startswith(('joint_max_', 'cartesian_max_')):
            result[key] = _number(value, key)
        elif key.startswith(('group_lower_limits.', 'group_upper_limits.')):
            result[key] = [_number(v, key) for v in value]
    # The simulator advances in wall time and stamps feedback with the node clock.
    result['use_sim_time'] = False
    return result


def load_configuration(share, profile='', robot_id='', plugin_root=''):
    share = Path(share)
    document = read_yaml(profile) if profile else {}
    if set(document) - {'robot_id', 'resources', 'visual_urdf', 'initial_pose', 'initial_poses_file',
                        'initial_positions', 'gripper_joints', 'speed_limit', 'watchdog_s'}:
        raise ValueError('Unknown simulation profile key')
    base = Path(profile).resolve().parent if profile else share
    robot_id = robot_id or document.get('robot_id', '')
    environment = {}
    initial_poses_file = document.get('initial_poses_file', '')
    if robot_id:
        from humanoid_manager.deployment import resolve_robot_deployment, DEFAULT_PLUGIN_ROOT
        deployment = resolve_robot_deployment(Path(plugin_root or DEFAULT_PLUGIN_ROOT), robot_id)
        resources = dict(deployment.resources)
        # Resource-only prefixes, never hardware LD_LIBRARY_PATH or vendor startup.
        environment = deployment.resource_environment()
        if not initial_poses_file:
            candidate = deployment.manifest_path.parent / 'initial_poses.yaml'
            if candidate.is_file():
                initial_poses_file = str(candidate)
    else:
        resources = {
            'motion_params': share / 'config/motion_control.yaml',
            'sdk_config': share / 'config/robo_manip.test_humanoid.yaml',
            'channel_config': share / 'config/channels.yaml',
            'tool_config': share / 'config/tools.yaml',
            'urdf': share / 'urdf/test_humanoid.urdf',
        }
    overrides = document.get('resources', {})
    if set(overrides) - set(RESOURCE_KEYS) - {'hc_teleop_config'}:
        raise ValueError('Unknown simulation resource key')
    prefixes = environment.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    resources.update({key: resource_path(value, base, prefixes) for key, value in overrides.items()})
    resources = {key: str(value) for key, value in resources.items()
                 if key in (*RESOURCE_KEYS, 'hc_teleop_config')}
    motion = normalize_parameters(read_yaml(resources['motion_params'])['humanoid_motion_control']['ros__parameters'])
    sdk = read_yaml(resources['sdk_config'])
    channels = read_yaml(resources['channel_config'])['channels']
    model_joints, mimics = joint_metadata(resources['urdf'])
    names = list(dict.fromkeys(name for group in motion['joint_group_names']
                              for name in motion[f'groups.{group}']))
    if not names:
        raise ValueError('Motion configuration has no joints')
    joints = {}
    for name in names:
        if name not in model_joints or name in mimics:
            raise ValueError(f'Motion joint must be an independent URDF joint: {name}')
        joints[name] = model_joints[name]
    # Intersect all configured group limits, so overlapping groups agree on safety.
    for group in motion['joint_group_names']:
        group_names = motion[f'groups.{group}']
        low = motion.get(f'group_lower_limits.{group}', [joints[n].lower for n in group_names])
        high = motion.get(f'group_upper_limits.{group}', [joints[n].upper for n in group_names])
        if len(low) != len(group_names) or len(high) != len(group_names):
            raise ValueError(f'Joint/limit length mismatch: {group}')
        for name, lower, upper in zip(group_names, low, high):
            joint = joints[name]
            lower, upper = max(lower, joint.lower), min(upper, joint.upper)
            if lower > upper:
                raise ValueError(f'Conflicting limits for {name}')
            joints[name] = Joint(lower, upper, joint.speed, joint.kind)
    initial = {name: min(max(0.0, joint.lower), joint.upper) for name, joint in joints.items()}
    for group, values in sdk.get('execution', {}).get('initial_state', {}).get('joint_groups', {}).items():
        group_names = sdk['joint_groups'][group]
        if len(group_names) != len(values):
            raise ValueError(f'SDK initial state length mismatch: {group}')
        initial.update({name: _number(value, name) for name, value in zip(group_names, values) if name in joints})
    initial_pose = document.get('initial_pose', '')
    if initial_pose:
        if not initial_poses_file:
            raise ValueError('initial_pose requires an initial_poses_file or deployed robot')
        poses = read_yaml(resource_path(initial_poses_file, base))['initial_poses']
        pose = next((item for item in poses if item['id'] == initial_pose), None)
        if pose is None:
            raise ValueError(f'Unknown initial pose: {initial_pose}')
        by_channel = {channel['name']: channel for channel in channels}
        for target in pose['targets']:
            channel = by_channel[target['channel']]
            if channel['kind'] != 'move_j':
                raise ValueError('Simulation initial pose must use MoveJ channels')
            group_names = motion[f"groups.{channel['group']}"]
            if len(group_names) != len(target['positions_rad']):
                raise ValueError('Initial pose joint count mismatch')
            initial.update(zip(group_names, target['positions_rad']))
    overrides = document.get('initial_positions', {})
    if set(overrides) - set(joints):
        raise ValueError('initial_positions contains unknown motion joints')
    initial.update({name: _number(value, name) for name, value in overrides.items()})
    for name, value in initial.items():
        if not math.isfinite(value) or not joints[name].lower <= value <= joints[name].upper:
            raise ValueError(f'Initial position outside limits: {name}={value}')
    teleop = read_yaml(resources['hc_teleop_config']) if 'hc_teleop_config' in resources else None
    grippers = []
    for entry in (teleop or {}).get('grippers', []):
        if not entry.get('enabled', True):
            continue
        if entry.get('command_type', 'joint_state') != 'joint_state' or entry.get('feedback_type', 'joint_state') != 'joint_state':
            raise ValueError('Simulation currently supports JointState grippers only')
        grippers.append({
            'name': entry['joint_name'], 'command_topic': entry['command_topic'],
            'feedback_topic': entry['feedback_topic'],
            'lower': float(min(entry['closed_position'], entry['open_position'])),
            'upper': float(max(entry['closed_position'], entry['open_position'])),
            'initial': float(entry['open_position']),
            'speed': _number(entry['max_speed'], 'gripper max_speed', positive=True),
        })
    if len({g['name'] for g in grippers}) != len(grippers):
        raise ValueError('Duplicate logical gripper names')
    bindings = document.get('gripper_joints', {})
    for name, binding in bindings.items():
        target = binding['joint']
        if name not in {g['name'] for g in grippers} or target not in model_joints or target in mimics or target in joints:
            raise ValueError(f'Invalid gripper display binding: {name} -> {target}')
        scale = _number(binding.get('scale', 1.0), 'gripper scale')
        offset = _number(binding.get('offset', 0.0), 'gripper offset')
        gripper = next(g for g in grippers if g['name'] == name)
        for value in (gripper['lower'], gripper['upper']):
            if not model_joints[target].lower <= value * scale + offset <= model_joints[target].upper:
                raise ValueError(f'Gripper mapping exceeds URDF limits: {name}')
    if len({b['joint'] for b in bindings.values()}) != len(bindings):
        raise ValueError('Multiple grippers map to the same model joint')
    return {
        'robot_id': robot_id or 'test_humanoid', 'resources': resources, 'environment': environment,
        'motion': motion, 'channels': channels, 'joints': {n: asdict(j) for n, j in joints.items()},
        'initial': initial, 'teleop': teleop, 'grippers': grippers, 'gripper_joints': bindings,
        # Resolve display-only assets at display startup, so headless mode does
        # not depend on visual packages being installed.
        'visual_urdf': document.get('visual_urdf', ''), 'profile_directory': str(base),
        'speed_limit': _number(document.get('speed_limit', 3.0), 'speed_limit', positive=True),
        'watchdog_s': _number(document.get('watchdog_s', 0.1), 'watchdog_s', positive=True),
    }
