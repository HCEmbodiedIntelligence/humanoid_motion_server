"""The simulation launch graph is intentionally independent of hardware bringup."""

from copy import deepcopy
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from .configuration import load_configuration
from .home import HOME_POSE_ID, home_targets
from .model import resource_path, visual_model


_TEMPORARY_DIRECTORIES = []


def _boolean(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError('Boolean launch arguments must be true or false')
    return value.lower() == 'true'


def _launch(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('humanoid_motion_server'))
    configuration = load_configuration(share, argument('profile'), argument('robot_id'), argument('plugin_root'))
    domain = int(argument('domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('domain_id must be between 0 and 232')
    meshcat, teleop = _boolean(argument('meshcat')), _boolean(argument('start_teleop'))
    interpreter = argument('simulation_python')
    # Fail before launching any part of the stack when display dependencies are missing.
    if meshcat:
        check = subprocess.run([interpreter, '-c', 'import meshcat; from pinocchio.visualize import MeshcatVisualizer'],
                               capture_output=True, text=True, timeout=20)
        if check.returncode:
            raise RuntimeError('Meshcat dependencies unavailable. Run scripts/setup_simulation.sh, then activate '
                               f'the venv or set simulation_python.\n{check.stderr}')
    if teleop and configuration['teleop'] is None:
        raise ValueError('start_teleop requires a profile/deployed robot with hc_teleop_config')
    port = int(argument('meshcat_port'))
    if not 1 <= port <= 65535:
        raise ValueError('meshcat_port must be between 1 and 65535')
    rate = float(argument('viewer_rate'))
    if not 1 <= rate <= 60:
        raise ValueError('viewer_rate must be between 1 and 60 Hz')
    directory = tempfile.TemporaryDirectory(prefix='humanoid-simulation-')
    _TEMPORARY_DIRECTORIES.append(directory)  # Lifetime of the launch process.
    temporary = Path(directory.name)
    if meshcat:
        display = temporary / 'display.urdf'
        prefixes = configuration['environment'].get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        overlay = (resource_path(configuration['visual_urdf'], configuration['profile_directory'], prefixes)
                   if configuration['visual_urdf'] else None)
        display.write_text(visual_model(configuration['resources']['urdf'], overlay, prefixes), encoding='utf-8')
        configuration['display_urdf'] = str(display)
    snapshot = temporary / 'simulation.yaml'
    snapshot.write_text(yaml.safe_dump(configuration), encoding='utf-8')
    motion = dict(configuration['motion'])
    motion.update({
        'channel_config_file': configuration['resources']['channel_config'],
        'sdk_config_file': configuration['resources']['sdk_config'],
        'tool_config_file': configuration['resources']['tool_config'],
        'urdf_file': configuration['resources']['urdf'],
    })
    motion_path = temporary / 'motion.yaml'
    motion_path.write_text(yaml.safe_dump({'humanoid_motion_control': {'ros__parameters': motion}}), encoding='utf-8')
    environment = {**configuration['environment'], 'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1'}
    executables = Path(get_package_prefix('humanoid_motion_server')) / 'lib/humanoid_motion_server'
    actions = [LogInfo(msg=f"Offline simulation: {configuration['robot_id']}; ROS_DOMAIN_ID={domain}; "
                            '100 Hz plant; hardware/cameras are not launched.'),
               Node(package='humanoid_motion_server', executable='simulation_node',
                    prefix=[interpreter], arguments=['--snapshot', str(snapshot)],
                    additional_env=environment, output='screen',
                    on_exit=Shutdown(reason='simulation plant exited')),
               Node(package='humanoid_motion_server', executable='humanoid_motion_control_node',
                    name='humanoid_motion_control', parameters=[str(motion_path)],
                    additional_env=environment, output='screen',
                    on_exit=Shutdown(reason='motion server exited'))]
    if meshcat:
        zmq_url = f'ipc://{temporary}/meshcat.sock'
        actions.extend([
            ExecuteProcess(cmd=[interpreter, str(executables / 'meshcat_server'), '--zmq-url', zmq_url,
                                '--host', argument('meshcat_host'), '--port', str(port)],
                           additional_env=environment, output='screen',
                           on_exit=Shutdown(reason='Meshcat server exited')),
            Node(package='humanoid_motion_server', executable='meshcat_viewer', prefix=[interpreter],
                 arguments=['--snapshot', str(snapshot), '--zmq-url', zmq_url, '--rate', str(rate)],
                 additional_env=environment, output='screen',
                 on_exit=Shutdown(reason='Meshcat viewer exited'))])
    if teleop:
        receiver = deepcopy(configuration['teleop'])
        receiver['control']['enabled_on_start'] = False
        if argument('pose_port'):
            receiver['input']['pose_port'] = int(argument('pose_port'))
        if argument('discovery_port'):
            receiver['input']['discovery_port'] = int(argument('discovery_port'))
        # Homing uses the actual simulation startup pose, including profile
        # overrides. Recording/posture workflows still require the manager.
        home_targets(configuration)  # Validate before launching any process.
        receiver.setdefault('actions', {}).update(home_pose_id=HOME_POSE_ID, home_gesture_enabled=True,
                                                 recording_buttons_enabled=False,
                                                 mark_gesture_enabled=False, posture_pose_id='')
        if receiver.get('chassis'):
            receiver['chassis']['enabled'] = False
        receiver_path = temporary / 'teleop.yaml'
        receiver_path.write_text(yaml.safe_dump(receiver), encoding='utf-8')
        from hc_teleop_recv.config import load_config
        parsed = load_config(receiver_path)
        configuration['simulation_home'] = {
            'configuration_sha256': hashlib.sha256(receiver_path.read_bytes()).hexdigest(),
            'robot_id': parsed.robot_id, 'stop_topic': parsed.emergency_stop_topic,
        }
        snapshot.write_text(yaml.safe_dump(configuration), encoding='utf-8')
        actions.append(Node(package='humanoid_motion_server', executable='simulation_home',
                            prefix=[interpreter], arguments=['--snapshot', str(snapshot)],
                            additional_env=environment, output='screen',
                            on_exit=Shutdown(reason='simulation home handler exited')))
        actions.append(Node(package='hc_teleop_recv', executable='hc_teleop_recv_node',
                            parameters=[{'config_file': str(receiver_path), 'use_sim_time': False}],
                            additional_env=environment, output='screen',
                            on_exit=Shutdown(reason='teleop frontend exited')))
    return actions


def description(default_profile=''):
    defaults = {
        'profile': (default_profile, 'Simulation YAML; empty uses the built-in humanoid'),
        'robot_id': ('', 'Optional deployed robot; overrides profile robot_id'),
        'plugin_root': ('', 'Deployment root; empty uses the manager default'),
        'domain_id': ('199', 'Dedicated offline ROS domain; do not use the physical robot domain'),
        'meshcat': ('true', 'Start Meshcat display; false runs headless'),
        'meshcat_host': ('127.0.0.1', 'HTTP bind address'),
        'meshcat_port': ('7000', 'Meshcat HTTP port'),
        'viewer_rate': ('30', 'Maximum render rate, 1 to 60 Hz'),
        'simulation_python': (os.environ.get('HUMANOID_SIMULATION_PYTHON') or shutil.which('python3') or '/usr/bin/python3',
                              'Python interpreter with Meshcat and ROS Pinocchio'),
        'start_teleop': ('false', 'Enable PICO/frontend input using the robot teleop configuration'),
        'pose_port': ('', 'Override UDP pose port; blank preserves robot configuration'),
        'discovery_port': ('', 'Override UDP discovery port; blank preserves robot configuration'),
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=help_text)
                              for name, (default, help_text) in defaults.items()] + [OpaqueFunction(function=_launch)])
