from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('humanoid_motion_server'))
    driver_share = Path(get_package_share_directory('humanoid_driver_runtime'))
    config = share / 'config'
    urdf = share / 'urdf' / 'test_humanoid.urdf'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(config / 'motion_control.yaml')),
        DeclareLaunchArgument(
            'driver_params_file',
            default_value=str(driver_share / 'config' / 'mock_driver.yaml')),
        DeclareLaunchArgument(
            'sdk_config_file',
            default_value=str(config / 'robo_manip.test_humanoid.yaml'),
            description='Original-SDK YAML matching the deployed robot model.'),
        DeclareLaunchArgument('urdf_file', default_value=str(urdf)),
        Node(
            package='humanoid_driver_runtime',
            executable='humanoid_driver_runtime_node',
            name='humanoid_driver_runtime',
            output='screen',
            parameters=[LaunchConfiguration('driver_params_file')],
        ),
        Node(
            package='humanoid_motion_server',
            executable='humanoid_motion_control_node',
            name='humanoid_motion_control',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'channel_config_file': str(config / 'channels.yaml'),
                'sdk_config_file': LaunchConfiguration('sdk_config_file'),
                'tool_config_file': str(config / 'tools.yaml'),
                'urdf_file': LaunchConfiguration('urdf_file'),
            }],
        ),
    ])
