from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('humanoid_motion_server'))
    driver_share = Path(get_package_share_directory('humanoid_driver_runtime'))
    config = share / 'config'
    return LaunchDescription([
        Node(
            package='humanoid_driver_runtime',
            executable='humanoid_driver_runtime_node',
            name='humanoid_driver_runtime',
            output='screen',
            parameters=[str(driver_share / 'config' / 'mock_driver.yaml')],
        ),
        Node(
            package='humanoid_motion_server',
            executable='humanoid_motion_control_node',
            name='humanoid_motion_control',
            output='screen',
            parameters=[str(config / 'motion_control.yaml'), {
                'channel_config_file': str(config / 'channels.yaml'),
                'sdk_config_file': str(config / 'robo_manip.test_humanoid.yaml'),
                'tool_config_file': str(config / 'tools.yaml'),
                'urdf_file': str(share / 'urdf' / 'test_humanoid.urdf'),
            }],
        ),
    ])
