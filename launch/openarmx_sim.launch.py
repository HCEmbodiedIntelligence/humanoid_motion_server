"""Example profile for the deployed OpenArmX robot; no hardware bringup."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from humanoid_motion_server.simulation.launching import description


def generate_launch_description():
    share = Path(get_package_share_directory('humanoid_motion_server'))
    return description(str(share / 'config/simulation/openarmx.yaml'))
