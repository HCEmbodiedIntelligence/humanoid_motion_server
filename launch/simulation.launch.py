"""Generic offline motion/teleop simulation with optional Meshcat display."""
from humanoid_motion_server.simulation.launching import description


def generate_launch_description():
    return description()
