"""ROS adapter for the offline arm and gripper plant."""

import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .configuration import read_yaml
from .model import Joint, expand_mimics, joint_metadata
from .plant import KinematicPlant


def state_message(positions, velocities, stamp):
    message = JointState()
    message.header.stamp = stamp
    message.name = list(positions)
    message.position = [float(positions[n]) for n in message.name]
    message.velocity = [float(velocities.get(n, 0.0)) for n in message.name]
    message.effort = [0.0] * len(message.name)
    return message


class SimulationNode(Node):
    def __init__(self, configuration):
        super().__init__('humanoid_simulation')
        self.configuration = configuration
        self.arm = KinematicPlant({n: Joint(**j) for n, j in configuration['joints'].items()},
                                  configuration['initial'], configuration['watchdog_s'],
                                  configuration['speed_limit'])
        grippers = configuration['grippers']
        self.gripper = KinematicPlant(
            {g['name']: Joint(g['lower'], g['upper'], g['speed'], 'prismatic') for g in grippers},
            {g['name']: g['initial'] for g in grippers}, watchdog=0.25,
            speed_limit=max([g['speed'] for g in grippers] or [1.0]))
        self.model_joints, self.mimics = joint_metadata(configuration['resources']['urdf'])
        self.display_positions = {n: min(max(0.0, j.lower), j.upper)
                                  for n, j in self.model_joints.items() if n not in self.mimics}
        motion = configuration['motion']
        self.arm_publisher = self.create_publisher(JointState, motion['joint_state_endpoint'], 1)
        # Queue covers independent partial arm packets. Callbacks only overwrite
        # targets; the simulation timer never executes a FIFO of old commands.
        self.subscriptions_owned = [self.create_subscription(
            JointState, motion['joint_command_endpoint'],
            lambda message: self.receive(self.arm, set(self.arm.joints), message), 10)]
        self.gripper_publishers = []
        for topic in sorted({g['command_topic'] for g in grippers}):
            names = {g['name'] for g in grippers if g['command_topic'] == topic}
            self.subscriptions_owned.append(self.create_subscription(
                JointState, topic, lambda message, names=names: self.receive(self.gripper, names, message), 10))
        for topic in sorted({g['feedback_topic'] for g in grippers}):
            names = [g['name'] for g in grippers if g['feedback_topic'] == topic]
            self.gripper_publishers.append((self.create_publisher(JointState, topic, 1), names))
        self.display_publisher = self.create_publisher(JointState, '/simulation/joint_states', 1)
        self.status_publisher = self.create_publisher(String, '/simulation/status', 1)
        self.rejected_commands = 0
        self.samples = 0
        self.create_timer(0.01, self.tick)
        self.create_timer(1.0, self.status)
        self.get_logger().info(
            f"Kinematic simulation: {configuration['robot_id']}, {len(self.arm.joints)} arm/body joints, "
            f'{len(grippers)} grippers. Hardware is not part of this launch.')

    def receive(self, plant, allowed, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (not set(message.name).issubset(allowed) or not plant.command(
                message.name, message.position, stamp,
                self.get_clock().now().nanoseconds, time.monotonic())):
            self.rejected_commands += 1

    def tick(self):
        now, stamp = time.monotonic(), self.get_clock().now()
        for plant in (self.arm, self.gripper):
            plant.observe_clock(stamp.nanoseconds)
            plant.step(now)
        self.arm_publisher.publish(state_message(self.arm.positions, self.arm.velocities, stamp.to_msg()))
        for publisher, names in self.gripper_publishers:
            publisher.publish(state_message({n: self.gripper.positions[n] for n in names},
                                             self.gripper.velocities, stamp.to_msg()))
        self.display_positions.update(self.arm.positions)
        for name, binding in self.configuration['gripper_joints'].items():
            self.display_positions[binding['joint']] = (
                self.gripper.positions[name] * binding.get('scale', 1.0) + binding.get('offset', 0.0))
        self.display_publisher.publish(state_message(
            expand_mimics(self.display_positions, self.mimics), {}, stamp.to_msg()))
        self.samples += 1

    def status(self):
        self.status_publisher.publish(String(data=json.dumps({
            'mode': 'kinematic_simulation', 'robot_id': self.configuration['robot_id'],
            'samples': self.samples, 'rejected_commands': self.rejected_commands,
            'watchdog_s': self.arm.watchdog,
        })))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = SimulationNode(read_yaml(args.snapshot))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
