"""Integration test for humanoid_motion_server with X1 robot configuration."""
import os
import time
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from humanoid_motion_interfaces.action import MoveJ
from humanoid_motion_server.motion_node import HumanoidMotionControlNode


def test_motion_server_node_x1_params():
    params_file = "/tmp/test_x1_gen/motion.yaml"
    assert os.path.isfile(params_file), f"Expected {params_file} to exist"

    rclpy.init(args=["--ros-args", "--params-file", params_file])
    try:
        test_node = Node("test_motion_client")
        received_fk = []

        sub = test_node.create_subscription(
            PoseStamped,
            "/observation/motion/left_arm/ee_pose",
            lambda msg: received_fk.append(msg),
            1,
        )

        joint_pub = test_node.create_publisher(JointState, "/internal/robot/joint_states", 1)

        server_node = HumanoidMotionControlNode()

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(server_node)

        names = [
            'Joint1_L', 'Joint2_L', 'Joint3_L', 'Joint4_L', 'Joint5_L', 'Joint6_L', 'Joint7_L',
            'Joint1_R', 'Joint2_R', 'Joint3_R', 'Joint4_R', 'Joint5_R', 'Joint6_R', 'Joint7_R'
        ]
        js = JointState()
        js.name = names
        js.position = [0.0] * len(names)

        for _ in range(15):
            js.header.stamp = test_node.get_clock().now().to_msg()
            joint_pub.publish(js)
            executor.spin_once(timeout_sec=0.05)

        assert len(received_fk) > 0, "Expected FK pose to be published on /observation/motion/left_arm/ee_pose"
        pose = received_fk[-1]
        assert pose.header.frame_id == "zhi_Link"
        assert abs(pose.pose.orientation.w) > 0.0 or abs(pose.pose.orientation.x) > 0.0

        server_node.destroy_node()
        test_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_motion_server_servo_p():
    params_file = "/tmp/test_x1_gen/motion.yaml"
    rclpy.init(args=["--ros-args", "--params-file", params_file])
    try:
        test_node = Node("test_servo_client")
        commands_received = []

        cmd_sub = test_node.create_subscription(
            JointState,
            "/control/motion/joint_targets",
            lambda msg: commands_received.append(msg),
            1,
        )

        joint_pub = test_node.create_publisher(JointState, "/internal/robot/joint_states", 1)
        servo_pub = test_node.create_publisher(PoseStamped, "/control/motion/left_arm/servo_p", 1)

        server_node = HumanoidMotionControlNode()

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(server_node)

        names = [
            'Joint1_L', 'Joint2_L', 'Joint3_L', 'Joint4_L', 'Joint5_L', 'Joint6_L', 'Joint7_L',
            'Joint1_R', 'Joint2_R', 'Joint3_R', 'Joint4_R', 'Joint5_R', 'Joint6_R', 'Joint7_R'
        ]
        js = JointState()
        js.name = names
        js.position = [0.0] * len(names)

        # Feed joint feedback
        for _ in range(10):
            js.header.stamp = test_node.get_clock().now().to_msg()
            joint_pub.publish(js)
            executor.spin_once(timeout_sec=0.02)

        # Send target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "zhi_Link"
        target_pose.pose.position.x = 0.25
        target_pose.pose.position.y = 0.35
        target_pose.pose.position.z = 0.15
        target_pose.pose.orientation.w = 1.0

        initial_cmd_count = len(commands_received)
        for _ in range(25):
            now_msg = test_node.get_clock().now().to_msg()
            js.header.stamp = now_msg
            target_pose.header.stamp = now_msg
            joint_pub.publish(js)
            servo_pub.publish(target_pose)
            executor.spin_once(timeout_sec=0.02)

        assert len(commands_received) > initial_cmd_count, "Expected commands to be published"
        latest_cmd = commands_received[-1]
        assert len(latest_cmd.position) == 14

        server_node.destroy_node()
        test_node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
