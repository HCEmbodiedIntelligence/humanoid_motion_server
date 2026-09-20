"""Latest-feedback Meshcat rendering in its own, rate-limited process."""

import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .configuration import read_yaml


def connect_viewer(url):
    import meshcat
    from meshcat.visualizer import ViewerWindow
    import zmq

    class BoundedWindow(ViewerWindow):
        def connect_zmq(self):
            previous = getattr(self, 'zmq_socket', None)
            if previous is not None:
                previous.close(linger=0)
            self.zmq_socket = self.context.socket(zmq.REQ)
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, 2000)
            self.zmq_socket.setsockopt(zmq.SNDTIMEO, 2000)
            self.zmq_socket.setsockopt(zmq.LINGER, 0)
            self.zmq_socket.connect(self.zmq_url)

    # The server starts concurrently. Probe without an unbounded constructor wait.
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and rclpy.ok():
        probe = zmq.Context.instance().socket(zmq.REQ)
        probe.setsockopt(zmq.LINGER, 0)
        probe.connect(url)
        probe.send(b'url')
        ready = probe.poll(200)
        if ready:
            probe.recv()
        probe.close()
        if ready:
            return meshcat.Visualizer(window=BoundedWindow(url, False, []))
    raise RuntimeError('Meshcat server did not become ready within 15 seconds')


class MeshcatNode(Node):
    def __init__(self, configuration, zmq_url, rate):
        super().__init__('humanoid_meshcat')
        import meshcat.geometry as geometry
        import pinocchio as pin
        from pinocchio.visualize import MeshcatVisualizer
        self.pin, self.geometry = pin, geometry
        self.viewer = connect_viewer(zmq_url)
        model_path = configuration['display_urdf']
        self.model = pin.buildModelFromUrdf(model_path)
        visuals = pin.buildGeomFromUrdf(self.model, model_path, pin.GeometryType.VISUAL)
        self.visualizer = MeshcatVisualizer(self.model, pin.GeometryModel(), visuals)
        self.visualizer.initViewer(viewer=self.viewer)
        self.visualizer.loadViewerModel(rootNodeName='simulation')
        self.visualizer.displayCollisions(False)
        self.q = pin.neutral(self.model)
        self.joints = {self.model.names[i]: self.model.joints[i]
                       for i in range(1, self.model.njoints)}
        self.skeleton = []
        if not visuals.ngeoms:
            for joint in ET.parse(model_path).getroot().findall('joint'):
                self.skeleton.append((self.model.getFrameId(joint.find('parent').get('link')),
                                      self.model.getFrameId(joint.find('child').get('link'))))
            self.get_logger().info('URDF has no visuals: rendering its kinematic skeleton')
        self.latest = None
        self.last_stamp = 0
        self.last_clock = 0
        self.rendered_stamp = 0
        self.frames = 0
        self.fresh = None
        self.viewer['simulation'].set_property('visible', False)
        self.subscription = self.create_subscription(
            JointState, '/simulation/joint_states', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_publisher = self.create_publisher(String, '/simulation/viewer_status', 1)
        self.create_timer(1.0 / rate, self.render)
        self.create_timer(1.0, self.status)
        self.get_logger().info(f'Meshcat ready: {self.viewer.url()} ({rate:g} Hz maximum)')
        self.centered = False

    def receive(self, message):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.last_stamp = self.rendered_stamp = 0
            self.latest = None
        self.last_clock = now
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (stamp <= self.last_stamp or not -0.01 <= (now - stamp) / 1e9 < 0.25 or
                len(message.name) != len(message.position) or len(set(message.name)) != len(message.name) or
                not set(self.joints).issubset(message.name) or
                not all(math.isfinite(value) for value in message.position)):
            return
        self.latest, self.last_stamp = message, stamp

    def render(self):
        fresh = self.latest is not None and -0.01 <= (self.get_clock().now().nanoseconds - self.last_stamp) / 1e9 < 0.25
        if fresh != self.fresh:
            self.viewer['simulation'].set_property('visible', fresh)
            self.fresh = fresh
            if not fresh:
                self.get_logger().warning('Simulation feedback is stale; hiding the robot until fresh feedback returns')
        if not fresh or self.rendered_stamp == self.last_stamp:
            return
        for name, value in zip(self.latest.name, self.latest.position):
            joint = self.joints.get(name)
            if joint is None:
                continue
            if joint.nq == 1:
                self.q[joint.idx_q] = value
            elif joint.nq == 2 and joint.nv == 1:
                self.q[joint.idx_q:joint.idx_q + 2] = [math.cos(value), math.sin(value)]
            else:
                raise ValueError(f'Unsupported display joint: {name}')
        self.visualizer.display(self.q)
        self.pin.updateFramePlacements(self.model, self.visualizer.data)
        if self.skeleton:
            points = np.array([self.visualizer.data.oMf[frame].translation
                               for pair in self.skeleton for frame in pair]).T
            self.viewer['simulation/skeleton'].set_object(
                self.geometry.LineSegments(self.geometry.PointsGeometry(points),
                                           self.geometry.LineBasicMaterial(color=0x36A9E1, linewidth=5)))
        if not self.centered:
            points = np.array([frame.translation for frame in self.visualizer.data.oMf])
            center = (points.min(axis=0) + points.max(axis=0)) / 2
            radius = max(float(np.ptp(points, axis=0).max()), 0.6)
            # Meshcat 0.3.2 supports camera scene properties, but not the newer
            # set_cam_target/set_cam_pos APIs on its development branch.
            transform = np.eye(4)
            transform[:3, 3] = center
            self.viewer['/Cameras/default'].set_transform(transform)
            self.viewer['/Cameras/default/rotated/<object>'].set_property(
                'position', (radius * np.array([1.4, 0.9, -1.4])).tolist())
            self.centered = True
        self.rendered_stamp = self.last_stamp
        self.frames += 1

    def status(self):
        self.status_publisher.publish(String(data=json.dumps({
            'feedback_fresh': bool(self.fresh), 'frames': self.frames,
            'source_stamp_ns': self.rendered_stamp, 'url': self.viewer.url(),
            'visual_geometries': int(self.visualizer.visual_model.ngeoms),
        })))

    def destroy_node(self):
        self.viewer.window.zmq_socket.close(linger=0)
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--zmq-url', required=True)
    parser.add_argument('--rate', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.rate) or not 1 <= args.rate <= 60:
        parser.error('--rate must be between 1 and 60 Hz')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = MeshcatNode(read_yaml(args.snapshot), args.zmq_url, args.rate)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
