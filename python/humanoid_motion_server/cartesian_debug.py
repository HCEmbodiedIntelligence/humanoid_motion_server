"""Pose math shared by keyboard control and simulation diagnostics (ROS xyzw)."""

import copy
import math

import numpy as np


def quaternion(pose):
    q = pose.orientation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = math.hypot(*values)
    if not np.isfinite(values).all() or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('四元数无效')
    return values / norm


def rotation(xyzw):
    x, y, z, w = xyzw
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform(message):
    if not message.header.frame_id.strip():
        raise ValueError('位姿缺少参考坐标系')
    p = message.pose.position
    position = np.array([p.x, p.y, p.z])
    if not np.isfinite(position).all():
        raise ValueError('位置数值无效')
    result = np.eye(4)
    result[:3, :3] = rotation(quaternion(message.pose))
    result[:3, 3] = position
    return result


def multiply(left, right):
    a, b = np.asarray(left), np.asarray(right)
    return np.r_[a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3]),
                 a[3]*b[3] - np.dot(a[:3], b[:3])]


def step_pose(message, axis, amount, *, angular=False, local=False):
    """Translate or rotate about a fixed reference axis or current target axis."""
    matrix = transform(message)
    result = copy.deepcopy(message)
    unit = np.eye(3)[axis]
    if angular:
        delta = np.r_[unit * math.sin(amount / 2), math.cos(amount / 2)]
        initial = quaternion(message.pose)
        value = multiply(initial, delta) if local else multiply(delta, initial)
        value /= math.hypot(*value)
        q = result.pose.orientation
        q.x, q.y, q.z, q.w = map(float, value)
    else:
        delta = (matrix[:3, :3] @ unit if local else unit) * amount
        p = result.pose.position
        p.x += float(delta[0])
        p.y += float(delta[1])
        p.z += float(delta[2])
    transform(result)  # Reject arithmetic overflow before anything is published.
    return result


def pose_error(actual, target):
    """Position distance (m) and shortest rotation distance (rad), in one frame."""
    position = float(np.linalg.norm(target[:3, 3] - actual[:3, 3]))
    relative = actual[:3, :3].T @ target[:3, :3]
    angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    return position, angle


def rpy_degrees(message):
    matrix = transform(message)[:3, :3]
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(matrix[2, 1], matrix[2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll, yaw = 0.0, math.atan2(-matrix[0, 1], matrix[1, 1])
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def fresh(message, now_ns, max_age=0.1):
    return message is not None and stamp_ns(message) > 0 and -0.1 <= (now_ns-stamp_ns(message))/1e9 < max_age
