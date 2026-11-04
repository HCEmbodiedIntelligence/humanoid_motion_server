"""Trajectory generators and target store for motion interpolation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple
import numpy as np
import pinocchio as pin


@dataclass
class TargetTransform:
    translation: np.ndarray
    rotation: np.ndarray

    @classmethod
    def identity(cls):
        return cls(np.zeros(3), np.eye(3))

    def as_se3(self) -> pin.SE3:
        return pin.SE3(
            np.asarray(self.rotation, dtype=float),
            np.asarray(self.translation, dtype=float),
        )


class TargetStore:
    def __init__(self):
        self.transforms: Dict[Tuple[str, str], TargetTransform] = {}

    def update(self, values: Mapping[Tuple[str, str], TargetTransform]):
        self.transforms.update(values)

    def set_target(self, root: str, frame: str, transform: TargetTransform):
        self.transforms[(root, frame)] = transform

    def relative_target(self, root: str, frame: str) -> Optional[pin.SE3]:
        direct = self.transforms.get((root, frame))
        if direct is not None:
            return direct.as_se3()
        root_transform = self.transforms.get(("", root)) or self.transforms.get(
            ("world", root)
        )
        child_transform = self.transforms.get(("", frame)) or self.transforms.get(
            ("world", frame)
        )
        if root_transform is None or child_transform is None:
            return None
        return root_transform.as_se3().inverse() * child_transform.as_se3()


def quintic_blend(t: float, total_time: float) -> Tuple[float, float]:
    """Quintic polynomial: s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0.
    Returns (s, ds).
    """
    if total_time <= 0.0:
        return 1.0, 0.0
    tau = max(0.0, min(1.0, t / total_time))
    tau3 = tau * tau * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau
    s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    ds = (30.0 * tau * tau - 60.0 * tau3 + 30.0 * tau4) / total_time
    return s, ds


class JointTrajectory:
    def __init__(
        self,
        start_positions: np.ndarray,
        target_positions: np.ndarray,
        max_velocity: float = 0.6,
        max_acceleration: float = 2.4,
        scale: float = 1.0,
        min_duration: float = 0.5,
    ):
        self.start = np.asarray(start_positions, dtype=float).copy()
        self.target = np.asarray(target_positions, dtype=float).copy()
        scale = max(0.05, min(1.0, float(scale)))
        eff_vel = max(0.05, max_velocity * scale)
        delta = np.abs(self.target - self.start)
        max_delta = float(np.max(delta)) if delta.size > 0 else 0.0
        self.duration = max(min_duration, max_delta / eff_vel)
        self.elapsed = 0.0

    def step(self, dt: float) -> Tuple[np.ndarray, float, bool]:
        self.elapsed += float(dt)
        s, _ = quintic_blend(self.elapsed, self.duration)
        q = self.start + s * (self.target - self.start)
        progress = max(0.0, min(1.0, self.elapsed / self.duration))
        done = self.elapsed >= self.duration
        return q, progress, done


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions (x, y, z, w)."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        res = q0 + t * (q1 - q0)
        return res / np.linalg.norm(res)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * q0 + s1 * q1


class CartesianTrajectory:
    def __init__(
        self,
        start_pose: pin.SE3,
        target_pose: pin.SE3,
        max_linear_velocity: float = 0.1,
        max_angular_velocity: float = 0.5,
        scale: float = 1.0,
        min_duration: float = 0.5,
    ):
        self.start_pose = start_pose.copy()
        self.target_pose = target_pose.copy()
        scale = max(0.05, min(1.0, float(scale)))
        eff_lin_vel = max(0.01, max_linear_velocity * scale)
        eff_ang_vel = max(0.05, max_angular_velocity * scale)
        lin_dist = float(np.linalg.norm(self.target_pose.translation - self.start_pose.translation))
        rel_rot = self.start_pose.rotation.T @ self.target_pose.rotation
        ang_dist = float(np.linalg.norm(pin.log3(rel_rot)))
        time_lin = lin_dist / eff_lin_vel
        time_ang = ang_dist / eff_ang_vel
        self.duration = max(min_duration, max(time_lin, time_ang))
        self.elapsed = 0.0
        self.q0 = pin.Quaternion(self.start_pose.rotation).coeffs()
        self.q1 = pin.Quaternion(self.target_pose.rotation).coeffs()

    def step(self, dt: float) -> Tuple[pin.SE3, float, bool]:
        self.elapsed += float(dt)
        s, _ = quintic_blend(self.elapsed, self.duration)
        trans = self.start_pose.translation + s * (self.target_pose.translation - self.start_pose.translation)
        quat = slerp(self.q0, self.q1, s)
        rot = pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).toRotationMatrix()
        progress = max(0.0, min(1.0, self.elapsed / self.duration))
        done = self.elapsed >= self.duration
        return pin.SE3(rot, trans), progress, done
