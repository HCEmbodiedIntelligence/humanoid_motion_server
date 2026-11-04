"""Task definitions for differential inverse kinematics."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Protocol, Sequence

import numpy as np
import pinocchio as pin


class TargetProvider(Protocol):
    def relative_target(self, root: str, frame: str): ...


@dataclass
class LinearizedTask:
    jacobian: np.ndarray
    error: np.ndarray
    weight: np.ndarray
    name: str = "task"

    def weighted_rows(self) -> tuple[np.ndarray, np.ndarray]:
        weights = np.sqrt(np.asarray(self.weight, dtype=float).reshape(-1))
        if self.jacobian.shape[0] != weights.size or self.error.size != weights.size:
            raise ValueError(f"{self.name}: inconsistent task dimensions")
        return self.jacobian * weights[:, None], self.error.reshape(-1) * weights


class Task(ABC):
    def __init__(self, gain: float = 1.0):
        self.gain = float(gain)

    @abstractmethod
    def linearize(
        self, interface, q: np.ndarray, targets: TargetProvider
    ) -> LinearizedTask | None:
        raise NotImplementedError


class FrameTask(Task):
    def __init__(
        self,
        root: str,
        frame: str,
        position_cost: float = 5.0,
        orientation_cost: float = 1.0,
        gain: float = 1.0,
    ):
        super().__init__(gain)
        self.root = root
        self.frame = frame
        self.position_cost = float(position_cost)
        self.orientation_cost = float(orientation_cost)

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        target = targets.relative_target(self.root, self.frame)
        if target is None:
            return None
        current = interface.relative_pose(self.root, self.frame, q)
        jacobian = interface.relative_jacobian(self.root, self.frame, q)
        position_error = np.asarray(
            target.translation - current.translation, dtype=float
        )
        rotation_error_local = pin.log3(current.rotation.T @ target.rotation)
        rotation_error = current.rotation @ np.asarray(
            rotation_error_local, dtype=float
        )
        error = self.gain * np.concatenate([position_error, rotation_error])
        weight = np.array(
            [self.position_cost] * 3 + [self.orientation_cost] * 3, dtype=float
        )
        return LinearizedTask(
            jacobian, weight=weight, error=error,
            name=f"frame:{self.root}->{self.frame}"
        )


class JointTask(Task):
    def __init__(
        self, joint_names: Sequence[str], cost: float = 0.01, gain: float = 1.0,
        target=None
    ):
        super().__init__(gain)
        self.joint_names = list(joint_names)
        self.cost = float(cost)
        self.target = None
        if target is not None:
            self.set_target(target)

    def set_target(self, values) -> None:
        if isinstance(values, dict):
            values = [values[name] for name in self.joint_names if name in values]
        values = np.asarray(values, dtype=float).reshape(-1)
        if values.size != len(self.joint_names):
            raise ValueError("joint target size mismatch")
        self.target = values

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        if self.target is None:
            return None
        name_to_column = {
            name: index for index, name in enumerate(interface.free_joints)
        }
        rows, errors = [], []
        for name, desired in zip(self.joint_names, self.target):
            if name not in name_to_column:
                continue
            column = name_to_column[name]
            row = np.zeros(interface.nv_free)
            row[column] = 1.0
            current = q[interface.joints[column].q_index]
            rows.append(row)
            errors.append(self.gain * (float(desired) - float(current)))
        if not rows:
            return None
        error = np.asarray(errors, dtype=float)
        return LinearizedTask(
            np.vstack(rows), error, np.full(error.size, self.cost), "joint-posture"
        )


_AXIS = {"X": 0, "Y": 1, "Z": 2}


class AxisTask(Task):
    def __init__(
        self, root: str, frame: str, axis: str, cost: float, gain: float = 1.0
    ):
        super().__init__(gain)
        axis = str(axis).upper()
        if axis not in _AXIS:
            raise ValueError(f"axis must be X/Y/Z, got {axis!r}")
        self.root = root
        self.frame = frame
        self.axis = axis
        self.cost = float(cost)

    def linearize(self, interface, q: np.ndarray, targets) -> LinearizedTask | None:
        target = targets.relative_target(self.root, self.frame)
        if target is None:
            return None
        current = interface.relative_pose(self.root, self.frame, q)
        jacobian = interface.relative_jacobian(self.root, self.frame, q)
        index = _AXIS[self.axis]
        current_axis = np.asarray(current.rotation[:, index], dtype=float)
        target_axis = np.asarray(target.rotation[:, index], dtype=float)
        error = self.gain * np.cross(current_axis, target_axis)
        return LinearizedTask(
            jacobian[3:, :], error, np.full(3, self.cost, dtype=float),
            f"axis:{self.root}->{self.frame}:{self.axis}"
        )
