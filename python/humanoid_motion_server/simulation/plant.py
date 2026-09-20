"""Deterministic, velocity-limited plant with per-joint latest targets.

This is an ideal position actuator, not a rigid-body/contact simulation.
ROS timestamps validate source age; monotonic time advances the plant/watchdog.
"""

import math


class KinematicPlant:
    def __init__(self, joints, initial, watchdog=0.1, speed_limit=3.0):
        if not math.isfinite(watchdog) or watchdog <= 0 or not math.isfinite(speed_limit) or speed_limit <= 0:
            raise ValueError('watchdog and speed_limit must be positive finite numbers')
        self.joints = dict(joints)
        self.positions = {name: float(initial[name]) for name in joints}
        for name, value in self.positions.items():
            joint = joints[name]
            if not math.isfinite(value) or not joint.lower <= value <= joint.upper:
                raise ValueError(f'Initial position outside limits: {name}={value}')
        self.targets = dict(self.positions)
        self.velocities = dict.fromkeys(joints, 0.0)
        self.stamps = dict.fromkeys(joints, 0)
        self.deadlines = dict.fromkeys(joints, -math.inf)
        self.watchdog = watchdog
        self.speed_limit = speed_limit
        self.last_step = None
        self.last_ros_ns = None

    def observe_clock(self, ros_ns):
        if self.last_ros_ns is not None and ros_ns < self.last_ros_ns:
            self.hold()
            self.stamps = dict.fromkeys(self.joints, 0)
        self.last_ros_ns = ros_ns

    def command(self, names, positions, stamp_ns, ros_ns, monotonic_now):
        self.observe_clock(ros_ns)
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            return False
        age = (ros_ns - stamp_ns) / 1e9
        if stamp_ns <= 0 or age < -0.01 or age >= self.watchdog:
            return False
        for name, value in zip(names, positions):
            joint = self.joints.get(name)
            if (joint is None or not math.isfinite(value) or not joint.lower <= value <= joint.upper
                    or stamp_ns <= self.stamps[name]):
                return False
        for name, value in zip(names, positions):
            self.targets[name] = value
            self.stamps[name] = stamp_ns
            self.deadlines[name] = monotonic_now + self.watchdog - max(0.0, age)
        return True

    def hold(self):
        self.targets = dict(self.positions)
        self.velocities = dict.fromkeys(self.joints, 0.0)
        self.deadlines = dict.fromkeys(self.joints, -math.inf)

    def step(self, now):
        dt = 0.0 if self.last_step is None else now - self.last_step
        self.last_step = now
        # A stalled process must not integrate a large catch-up jump.
        if dt < 0 or dt > self.watchdog:
            self.hold()
            return
        for name, joint in self.joints.items():
            if now >= self.deadlines[name]:
                self.targets[name] = self.positions[name]
            distance = self.targets[name] - self.positions[name]
            maximum = min(joint.speed, self.speed_limit) * dt
            step = min(max(distance, -maximum), maximum)
            self.positions[name] += step
            self.velocities[name] = step / dt if dt > 0 else 0.0
