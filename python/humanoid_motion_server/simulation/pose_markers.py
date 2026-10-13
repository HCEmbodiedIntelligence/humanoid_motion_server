"""Render measured FK and ServoP targets in the robot model's world frame."""

import math

import numpy as np

from ..cartesian_debug import fresh, pose_error, quaternion, stamp_ns, transform


def segment_transform(start, end):
    """Map the unit X segment to two world points without shear or zero scale."""
    delta = np.asarray(end) - np.asarray(start)
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return None
    x = delta / length
    helper = np.eye(3)[int(np.argmin(np.abs(x)))]
    z = np.cross(x, helper)
    z /= np.linalg.norm(z)
    matrix = np.eye(4)
    matrix[:3, 0] = delta
    matrix[:3, 1] = np.cross(z, x)
    matrix[:3, 2] = z
    matrix[:3, 3] = start
    return matrix


class PoseMarkers:
    def __init__(self, viewer, geometry, channels):
        self.viewer, self.geometry = viewer, geometry
        self.entries = {}
        self.visibility = {}
        self.last_clock = 0
        for channel in channels:
            if channel['kind'] != 'servo_p' or not channel.get('fk_pose_topic'):
                continue
            name = channel['name']
            path = f'pose_debug/{name}'
            entry = dict(channel=channel, path=path, actual=None, target=None,
                         actual_error='', target_error='', target_active=None, status={})
            self.entries[name] = entry
            self._marker(path + '/actual_FK', 0x36C5F0, 0.08, box=False)
            self._marker(path + '/target_ServoP', 0xFFAA33, 0.13, box=True)
            self._axes(path + '/reference_frame', 0.18, radius=0.0015)
            viewer[path + '/position_error'].set_object(geometry.Line(
                geometry.PointsGeometry(np.array([[0., 1.], [0., 0.], [0., 0.]])),
                geometry.LineBasicMaterial(color=0xFFDD66, depthTest=False)))
            for child in ('actual_FK', 'target_ServoP', 'reference_frame', 'position_error'):
                self._visible(path + '/' + child, False)

    def _visible(self, path, value):
        if self.visibility.get(path) != value:
            self.viewer[path].set_property('visible', value)
            self.visibility[path] = value

    def _axes(self, path, length, radius=0.002):
        g = self.geometry
        # Meshcat cylinders and cones point along their local +Y axis.
        rotations = (
            np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]]),
            np.eye(3),
            np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]),
        )
        for index, (axis, color) in enumerate(zip('XYZ', (0xEF4444, 0x22CC66, 0x3388FF))):
            material = g.MeshBasicMaterial(color=color, depthTest=False)
            for part, shape, center in (
                    ('shaft', g.Cylinder(length, radius), length/2),
                    ('positive', g.Cylinder(0.012, radiusTop=0., radiusBottom=radius*3), length+0.006)):
                marker = self.viewer[f'{path}/{axis}/{part}']
                marker.set_object(shape, material)
                matrix = np.eye(4)
                matrix[:3, :3] = rotations[index]
                matrix[index, 3] = center
                marker.set_transform(matrix)

    def _marker(self, path, color, length, *, box):
        g = self.geometry
        self.viewer[path + '/origin'].set_object(
            g.Sphere(0.009 if box else 0.014), g.MeshBasicMaterial(color=color, depthTest=False))
        self._axes(path + '/axes', length)
        if box:
            self.viewer[path + '/box'].set_object(
                g.Box([0.065, 0.04, 0.025]),
                g.MeshBasicMaterial(color=color, opacity=0.35, depthTest=False))
            matrix = np.eye(4)
            matrix[0, 3] = 0.0325  # Box extends along +X from the TCP origin.
            self.viewer[path + '/box'].set_transform(matrix)

    def receive(self, name, kind, message, now_ns):
        entry = self.entries[name]
        previous = entry[kind]
        # A deliberate clock reset invalidates previously cached commands.
        if previous is not None and stamp_ns(previous) > now_ns + 100_000_000:
            entry[kind] = previous = None
        try:
            transform(message)
            if not fresh(message, now_ns, 0.25):
                raise ValueError('位姿时间戳过期或无效')
            if previous is not None and stamp_ns(message) <= stamp_ns(previous):
                return
        except ValueError as error:
            entry[kind] = None
            entry[kind + '_error'] = str(error)
            return
        entry[kind], entry[kind + '_error'] = message, ''

    def render(self, now_ns, frame_transform, robot_fresh):
        if now_ns < self.last_clock:
            for entry in self.entries.values():
                entry['actual'] = entry['target'] = None
        self.last_clock = now_ns
        for entry in self.entries.values():
            path, channel = entry['path'], entry['channel']
            status = {'group': channel['group'], 'reference_frame': channel['base_frame'],
                      'tool_frame': channel['tip_frame'], 'target_topic': channel['endpoint']}
            matrices = {}
            try:
                base = frame_transform(channel['base_frame'])
            except ValueError:
                base = None
            self._visible(path + '/reference_frame', robot_fresh and base is not None)
            if robot_fresh and base is not None:
                self.viewer[path + '/reference_frame'].set_transform(base)
            for kind, child in (('actual', 'actual_FK'), ('target', 'target_ServoP')):
                message = entry[kind]
                current = fresh(message, now_ns, 0.25)
                status[kind + '_fresh'] = current
                status[kind + '_error'] = entry[kind + '_error']
                visible = robot_fresh and message is not None and (kind == 'target' or current)
                if visible:
                    try:
                        matrix = frame_transform(message.header.frame_id) @ transform(message)
                        matrices[kind] = matrix
                        self.viewer[path + '/' + child].set_transform(matrix)
                        status[kind + '_frame'] = message.header.frame_id
                        status[kind + '_xyz_m'] = transform(message)[:3, 3].tolist()
                        status[kind + '_world_xyz_m'] = matrix[:3, 3].tolist()
                        status[kind + '_quaternion_xyzw'] = quaternion(message.pose).tolist()
                        status[kind + '_age_s'] = (now_ns-stamp_ns(message))/1e9
                    except ValueError as error:
                        visible = False
                        status[kind + '_error'] = str(error)
                status[kind + '_visible'] = visible
                self._visible(path + '/' + child, visible)
            active = status['target_fresh']
            if active != entry['target_active']:
                color = [1.0, 170/255, 0.2] if active else [0.53, 0.53, 0.53]
                for part in ('origin', 'box'):
                    alpha = (1.0 if active else 0.65) if part == 'origin' else (0.35 if active else 0.18)
                    self.viewer[f'{path}/target_ServoP/{part}'].set_property('color', [*color, alpha])
                # Keep the last commanded pose for inspection, with dimmed axes.
                for axis, rgb in zip('XYZ', ([239/255, 68/255, 68/255],
                                            [34/255, 204/255, 102/255], [51/255, 136/255, 1.0])):
                    for part in ('shaft', 'positive'):
                        self.viewer[f'{path}/target_ServoP/axes/{axis}/{part}'].set_property(
                            'color', [*rgb, 1.0 if active else 0.3])
                entry['target_active'] = active
            comparable = 'actual' in matrices and 'target' in matrices
            line = None
            if comparable:
                actual, target = matrices['actual'], matrices['target']
                position, angle = pose_error(actual, target)
                status.update(position_error_m=position, orientation_error_deg=math.degrees(angle))
                line = segment_transform(actual[:3, 3], target[:3, 3])
            self._visible(path + '/position_error', line is not None)
            if line is not None:
                self.viewer[path + '/position_error'].set_transform(line)
            entry['status'] = status

    def status(self):
        return {name: entry['status'] for name, entry in self.entries.items()}
