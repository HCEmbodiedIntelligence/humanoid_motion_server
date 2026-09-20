"""URDF resources and scalar joint metadata shared by simulation and display."""

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def resource_path(value, base, prefixes=()):
    if value.startswith('package://'):
        package, relative = value[len('package://'):].split('/', 1)
        prefix = next((Path(p) for p in prefixes if p and
                       (Path(p) / 'share/ament_index/resource_index/packages' / package).is_file()), None)
        if prefix is None:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory(package))
        else:
            share = prefix / 'share' / package
        path = share / relative
    elif value.startswith('file://'):
        path = Path(value[len('file://'):])
    else:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(base) / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f'Resource does not exist: {path}')
    return path


@dataclass(frozen=True)
class Joint:
    lower: float
    upper: float
    speed: float
    kind: str


def joint_metadata(urdf):
    joints, mimics = {}, {}
    for element in ET.parse(urdf).getroot().findall('joint'):
        name, kind = element.attrib['name'], element.attrib['type']
        if kind == 'fixed':
            continue
        if kind not in ('revolute', 'continuous', 'prismatic'):
            raise ValueError(f'Simulation supports fixed-base scalar joints; {name} is {kind}')
        limit = element.find('limit')
        lower = -math.inf if kind == 'continuous' else float(limit.attrib['lower'])
        upper = math.inf if kind == 'continuous' else float(limit.attrib['upper'])
        speed = float(limit.attrib['velocity']) if limit is not None else 1.0
        if lower > upper or math.isnan(lower) or math.isnan(upper) or not math.isfinite(speed) or speed <= 0:
            raise ValueError(f'Invalid URDF limits: {name}')
        joints[name] = Joint(lower, upper, speed, kind)
        mimic = element.find('mimic')
        if mimic is not None:
            mimics[name] = (mimic.attrib['joint'], float(mimic.get('multiplier', 1)),
                            float(mimic.get('offset', 0)))
    # Also validate cycles and unknown parents for chained mimic joints.
    expand_mimics({name: 0.0 for name in joints if name not in mimics}, mimics)
    return joints, mimics


def expand_mimics(positions, mimics):
    result = dict(positions)
    pending = dict(mimics)
    while pending:
        ready = [name for name, (parent, _, _) in pending.items() if parent in result]
        if not ready:
            raise ValueError('URDF mimic cycle or unknown source joint')
        for name in ready:
            parent, scale, offset = pending.pop(name)
            if not math.isfinite(scale) or not math.isfinite(offset):
                raise ValueError(f'Invalid mimic transform: {name}')
            result[name] = result[parent] * scale + offset
    return result


def _joint_structure(root):
    # Visual overlays may never replace the authoritative kinematics.
    result = {}
    for joint in root.findall('joint'):
        values = [joint.attrib['type']]
        for tag in ('parent', 'child', 'origin', 'axis'):
            item = joint.find(tag)
            attrs = dict(item.attrib) if item is not None else {}
            if tag == 'origin':
                attrs = {key: tuple(float(x) for x in attrs.get(key, '0 0 0').split())
                         for key in ('xyz', 'rpy')}
            elif tag == 'axis':
                attrs = {'xyz': tuple(float(x) for x in attrs.get('xyz', '1 0 0').split())}
            values.append(attrs)
        result[joint.attrib['name']] = values
    return result


def visual_model(urdf, overlay=None, prefixes=()):
    """Return a display-only URDF, resolving assets before moving it to a temp dir."""
    urdf = Path(urdf)
    root = ET.parse(urdf).getroot()
    source, asset_base = root, urdf.parent
    if overlay:
        overlay = Path(overlay)
        source = ET.parse(overlay).getroot()
        if (_joint_structure(root) != _joint_structure(source) or
                {x.get('name') for x in root.findall('link')} !=
                {x.get('name') for x in source.findall('link')}):
            raise ValueError('visual_urdf must have the same links and joint kinematics as urdf')
        asset_base = overlay.parent
        source_links = {link.attrib['name']: link for link in source.findall('link')}
        for link in root.findall('link'):
            for visual in list(link.findall('visual')):
                link.remove(visual)
            for visual in source_links[link.attrib['name']].findall('visual'):
                link.append(deepcopy(visual))
        for material in list(root.findall('material')):
            root.remove(material)
        root.extend(deepcopy(source.findall('material')))
    for link in root.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)
    for element in list(root.findall('.//mesh')) + list(root.findall('.//texture')):
        element.set('filename', str(resource_path(element.attrib['filename'], asset_base, prefixes)))
    return ET.tostring(root, encoding='unicode')
