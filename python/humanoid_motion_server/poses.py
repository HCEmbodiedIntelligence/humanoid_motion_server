"""Robot-independent named joint poses shared by management and simulation."""
import copy
import math
import re


class PoseConfigurationError(ValueError):
    pass


def pose_from_joint_positions(resources, positions, *, pose_id, name, options=None):
    """Represent a joint snapshot using the same named-pose schema as saved poses."""
    motion = resources['motion_params']['humanoid_motion_control']['ros__parameters']
    candidates, seen = [], set()
    for channel in resources['channel_config']['channels']:
        if channel['kind'] != 'move_j':
            continue
        names = motion[f"groups.{channel['group']}"]
        key = frozenset(names)
        if key and key <= positions.keys() and key not in seen:
            seen.add(key)
            candidates.append((key, {'channel': channel['name'],
                                     'positions_rad': [positions[n] for n in names]}))
    candidates.sort(key=lambda item: -len(item[0]))

    def cover(remaining):
        if not remaining:
            return []
        joint = min(remaining)
        for names, target in candidates:
            if joint in names and names <= remaining:
                rest = cover(remaining - names)
                if rest is not None:
                    return [target] + rest
        return None

    targets = cover(set(positions))
    if not targets:
        raise PoseConfigurationError('Pose requires disjoint MoveJ channels covering all requested joints')
    pose = {'id': pose_id, 'name': name, 'targets': targets, **(options or {})}
    return validate_initial_poses([pose], resources)[0]


def validate_initial_poses(value, resources):
    """Validate named MoveJ poses against the editable model and channel resources."""
    if not isinstance(value, list) or len(value) > 32:
        raise PoseConfigurationError("初始姿态必须是列表，且最多 32 个")
    if not isinstance(resources, dict):
        raise PoseConfigurationError("初始姿态缺少机器人资源")
    try:
        motion = resources["motion_params"]["humanoid_motion_control"]["ros__parameters"]
        channels = resources["channel_config"]["channels"]
    except (KeyError, TypeError) as error:
        raise PoseConfigurationError("初始姿态无法读取运动分组或通道配置") from error
    if not isinstance(channels, list):
        raise PoseConfigurationError("初始姿态无法读取运动通道")
    channel_map = {}
    for channel in channels:
        if isinstance(channel, dict) and isinstance(channel.get("name"), str):
            channel_map[channel["name"]] = channel

    result, identifiers = [], set()
    allowed_pose_keys = {
        "id", "name", "targets", "velocity_scale", "acceleration_scale",
        "jerk_scale", "timeout_sec",
    }
    for raw in value:
        if not isinstance(raw, dict) or set(raw) - allowed_pose_keys:
            raise PoseConfigurationError("每个初始姿态必须是有效对象且不能包含未知字段")
        pose = copy.deepcopy(raw)
        ident = pose.get("id", "")
        if not isinstance(ident, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", ident):
            raise PoseConfigurationError("初始姿态 ID 需以小写字母开头，只允许小写字母、数字和下划线")
        if ident in identifiers:
            raise PoseConfigurationError(f"初始姿态 ID 重复: {ident}")
        identifiers.add(ident)
        name = pose.get("name", "")
        if not isinstance(name, str) or not name.strip() or len(name) > 100:
            raise PoseConfigurationError(f"{ident}: 初始姿态名称需为 1–100 个字符")
        pose["name"] = name.strip()
        defaults = {
            "velocity_scale": 0.15,
            "acceleration_scale": 0.15,
            "jerk_scale": 0.15,
            "timeout_sec": 60.0,
        }
        for key, default in defaults.items():
            pose.setdefault(key, default)
            number = pose[key]
            upper = 600.0 if key == "timeout_sec" else 1.0
            if (isinstance(number, bool) or not isinstance(number, (int, float)) or
                    not math.isfinite(number) or not 0 < number <= upper):
                unit = "0–600 秒（不含 0）" if key == "timeout_sec" else "0–1（不含 0）"
                raise PoseConfigurationError(f"{ident}.{key} 必须在 {unit}范围内")
            pose[key] = float(number)

        targets = pose.get("targets")
        if not isinstance(targets, list) or not targets or len(targets) > 16:
            raise PoseConfigurationError(f"{ident}: 至少需要一个初始姿态目标，最多 16 个")
        normalized_targets, used_channels, used_joints = [], set(), set()
        for raw_target in targets:
            if not isinstance(raw_target, dict) or set(raw_target) != {"channel", "positions_rad"}:
                raise PoseConfigurationError(f"{ident}: 姿态目标只能包含 channel 和 positions_rad")
            channel_name = raw_target.get("channel", "")
            channel = channel_map.get(channel_name)
            if channel is None or channel.get("kind") != "move_j" or not channel.get("group"):
                raise PoseConfigurationError(f"{ident}: 通道 {channel_name} 不是带关节组的 MoveJ 通道")
            if channel_name in used_channels:
                raise PoseConfigurationError(f"{ident}: MoveJ 通道重复: {channel_name}")
            used_channels.add(channel_name)
            group = channel["group"]
            joints = motion.get(f"groups.{group}")
            lower = motion.get(f"group_lower_limits.{group}")
            upper = motion.get(f"group_upper_limits.{group}")
            if not (isinstance(joints, list) and isinstance(lower, list) and isinstance(upper, list) and
                    len(joints) == len(lower) == len(upper) and joints):
                raise PoseConfigurationError(f"{ident}: 通道 {channel_name} 的关节组或限位不完整")
            overlap = used_joints.intersection(joints)
            if overlap:
                raise PoseConfigurationError(f"{ident}: 多个目标包含相同关节: {', '.join(sorted(overlap))}")
            used_joints.update(joints)
            positions = raw_target.get("positions_rad")
            if not isinstance(positions, list) or len(positions) != len(joints):
                raise PoseConfigurationError(f"{ident}: {group} 需要 {len(joints)} 个关节位置")
            normalized_positions = []
            for joint, position, low, high in zip(joints, positions, lower, upper):
                if (isinstance(position, bool) or not isinstance(position, (int, float)) or
                        not math.isfinite(position)):
                    raise PoseConfigurationError(f"{ident}.{joint}: 初始位置必须是有限数值")
                if position < low or position > high:
                    raise PoseConfigurationError(f"{ident}.{joint}: {position} rad 超出限位 [{low}, {high}]")
                normalized_positions.append(float(position))
            normalized_targets.append({"channel": channel_name, "positions_rad": normalized_positions})
        pose["targets"] = normalized_targets
        result.append(pose)
    return result


def resolve_initial_pose(document, pose_id):
    """Resolve a validated named pose to concrete MoveJ action endpoints."""
    poses = validate_initial_poses(document.get("initial_poses", []), document.get("resources"))
    pose = next((item for item in poses if item["id"] == pose_id), None)
    if pose is None:
        raise PoseConfigurationError("初始姿态不存在")
    resources = document["resources"]
    motion = resources["motion_params"]["humanoid_motion_control"]["ros__parameters"]
    channels = {item["name"]: item for item in resources["channel_config"]["channels"]}
    goals = []
    for target in pose["targets"]:
        channel = channels[target["channel"]]
        group = channel["group"]
        goals.append({
            "channel": channel["name"],
            "endpoint": channel["endpoint"],
            "group": group,
            "joint_names": list(motion[f"groups.{group}"]),
            "positions_rad": list(target["positions_rad"]),
            "velocity_scale": pose["velocity_scale"],
            "acceleration_scale": pose["acceleration_scale"],
            "jerk_scale": pose["jerk_scale"],
            "timeout_sec": pose["timeout_sec"],
        })
    return {**pose, "goals": goals}


def resolve_joint_jog(document, joint_name, delta_rad):
    """Resolve one small joint increment to a feedback-seeded MoveJ goal."""
    if not isinstance(joint_name, str) or not joint_name:
        raise PoseConfigurationError("点动关节名不能为空")
    if (isinstance(delta_rad, bool) or not isinstance(delta_rad, (int, float)) or
            not math.isfinite(delta_rad) or not 0 < abs(delta_rad) <= 0.2):
        raise PoseConfigurationError("关节点动步长必须在 0–0.2 rad 范围内（不含 0）")
    resources = document.get("resources")
    if not isinstance(resources, dict):
        raise PoseConfigurationError("机器人运动资源不存在")
    try:
        motion = resources["motion_params"]["humanoid_motion_control"]["ros__parameters"]
        channels = resources["channel_config"]["channels"]
    except (KeyError, TypeError) as error:
        raise PoseConfigurationError("机器人运动参数或通道不完整") from error
    candidates = []
    for channel in channels:
        if not isinstance(channel, dict) or channel.get("kind") != "move_j":
            continue
        group = channel.get("group")
        joints = motion.get(f"groups.{group}")
        lower = motion.get(f"group_lower_limits.{group}")
        upper = motion.get(f"group_upper_limits.{group}")
        if (joint_name in (joints or []) and isinstance(lower, list) and
                isinstance(upper, list) and len(joints) == len(lower) == len(upper)):
            candidates.append((len(joints), channel, joints, lower, upper))
    if not candidates:
        raise PoseConfigurationError(f"{joint_name}: 没有可用于点动的 MoveJ 通道")
    _, channel, joints, lower, upper = min(candidates, key=lambda item: item[0])
    driver = resources.get("driver_params", {}).get(
        "humanoid_driver_runtime", {}).get("ros__parameters", {})
    state_topic = motion.get("joint_state_endpoint") or driver.get(
        "platform_joint_state_topic", "/hc_teleop/joint_states"
    )
    if not isinstance(state_topic, str) or not state_topic:
        raise PoseConfigurationError("机器人没有配置统一关节反馈话题")
    return {
        "channel": channel["name"],
        "endpoint": channel["endpoint"],
        "group": channel["group"],
        "joint_names": list(joints),
        "lower_limits": [float(value) for value in lower],
        "upper_limits": [float(value) for value in upper],
        "joint_name": joint_name,
        "delta_rad": float(delta_rad),
        "state_topic": state_topic,
        "velocity_scale": 0.1,
        "acceleration_scale": 0.1,
        "jerk_scale": 0.1,
        "timeout_sec": 15.0,
    }
