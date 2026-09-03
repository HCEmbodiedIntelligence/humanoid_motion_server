"""Validate and resolve no-build robot and hardware-plugin deployments."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import stat
import subprocess
import tempfile
from typing import Any
import xml.etree.ElementTree as ET
import zipfile

import yaml


SCHEMA_VERSION = 1
DRIVER_INTERFACE_ABI = 1
SUPPORTED_ROS_DISTRO = "humble"
DEFAULT_PLUGIN_ROOT = Path("/var/lib/humanoid-plugins")
MAX_ARCHIVE_FILES = 10000
MAX_ARCHIVE_BYTES = 2 * 1024 * 1024 * 1024
MAX_CONFIG_BYTES = 16 * 1024 * 1024
MAX_MEMBER_PATH_LENGTH = 512

_SAFE_ID = re.compile(r"^[a-z0-9][a-z0-9_.-]*$")
_SAFE_PACKAGE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_PACKAGE_URI = re.compile(r"^package://([A-Za-z][A-Za-z0-9_-]*)/(.+)$")

_TYPE_DIR = {
    "hardware_driver": "hardware_drivers",
    "robot_profile": "robot_profiles",
}
_RESOURCE_REQUIRED = {
    "driver_params",
    "motion_params",
    "sdk_config",
    "channel_config",
    "tool_config",
    "urdf",
}
_RESOURCE_OPTIONAL = {"teleop_config"}
_CORE_DRIVER_CLASSES = {
    "humanoid_driver_runtime/MockRobotDriver",
    "humanoid_driver_runtime/RosTopicRobotDriver",
}


class DeploymentError(RuntimeError):
    """A deployment bundle is unsafe, incompatible, or internally inconsistent."""


@dataclass(frozen=True)
class ResolvedDeployment:
    robot_id: str
    name: str
    manifest_path: Path
    resources: dict[str, Path]
    driver_class: str
    driver_plugin_xml_paths: tuple[Path, ...]
    ament_prefixes: tuple[Path, ...]
    library_paths: tuple[Path, ...]

    def environment(self, inherited: dict[str, str] | None = None) -> dict[str, str]:
        base = os.environ if inherited is None else inherited
        result: dict[str, str] = {}
        if self.ament_prefixes:
            result["AMENT_PREFIX_PATH"] = _prepend_paths(
                self.ament_prefixes, base.get("AMENT_PREFIX_PATH", "")
            )
        if self.library_paths:
            result["LD_LIBRARY_PATH"] = _prepend_paths(
                self.library_paths, base.get("LD_LIBRARY_PATH", "")
            )
        return result

    def as_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "name": self.name,
            "manifest_path": str(self.manifest_path),
            "resources": {key: str(value) for key, value in self.resources.items()},
            "driver_class": self.driver_class,
            "driver_plugin_xml_paths": [str(path) for path in self.driver_plugin_xml_paths],
            "ament_prefixes": [str(path) for path in self.ament_prefixes],
            "library_paths": [str(path) for path in self.library_paths],
            "environment": self.environment(),
        }


def _prepend_paths(paths: tuple[Path, ...], inherited: str) -> str:
    values = [str(path) for path in paths]
    if inherited:
        values.append(inherited)
    return os.pathsep.join(values)


def _load_yaml(path: Path, label: str) -> Any:
    try:
        if path.stat().st_size > MAX_CONFIG_BYTES:
            raise DeploymentError(f"{label} exceeds the configuration size limit")
        with path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)
    except DeploymentError:
        raise
    except (OSError, yaml.YAMLError) as error:
        raise DeploymentError(f"failed to load {label} '{path}': {error}") from error


def _require_closed_mapping(
    value: Any, required: set[str], optional: set[str], label: str
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DeploymentError(f"{label} must be a mapping")
    keys = set(value)
    missing = required - keys
    unknown = keys - required - optional
    if missing:
        raise DeploymentError(f"{label} is missing keys: {', '.join(sorted(missing))}")
    if unknown:
        raise DeploymentError(f"{label} has unknown keys: {', '.join(sorted(unknown))}")
    return value


def _require_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise DeploymentError(f"{label} must be a non-empty string")
    return value


def _require_id(value: Any, label: str = "plugin_id") -> str:
    text = _require_text(value, label)
    if not _SAFE_ID.fullmatch(text):
        raise DeploymentError(f"{label} contains unsupported characters")
    return text


def _resolve_member(root: Path, value: Any, label: str, *, directory: bool = False) -> Path:
    text = _require_text(value, label)
    relative = Path(text)
    if relative.is_absolute() or ".." in relative.parts:
        raise DeploymentError(f"{label} must be a bundle-relative path")
    resolved_root = root.resolve()
    resolved = (resolved_root / relative).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError as error:
        raise DeploymentError(f"{label} escapes the bundle root") from error
    valid = resolved.is_dir() if directory else resolved.is_file()
    if not valid:
        kind = "directory" if directory else "file"
        raise DeploymentError(f"{label} does not reference an existing {kind}: {text}")
    return resolved


def _normalize_architecture(value: str) -> str:
    lowered = value.lower()
    if lowered in {"amd64", "x86_64"}:
        return "x86_64"
    if lowered in {"arm64", "aarch64"}:
        return "aarch64"
    return lowered


def _validate_common_manifest(document: Any, plugin_type: str) -> dict[str, Any]:
    manifest = _require_closed_mapping(
        document,
        {
            "schema_version",
            "plugin_type",
            "plugin_id",
            "name",
        },
        {
            "compatibility",
            "package_name",
            "ament_prefix",
            "plugin_xml",
            "library",
            "plugin_class",
            "driver",
            "resources",
        },
        "manifest",
    )
    if manifest["schema_version"] != SCHEMA_VERSION:
        raise DeploymentError("unsupported deployment schema_version")
    if manifest["plugin_type"] != plugin_type:
        raise DeploymentError(f"manifest plugin_type must be '{plugin_type}'")
    _require_id(manifest["plugin_id"])
    _require_text(manifest["name"], "name")
    return manifest


def validate_hardware_tree(root: Path, *, check_linkage: bool = False) -> dict[str, Any]:
    root = root.resolve()
    manifest = _validate_common_manifest(
        _load_yaml(root / "manifest.yaml", "hardware manifest"), "hardware_driver"
    )
    required = {
        "schema_version",
        "plugin_type",
        "plugin_id",
        "name",
        "compatibility",
        "package_name",
        "ament_prefix",
        "plugin_xml",
        "library",
        "plugin_class",
    }
    optional: set[str] = set()
    _require_closed_mapping(manifest, required, optional, "hardware manifest")

    compatibility = _require_closed_mapping(
        manifest["compatibility"],
        {"ros_distro", "architecture", "driver_interface_abi"},
        set(),
        "hardware compatibility",
    )
    if compatibility["ros_distro"] != SUPPORTED_ROS_DISTRO:
        raise DeploymentError(
            f"driver targets ROS {compatibility['ros_distro']}, expected {SUPPORTED_ROS_DISTRO}"
        )
    expected_arch = _normalize_architecture(platform.machine())
    if _normalize_architecture(_require_text(
        compatibility["architecture"], "compatibility.architecture"
    )) != expected_arch:
        raise DeploymentError("driver architecture does not match this host")
    if compatibility["driver_interface_abi"] != DRIVER_INTERFACE_ABI:
        raise DeploymentError("driver_interface ABI is incompatible")

    package_name = _require_text(manifest["package_name"], "package_name")
    if not _SAFE_PACKAGE.fullmatch(package_name):
        raise DeploymentError("package_name is invalid")
    plugin_class = _require_text(manifest["plugin_class"], "plugin_class")
    prefix = _resolve_member(root, manifest["ament_prefix"], "ament_prefix", directory=True)
    plugin_xml = _resolve_member(root, manifest["plugin_xml"], "plugin_xml")
    library = _resolve_member(root, manifest["library"], "library")

    expected_package_share = prefix / "share" / package_name
    try:
        plugin_xml.relative_to(expected_package_share)
    except ValueError as error:
        raise DeploymentError("plugin_xml must be inside the exported package share") from error
    if library.parent != prefix / "lib" or library.suffix != ".so":
        raise DeploymentError("hardware library must be a .so directly inside ament_prefix/lib")

    package_marker = prefix / "share/ament_index/resource_index/packages" / package_name
    package_xml = prefix / "share" / package_name / "package.xml"
    if not package_marker.is_file() or not package_xml.is_file():
        raise DeploymentError("hardware bundle does not contain a complete ament package prefix")
    if package_xml.stat().st_size > MAX_CONFIG_BYTES:
        raise DeploymentError("deployed package.xml exceeds the configuration size limit")
    if plugin_xml.stat().st_size > MAX_CONFIG_BYTES:
        raise DeploymentError("plugin XML exceeds the configuration size limit")
    try:
        package_root = ET.parse(package_xml).getroot()
        declared_name = package_root.findtext("name")
    except ET.ParseError as error:
        raise DeploymentError(f"invalid deployed package.xml: {error}") from error
    if declared_name != package_name:
        raise DeploymentError("deployed package.xml name differs from manifest package_name")

    try:
        xml_root = ET.parse(plugin_xml).getroot()
    except ET.ParseError as error:
        raise DeploymentError(f"invalid plugin XML: {error}") from error
    libraries = [xml_root] if xml_root.tag == "library" else list(xml_root.findall("library"))
    matching_classes = []
    library_names = []
    for library_node in libraries:
        library_names.append(library_node.attrib.get("path", ""))
        for class_node in library_node.findall("class"):
            if class_node.attrib.get("name") == plugin_class:
                matching_classes.append(class_node)
    if len(matching_classes) != 1:
        raise DeploymentError("plugin XML must declare plugin_class exactly once")
    if matching_classes[0].attrib.get("base_class_type") != (
        "humanoid_driver_interface::RobotDriverPlugin"
    ):
        raise DeploymentError("hardware plugin uses the wrong base_class_type")
    expected_library_names = {
        library.stem,
        library.stem.removeprefix("lib"),
    }
    declared_library_names = {Path(name).name for name in library_names if name}
    if not expected_library_names.intersection(declared_library_names):
        raise DeploymentError("plugin XML library path differs from the bundled shared library")
    try:
        with library.open("rb") as stream:
            if stream.read(4) != b"\x7fELF":
                raise DeploymentError("hardware library is not an ELF shared object")
    except OSError as error:
        raise DeploymentError(f"failed to inspect hardware library: {error}") from error

    if check_linkage:
        environment = os.environ.copy()
        environment["LD_LIBRARY_PATH"] = _prepend_paths(
            (prefix / "lib",), environment.get("LD_LIBRARY_PATH", "")
        )
        completed = subprocess.run(
            ["ldd", "-r", str(library)],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        detail = (completed.stdout + completed.stderr).strip()
        if (
            completed.returncode != 0
            or "not found" in detail
            or "undefined symbol:" in detail
        ):
            raise DeploymentError(f"hardware plugin dependency check failed: {detail}")

    result = dict(manifest)
    result["_root"] = root
    result["_ament_prefix"] = prefix
    result["_plugin_xml"] = plugin_xml
    result["_library"] = library
    return result


def _ros_parameters(document: Any, node_name: str, label: str) -> dict[str, Any]:
    root = _require_closed_mapping(document, {node_name}, set(), label)
    node = _require_closed_mapping(root[node_name], {"ros__parameters"}, set(), label)
    parameters = node["ros__parameters"]
    if not isinstance(parameters, dict):
        raise DeploymentError(f"{label} ros__parameters must be a mapping")
    return parameters


def _string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list) or not value or not all(
        isinstance(item, str) and item for item in value
    ):
        raise DeploymentError(f"{label} must be a non-empty string list")
    if len(set(value)) != len(value):
        raise DeploymentError(f"{label} contains duplicates")
    return value


def _text_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list) or not value or not all(
        isinstance(item, str) and item for item in value
    ):
        raise DeploymentError(f"{label} must be a non-empty string list")
    return value


def _finite_number_list(value: Any, label: str, size: int) -> list[float]:
    if not isinstance(value, list) or len(value) != size or not all(
        isinstance(item, (int, float))
        and not isinstance(item, bool)
        and math.isfinite(float(item))
        for item in value
    ):
        raise DeploymentError(f"{label} must contain {size} finite numbers")
    return [float(item) for item in value]


def validate_robot_tree(root: Path) -> dict[str, Any]:
    root = root.resolve()
    manifest = _validate_common_manifest(
        _load_yaml(root / "manifest.yaml", "robot manifest"), "robot_profile"
    )
    required = {
        "schema_version",
        "plugin_type",
        "plugin_id",
        "name",
        "driver",
        "resources",
    }
    optional = {"ament_prefix"}
    _require_closed_mapping(manifest, required, optional, "robot manifest")

    driver = _require_closed_mapping(
        manifest["driver"], {"source", "plugin_class"}, {"plugin_id"}, "robot driver"
    )
    source = driver["source"]
    if source not in {"core", "deployed"}:
        raise DeploymentError("robot driver source must be 'core' or 'deployed'")
    _require_text(driver["plugin_class"], "driver.plugin_class")
    if source == "deployed":
        _require_id(driver.get("plugin_id"), "driver.plugin_id")
    elif "plugin_id" in driver:
        raise DeploymentError("core robot drivers must not declare driver.plugin_id")
    elif driver["plugin_class"] not in _CORE_DRIVER_CLASSES:
        raise DeploymentError("robot profile references an unknown core driver class")

    resources_node = _require_closed_mapping(
        manifest["resources"], _RESOURCE_REQUIRED, _RESOURCE_OPTIONAL, "robot resources"
    )
    resources = {
        key: _resolve_member(root, value, f"resources.{key}")
        for key, value in resources_node.items()
    }
    for key, path in resources.items():
        if path.stat().st_size > MAX_CONFIG_BYTES:
            raise DeploymentError(f"resources.{key} exceeds the configuration size limit")
    prefix: Path | None = None
    if "ament_prefix" in manifest:
        prefix = _resolve_member(root, manifest["ament_prefix"], "ament_prefix", directory=True)

    driver_parameters = _ros_parameters(
        _load_yaml(resources["driver_params"], "driver parameters"),
        "humanoid_driver_runtime",
        "driver parameters",
    )
    if "plugin_xml_paths" in driver_parameters:
        raise DeploymentError("driver YAML must not override managed plugin_xml_paths")
    if driver_parameters.get("plugin_class") != driver["plugin_class"]:
        raise DeploymentError("driver YAML plugin_class differs from the robot manifest")
    joint_names = _string_list(driver_parameters.get("joint_names"), "driver joint_names")
    vendor_names = _text_list(
        driver_parameters.get("vendor_joint_names"), "driver vendor_joint_names"
    )
    vendor_groups = driver_parameters.get("vendor_joint_groups")
    if not isinstance(vendor_groups, list) or len(vendor_groups) != len(joint_names) or not all(
        isinstance(item, str) and item for item in vendor_groups
    ):
        raise DeploymentError("driver vendor_joint_groups must match joint_names")
    if len(vendor_names) != len(joint_names):
        raise DeploymentError("driver vendor_joint_names must match joint_names")
    vendor_keys = list(zip(vendor_groups, vendor_names))
    if len(set(vendor_keys)) != len(vendor_keys):
        raise DeploymentError("driver vendor group/name pairs must be unique")
    for parameter_name, default in (
        ("vendor_to_logical_scales", [1.0] * len(joint_names)),
        ("vendor_to_logical_offsets_rad", [0.0] * len(joint_names)),
    ):
        values = driver_parameters.get(parameter_name, default)
        numbers = _finite_number_list(values, f"driver {parameter_name}", len(joint_names))
        if parameter_name == "vendor_to_logical_scales" and any(
            abs(value) < 1.0e-12 for value in numbers
        ):
            raise DeploymentError("driver vendor_to_logical_scales cannot contain zero")

    motion_parameters = _ros_parameters(
        _load_yaml(resources["motion_params"], "motion parameters"),
        "humanoid_motion_control",
        "motion parameters",
    )
    group_names = _string_list(
        motion_parameters.get("joint_group_names"), "motion joint_group_names"
    )
    motion_groups: dict[str, list[str]] = {}
    for group_name in group_names:
        joints = _string_list(
            motion_parameters.get(f"groups.{group_name}"), f"motion groups.{group_name}"
        )
        lower = _finite_number_list(
            motion_parameters.get(f"group_lower_limits.{group_name}"),
            f"motion group_lower_limits.{group_name}",
            len(joints),
        )
        upper = _finite_number_list(
            motion_parameters.get(f"group_upper_limits.{group_name}"),
            f"motion group_upper_limits.{group_name}",
            len(joints),
        )
        if any(low > high for low, high in zip(lower, upper)):
            raise DeploymentError(f"motion limits are inverted for group '{group_name}'")
        motion_groups[group_name] = joints
    motion_joints = {joint for group in motion_groups.values() for joint in group}
    if motion_joints != set(joint_names):
        raise DeploymentError("driver and motion configurations expose different logical joints")

    sdk = _load_yaml(resources["sdk_config"], "SDK configuration")
    if not isinstance(sdk, dict) or not isinstance(sdk.get("joint_groups"), dict):
        raise DeploymentError("SDK configuration must contain joint_groups")
    for group_name, expected in motion_groups.items():
        actual = sdk["joint_groups"].get(group_name)
        if actual != expected:
            raise DeploymentError(f"SDK and motion joint order differ for group '{group_name}'")

    try:
        urdf_root = ET.parse(resources["urdf"]).getroot()
    except ET.ParseError as error:
        raise DeploymentError(f"invalid robot URDF: {error}") from error
    if urdf_root.tag != "robot":
        raise DeploymentError("URDF root element must be robot")
    link_elements = urdf_root.findall("link")
    joint_elements = urdf_root.findall("joint")
    link_names = [element.attrib.get("name") for element in link_elements]
    joint_element_names = [element.attrib.get("name") for element in joint_elements]
    if (
        not link_names
        or any(not name for name in link_names)
        or len(set(link_names)) != len(link_names)
    ):
        raise DeploymentError("URDF link names must be non-empty and unique")
    if any(not name for name in joint_element_names) or len(set(joint_element_names)) != len(
        joint_element_names
    ):
        raise DeploymentError("URDF joint names must be non-empty and unique")
    urdf_joints = set(joint_element_names)
    urdf_links = set(link_names)
    child_links: set[str] = set()
    adjacency: dict[str, list[str]] = {name: [] for name in urdf_links}
    for element in joint_elements:
        parent_node = element.find("parent")
        child_node = element.find("child")
        parent = parent_node.attrib.get("link") if parent_node is not None else None
        child = child_node.attrib.get("link") if child_node is not None else None
        if not parent or not child or parent not in urdf_links or child not in urdf_links:
            raise DeploymentError(f"URDF joint '{element.attrib.get('name')}' has invalid links")
        if parent == child or child in child_links:
            raise DeploymentError("URDF must be a tree with one parent per child link")
        child_links.add(child)
        adjacency[parent].append(child)
    roots = urdf_links - child_links
    if len(roots) != 1:
        raise DeploymentError("URDF must contain exactly one connected root link")
    visited: set[str] = set()
    pending = [next(iter(roots))]
    while pending:
        link = pending.pop()
        if link in visited:
            raise DeploymentError("URDF link graph contains a cycle")
        visited.add(link)
        pending.extend(adjacency[link])
    if visited != urdf_links:
        raise DeploymentError("URDF contains disconnected links or a cycle")
    missing_urdf_joints = set(joint_names) - urdf_joints
    if missing_urdf_joints:
        raise DeploymentError(
            "URDF is missing configured joints: " + ", ".join(sorted(missing_urdf_joints))
        )

    package_resources: set[tuple[str, str]] = set()
    for element in urdf_root.iter():
        for value in element.attrib.values():
            if not value.startswith("package://"):
                continue
            match = _PACKAGE_URI.fullmatch(value)
            if match is None:
                raise DeploymentError(f"URDF contains an invalid package URI: {value}")
            package_resources.add((match.group(1), match.group(2)))
    if package_resources:
        if prefix is None:
            raise DeploymentError("URDF package:// resources require a bundled ament_prefix")
        for package_name, relative_text in package_resources:
            marker = prefix / "share/ament_index/resource_index/packages" / package_name
            if not marker.is_file():
                raise DeploymentError(
                    f"URDF references package '{package_name}' outside the resource bundle"
                )
            member = PurePosixPath(relative_text)
            package_share = (prefix / "share" / package_name).resolve()
            resource = (package_share / Path(*member.parts)).resolve()
            if member.is_absolute() or ".." in member.parts:
                raise DeploymentError(f"URDF package URI contains an unsafe path: {relative_text}")
            try:
                resource.relative_to(package_share)
            except ValueError as error:
                raise DeploymentError("URDF package resource escapes its bundled share") from error
            if not resource.is_file():
                raise DeploymentError(
                    "URDF package resource is missing: "
                    f"package://{package_name}/{relative_text}"
                )

    tools_document = _load_yaml(resources["tool_config"], "tool configuration")
    tools_mapping = _require_closed_mapping(
        tools_document, {"tools"}, set(), "tool configuration"
    )
    if not isinstance(tools_mapping["tools"], list):
        raise DeploymentError("tool configuration tools must be a sequence")
    tool_frames: set[str] = set()
    for tool in tools_mapping["tools"]:
        tool = _require_closed_mapping(
            tool,
            {"name", "parent_frame", "child_frame", "translation_m", "rotation_xyzw"},
            set(),
            "tool entry",
        )
        name = _require_text(tool.get("name"), "tool name")
        parent = _require_text(tool.get("parent_frame"), "tool parent_frame")
        child = _require_text(tool.get("child_frame"), "tool child_frame")
        if name != child:
            raise DeploymentError("tool name must equal child_frame")
        _finite_number_list(tool.get("translation_m"), "tool translation_m", 3)
        quaternion = _finite_number_list(tool.get("rotation_xyzw"), "tool rotation_xyzw", 4)
        if sum(value * value for value in quaternion) < 1.0e-12:
            raise DeploymentError("tool quaternion cannot be normalized")
        if parent not in urdf_links:
            raise DeploymentError(f"tool parent frame is absent from URDF: {parent}")
        if child in tool_frames or child in urdf_links:
            raise DeploymentError(f"duplicate tool frame: {child}")
        tool_frames.add(child)

    channels_document = _load_yaml(resources["channel_config"], "channel configuration")
    channels_mapping = _require_closed_mapping(
        channels_document, {"channels"}, set(), "channel configuration"
    )
    if not isinstance(channels_mapping["channels"], list) or not channels_mapping["channels"]:
        raise DeploymentError("channel configuration channels must be a non-empty sequence")
    channel_names: set[str] = set()
    channel_endpoints: set[str] = set()
    fk_topics: set[str] = set()
    allowed_channel_keys = {
        "name", "kind", "endpoint", "priority", "group", "base_frame", "tip_frame",
        "fk_pose_topic",
    }
    cartesian_kinds = {"move_l", "move_p", "servo_p"}
    servo_kinds = {"servo_j", "servo_p"}
    for channel in channels_mapping["channels"]:
        channel = _require_closed_mapping(
            channel, {"name", "kind", "endpoint", "priority"},
            allowed_channel_keys - {"name", "kind", "endpoint", "priority"},
            "channel entry",
        )
        name = _require_text(channel.get("name"), "channel name")
        kind = _require_text(channel.get("kind"), "channel kind")
        endpoint = _require_text(channel.get("endpoint"), "channel endpoint")
        if kind not in {"move_j", "move_l", "move_p", "servo_j", "servo_p"}:
            raise DeploymentError(f"channel has unsupported kind: {kind}")
        priority = channel.get("priority")
        if not isinstance(priority, int) or isinstance(priority, bool):
            raise DeploymentError("channel priority must be an integer")
        if name in channel_names or endpoint in channel_endpoints:
            raise DeploymentError("channel names and endpoints must be unique")
        channel_names.add(name)
        channel_endpoints.add(endpoint)
        group_name = channel.get("group")
        if group_name is not None:
            group_name = _require_text(group_name, "channel group")
        if group_name and group_name not in motion_groups:
            raise DeploymentError(f"channel references unknown group: {group_name}")
        if kind in servo_kinds and not group_name:
            raise DeploymentError("Servo channels require a group")
        if kind in cartesian_kinds and (
            not channel.get("base_frame") or not channel.get("tip_frame")
        ):
            raise DeploymentError("Cartesian channels require base_frame and tip_frame")
        for field in ("base_frame", "tip_frame"):
            frame = channel.get(field)
            if frame is not None:
                frame = _require_text(frame, f"channel {field}")
            if frame and frame not in urdf_links and frame not in tool_frames:
                raise DeploymentError(f"channel {field} is absent from URDF/tools: {frame}")
        fk_topic = channel.get("fk_pose_topic")
        if fk_topic:
            _require_text(fk_topic, "channel fk_pose_topic")
            if kind not in cartesian_kinds or not group_name or fk_topic in fk_topics:
                raise DeploymentError("FK topics require a unique Cartesian channel with a group")
            fk_topics.add(fk_topic)

    result = dict(manifest)
    result["_root"] = root
    result["_resources"] = resources
    result["_ament_prefix"] = prefix
    return result


def _tree_files(root: Path) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for path in root.rglob("*"):
        if path.is_symlink():
            raise DeploymentError(f"bundle trees must not contain symbolic links: {path}")
        if path.is_file():
            relative = path.relative_to(root).as_posix()
            if relative != "checksums.sha256":
                result[relative] = path
    return result


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_checksums(root: Path) -> None:
    files = _tree_files(root)
    lines = []
    for relative in sorted(files):
        digest = _sha256_file(files[relative])
        lines.append(f"{digest}  {relative}\n")
    (root / "checksums.sha256").write_text("".join(lines), encoding="utf-8")


def verify_checksums(root: Path) -> None:
    checksum_path = root / "checksums.sha256"
    if not checksum_path.is_file():
        raise DeploymentError("bundle is missing checksums.sha256")
    if checksum_path.stat().st_size > MAX_CONFIG_BYTES:
        raise DeploymentError("checksums.sha256 exceeds the configuration size limit")
    expected: dict[str, str] = {}
    for line in checksum_path.read_text(encoding="utf-8").splitlines():
        digest, separator, relative = line.partition("  ")
        if not separator or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise DeploymentError("checksums.sha256 has invalid syntax")
        member = PurePosixPath(relative)
        if member.is_absolute() or ".." in member.parts or relative in expected:
            raise DeploymentError("checksums.sha256 contains an unsafe or duplicate path")
        expected[relative] = digest
    files = _tree_files(root)
    if set(expected) != set(files):
        raise DeploymentError("checksums.sha256 does not cover every bundle file exactly once")
    for relative, path in files.items():
        actual = _sha256_file(path)
        if actual != expected[relative]:
            raise DeploymentError(f"checksum mismatch for {relative}")


def validate_tree(
    root: Path,
    *,
    check_linkage: bool = False,
) -> dict[str, Any]:
    verify_checksums(root)
    document = _load_yaml(root / "manifest.yaml", "manifest")
    if not isinstance(document, dict):
        raise DeploymentError("manifest must be a mapping")
    plugin_type = document.get("plugin_type")
    if plugin_type == "hardware_driver":
        return validate_hardware_tree(root, check_linkage=check_linkage)
    if plugin_type == "robot_profile":
        return validate_robot_tree(root)
    raise DeploymentError("manifest plugin_type is unsupported")


def _safe_extract(archive: Path, destination: Path) -> None:
    total_size = 0
    names: set[str] = set()
    try:
        source = zipfile.ZipFile(archive)
    except (OSError, zipfile.BadZipFile) as error:
        raise DeploymentError(f"invalid deployment archive: {error}") from error
    with source:
        members = source.infolist()
        if len(members) > MAX_ARCHIVE_FILES:
            raise DeploymentError("deployment archive contains too many files")
        for info in members:
            path = PurePosixPath(info.filename)
            if (
                not info.filename
                or len(info.filename) > MAX_MEMBER_PATH_LENGTH
                or len(path.parts) > 32
                or path.is_absolute()
                or ".." in path.parts
            ):
                raise DeploymentError("deployment archive contains an unsafe path")
            normalized = path.as_posix().rstrip("/")
            if normalized in names:
                raise DeploymentError("deployment archive contains duplicate paths")
            names.add(normalized)
            mode = (info.external_attr >> 16) & 0xFFFF
            if stat.S_ISLNK(mode) or info.flag_bits & 0x1:
                raise DeploymentError("deployment archive contains a symlink or encrypted entry")
            total_size += info.file_size
            if total_size > MAX_ARCHIVE_BYTES:
                raise DeploymentError("deployment archive is too large")
            target = destination.joinpath(*path.parts)
            if info.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open(info, "r") as input_stream, target.open("wb") as output_stream:
                shutil.copyfileobj(input_stream, output_stream)
            target.chmod(0o644)


def validate_archive(
    archive: Path,
    *,
    check_linkage: bool = False,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="humanoid_plugin_validate_") as temporary:
        root = Path(temporary)
        _safe_extract(archive.resolve(), root)
        return validate_tree(
            root,
            check_linkage=check_linkage,
        )


def pack_directory(source: Path, output: Path) -> Path:
    source = source.resolve()
    if not (source / "manifest.yaml").is_file():
        raise DeploymentError("bundle source is missing manifest.yaml")
    with tempfile.TemporaryDirectory(prefix="humanoid_plugin_pack_") as temporary:
        staged = Path(temporary) / "bundle"
        # Development installs commonly use colcon --symlink-install. A deployment archive must
        # be self-contained, so dereference those links while staging and reject links thereafter.
        shutil.copytree(source, staged, symlinks=False)
        write_checksums(staged)
        validate_tree(staged, check_linkage=False)
        output = output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
        )
        os.close(descriptor)
        temporary_archive = Path(temporary_name)
        try:
            with zipfile.ZipFile(
                temporary_archive, "w", compression=zipfile.ZIP_DEFLATED, allowZip64=True
            ) as archive:
                for path in sorted(staged.rglob("*")):
                    if path.is_file():
                        archive.write(path, path.relative_to(staged).as_posix())
            os.replace(temporary_archive, output)
        finally:
            if temporary_archive.exists():
                temporary_archive.unlink()
    return output


def _deployed_path(plugin_root: Path, plugin_type: str, plugin_id: str) -> Path:
    if plugin_type not in _TYPE_DIR:
        raise DeploymentError("unsupported plugin type")
    path = plugin_root.resolve() / _TYPE_DIR[plugin_type] / _require_id(plugin_id)
    if path.is_symlink() or not path.is_dir():
        raise DeploymentError(f"plugin is not deployed: {plugin_id}")
    return path


def _validate_robot_driver_selection(plugin_root: Path, robot: dict[str, Any]) -> None:
    driver = robot["driver"]
    if driver["source"] != "deployed":
        return
    driver_root = _deployed_path(plugin_root, "hardware_driver", driver["plugin_id"])
    hardware = validate_tree(driver_root)
    if hardware["plugin_class"] != driver["plugin_class"]:
        raise DeploymentError("deployed hardware plugin class differs from robot manifest")


def _validate_robot_dependents(
    plugin_root: Path,
    hardware_id: str,
    hardware_class: str,
) -> None:
    deployed_robots = plugin_root / _TYPE_DIR["robot_profile"]
    if not deployed_robots.is_dir():
        return
    for path in deployed_robots.iterdir():
        if path.is_symlink() or not path.is_dir():
            raise DeploymentError(f"deployed robot path is invalid: {path}")
        robot = validate_tree(path)
        driver = robot["driver"]
        if (
            driver["source"] == "deployed"
            and driver["plugin_id"] == hardware_id
            and driver["plugin_class"] != hardware_class
        ):
            raise DeploymentError(
                f"hardware class is incompatible with deployed robot '{robot['plugin_id']}'"
            )


def deploy_archive(
    archive: Path,
    plugin_root: Path,
    *,
    check_linkage: bool = True,
) -> Path:
    plugin_root = plugin_root.resolve()
    staging_parent = plugin_root / ".staging"
    staging_parent.mkdir(parents=True, exist_ok=True)
    incoming: Path | None = Path(tempfile.mkdtemp(prefix="deploy_", dir=staging_parent))
    replaced_container: Path | None = None
    replaced: Path | None = None
    destination: Path | None = None
    try:
        _safe_extract(archive.resolve(), incoming)
        manifest = validate_tree(incoming, check_linkage=check_linkage)
        plugin_type = manifest["plugin_type"]
        plugin_id = manifest["plugin_id"]
        if plugin_type == "robot_profile":
            _validate_robot_driver_selection(plugin_root, manifest)
        else:
            _validate_robot_dependents(
                plugin_root, plugin_id, manifest["plugin_class"]
            )

        destination = plugin_root / _TYPE_DIR[plugin_type] / plugin_id
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink() or (destination.exists() and not destination.is_dir()):
            raise DeploymentError(f"deployment target is not a managed directory: {destination}")
        if destination.exists():
            replaced_container = Path(
                tempfile.mkdtemp(prefix="replaced_", dir=staging_parent)
            )
            replaced = replaced_container / "old_content"
            os.replace(destination, replaced)
        try:
            os.replace(incoming, destination)
            incoming = None
        except OSError:
            if replaced is not None and replaced.exists():
                os.replace(replaced, destination)
            raise
        return destination
    finally:
        if incoming is not None and incoming.exists():
            shutil.rmtree(incoming, ignore_errors=True)
        if replaced_container is not None and replaced_container.exists():
            shutil.rmtree(replaced_container, ignore_errors=True)


def resolve_robot_deployment(
    plugin_root: Path,
    robot_id: str,
) -> ResolvedDeployment:
    plugin_root = plugin_root.resolve()
    robot_root = _deployed_path(plugin_root, "robot_profile", robot_id)
    robot = validate_tree(robot_root)
    driver = robot["driver"]
    xml_paths: tuple[Path, ...] = ()
    prefixes: list[Path] = []
    library_paths: list[Path] = []
    if robot.get("_ament_prefix") is not None:
        prefixes.append(robot["_ament_prefix"])

    if driver["source"] == "deployed":
        driver_root = _deployed_path(plugin_root, "hardware_driver", driver["plugin_id"])
        hardware = validate_tree(driver_root)
        if hardware["plugin_class"] != driver["plugin_class"]:
            raise DeploymentError("deployed hardware plugin class differs from robot manifest")
        prefixes.insert(0, hardware["_ament_prefix"])
        library_paths.append(hardware["_ament_prefix"] / "lib")
        xml_paths = (hardware["_plugin_xml"],)

    return ResolvedDeployment(
        robot_id=robot["plugin_id"],
        name=robot["name"],
        manifest_path=robot_root / "manifest.yaml",
        resources=robot["_resources"],
        driver_class=driver["plugin_class"],
        driver_plugin_xml_paths=xml_paths,
        ament_prefixes=tuple(prefixes),
        library_paths=tuple(library_paths),
    )


def list_deployed(plugin_root: Path) -> dict[str, Any]:
    plugin_root = plugin_root.resolve()
    result: dict[str, Any] = {"hardware_drivers": {}, "robot_profiles": {}}
    for plugin_type, directory in _TYPE_DIR.items():
        store = plugin_root / directory
        if not store.is_dir():
            continue
        for plugin_dir in sorted(path for path in store.iterdir() if path.is_dir()):
            result[directory][plugin_dir.name] = {"path": str(plugin_dir.resolve())}
    return result


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2)
