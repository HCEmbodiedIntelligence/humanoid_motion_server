#!/usr/bin/env python3
"""Launch a robot stack from a package profile or a managed no-build deployment."""

from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_motion_server.deployment import (
    DEFAULT_PLUGIN_ROOT,
    resolve_robot_deployment,
)


_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_REQUIRED_RESOURCES = {
    "driver_params",
    "motion_params",
    "sdk_config",
    "channel_config",
    "tool_config",
    "urdf",
}
_OPTIONAL_RESOURCES = {"teleop_config"}


def _resolve_resource(value: object, profile_dir: Path, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise RuntimeError(f"profile resource '{label}' must be a non-empty string")

    if value.startswith("package://"):
        package_path = value[len("package://"):]
        package, separator, relative = package_path.partition("/")
        if not separator or not _SAFE_NAME.fullmatch(package) or not relative:
            raise RuntimeError(f"invalid package resource for '{label}': {value}")
        package_share = Path(get_package_share_directory(package)).resolve()
        resolved = (package_share / relative).resolve()
        try:
            resolved.relative_to(package_share)
        except ValueError as error:
            raise RuntimeError(
                f"profile resource '{label}' escapes package '{package}'"
            ) from error
    else:
        if Path(value).is_absolute():
            raise RuntimeError(
                f"profile resource '{label}' must be relative or use package://"
            )
        resolved = (profile_dir / value).resolve()
        try:
            resolved.relative_to(profile_dir.resolve())
        except ValueError as error:
            raise RuntimeError(
                f"profile resource '{label}' escapes its profile directory"
            ) from error

    if not resolved.is_file():
        raise RuntimeError(f"profile resource '{label}' does not exist: {resolved}")
    return resolved


def _launch_registered_robot(context):
    robot_package = LaunchConfiguration("robot_package").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    plugin_root = Path(LaunchConfiguration("plugin_root").perform(context)).resolve()

    if robot_id:
        if robot_package or robot_profile:
            raise RuntimeError(
                "robot_id managed deployment cannot be combined with robot_package/robot_profile"
            )
        deployment = resolve_robot_deployment(
            plugin_root,
            robot_id,
        )
        paths = deployment.resources
        driver_overrides = {
            "plugin_class": deployment.driver_class,
            "plugin_xml_paths": [
                str(path) for path in deployment.driver_plugin_xml_paths
            ],
        }
        additional_env = deployment.environment()
    else:
        if not robot_package or not robot_profile:
            raise RuntimeError(
                "select either robot_id or both robot_package and robot_profile"
            )
        paths = _resolve_package_profile(robot_package, robot_profile)
        driver_overrides = {}
        additional_env = {}

    return _launch_paths(context, paths, driver_overrides, additional_env)


def _resolve_package_profile(robot_package: str, robot_profile: str):
    if not _SAFE_NAME.fullmatch(robot_package):
        raise RuntimeError("robot_package must be a package name")
    if not _SAFE_NAME.fullmatch(robot_profile):
        raise RuntimeError("robot_profile must be a simple profile name")

    package_share = Path(get_package_share_directory(robot_package)).resolve()
    profile_dir = package_share / "config" / "humanoid_stack" / robot_profile
    manifest_path = profile_dir / "profile.yaml"
    if not manifest_path.is_file():
        raise RuntimeError(
            f"robot profile '{robot_profile}' is not registered by package "
            f"'{robot_package}': {manifest_path}"
        )

    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    if not isinstance(manifest, dict) or set(manifest) != {
        "schema_version", "name", "resources"
    }:
        raise RuntimeError(
            "robot profile must contain only schema_version, name, and resources"
        )
    if manifest["schema_version"] != 1:
        raise RuntimeError("unsupported robot profile schema_version")
    if not isinstance(manifest["name"], str) or not manifest["name"]:
        raise RuntimeError("robot profile name must be a non-empty string")
    resources = manifest["resources"]
    if not isinstance(resources, dict):
        raise RuntimeError("robot profile resources must be a mapping")
    resource_keys = set(resources)
    if not _REQUIRED_RESOURCES.issubset(resource_keys):
        missing = sorted(_REQUIRED_RESOURCES - resource_keys)
        raise RuntimeError(f"robot profile is missing resources: {', '.join(missing)}")
    unknown = resource_keys - _REQUIRED_RESOURCES - _OPTIONAL_RESOURCES
    if unknown:
        raise RuntimeError(f"robot profile has unknown resources: {', '.join(sorted(unknown))}")

    return {
        key: _resolve_resource(value, profile_dir, key)
        for key, value in resources.items()
    }


def _launch_paths(context, paths, driver_overrides, additional_env):
    start_driver = LaunchConfiguration("start_driver")
    start_motion = LaunchConfiguration("start_motion")
    start_teleop = LaunchConfiguration("start_teleop")

    actions = [
        Node(
            package="humanoid_driver_runtime",
            executable="humanoid_driver_runtime_node",
            name="humanoid_driver_runtime",
            output="screen",
            parameters=[str(paths["driver_params"]), driver_overrides],
            additional_env=additional_env,
            condition=IfCondition(start_driver),
        ),
        Node(
            package="humanoid_motion_server",
            executable="humanoid_motion_control_node",
            name="humanoid_motion_control",
            output="screen",
            parameters=[
                str(paths["motion_params"]),
                {
                    "channel_config_file": str(paths["channel_config"]),
                    "sdk_config_file": str(paths["sdk_config"]),
                    "tool_config_file": str(paths["tool_config"]),
                    "urdf_file": str(paths["urdf"]),
                },
            ],
            additional_env=additional_env,
            condition=IfCondition(start_motion),
        ),
    ]

    if "teleop_config" in paths:
        actions.append(
            Node(
                package="teleop_vr_recv",
                executable="teleop_vr_recv_node",
                name="teleop_vr_recv",
                output="screen",
                parameters=[{"config_file": str(paths["teleop_config"])}],
                additional_env=additional_env,
                condition=IfCondition(start_teleop),
            )
        )
    elif LaunchConfiguration("start_teleop").perform(context).lower() in {
        "1", "true", "yes", "on"
    }:
        raise RuntimeError("start_teleop is true but the profile has no teleop_config")

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_package",
            default_value="",
            description="Package exporting config/humanoid_stack/<profile>/profile.yaml",
        ),
        DeclareLaunchArgument(
            "robot_profile",
            default_value="",
            description="Registered robot profile directory name",
        ),
        DeclareLaunchArgument(
            "robot_id",
            default_value="",
            description="Deployed robot_profile plugin ID below plugin_root.",
        ),
        DeclareLaunchArgument(
            "plugin_root",
            default_value=str(DEFAULT_PLUGIN_ROOT),
            description="Managed deployment root used with robot_id.",
        ),
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument("start_motion", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="false"),
        OpaqueFunction(function=_launch_registered_robot),
    ])
