from __future__ import annotations

from pathlib import Path
import platform
import shutil
import zipfile

import pytest
import yaml

from humanoid_motion_server.deployment import (
    DeploymentError,
    deploy_archive,
    list_deployed,
    pack_directory,
    resolve_robot_deployment,
    validate_archive,
)


PLUGIN_CLASS = "fake_driver/FakeRobotDriver"


def _architecture() -> str:
    value = platform.machine().lower()
    return {"amd64": "x86_64", "arm64": "aarch64"}.get(value, value)


def _write_yaml(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _hardware_tree(root: Path) -> Path:
    package_share = root / "prefix/share/fake_driver"
    marker = root / "prefix/share/ament_index/resource_index/packages/fake_driver"
    library = root / "prefix/lib/libfake_driver.so"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("", encoding="utf-8")
    package_share.mkdir(parents=True, exist_ok=True)
    (package_share / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3"><name>fake_driver</name><version>1.0.0</version>
<description>test</description><maintainer email="test@example.com">test</maintainer>
<license>Proprietary</license></package>
""",
        encoding="utf-8",
    )
    plugins = package_share / "plugins/fake_driver_plugins.xml"
    plugins.parent.mkdir(parents=True, exist_ok=True)
    plugins.write_text(
        f"""<library path="fake_driver">
  <class name="{PLUGIN_CLASS}" type="fake_driver::FakeRobotDriver"
    base_class_type="humanoid_driver_interface::RobotDriverPlugin"/>
</library>
""",
        encoding="utf-8",
    )
    library.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile("/bin/true", library)
    manifest = {
        "schema_version": 1,
        "plugin_type": "hardware_driver",
        "plugin_id": "fake_driver",
        "name": "Fake test driver",
        "compatibility": {
            "ros_distro": "humble",
            "architecture": _architecture(),
            "driver_interface_abi": 1,
        },
        "package_name": "fake_driver",
        "ament_prefix": "prefix",
        "plugin_xml": "prefix/share/fake_driver/plugins/fake_driver_plugins.xml",
        "library": "prefix/lib/libfake_driver.so",
        "plugin_class": PLUGIN_CLASS,
    }
    _write_yaml(root / "manifest.yaml", manifest)
    return root


def _robot_tree(
    root: Path,
    *,
    name: str = "Test robot",
    sdk_joint_order: list[str] | None = None,
) -> Path:
    resources = root / "resources"
    _write_yaml(
        resources / "driver.yaml",
        {
            "humanoid_driver_runtime": {
                "ros__parameters": {
                    "plugin_class": PLUGIN_CLASS,
                    "joint_names": ["joint1", "joint2"],
                    "vendor_joint_names": ["Joint1", "Joint2"],
                    "vendor_joint_groups": ["arm", "arm"],
                }
            }
        },
    )
    _write_yaml(
        resources / "motion.yaml",
        {
            "humanoid_motion_control": {
                "ros__parameters": {
                    "joint_group_names": ["arm"],
                    "groups.arm": ["joint1", "joint2"],
                    "group_lower_limits.arm": [-1.0, -1.0],
                    "group_upper_limits.arm": [1.0, 1.0],
                }
            }
        },
    )
    _write_yaml(
        resources / "sdk.yaml",
        {
            "model_path": "robot.urdf",
            "joint_groups": {"arm": sdk_joint_order or ["joint1", "joint2"]},
        },
    )
    _write_yaml(
        resources / "channels.yaml",
        {
            "channels": [
                {
                    "name": "arm_move_j",
                    "kind": "move_j",
                    "endpoint": "/motion/arm/move_j",
                    "priority": 50,
                    "group": "arm",
                }
            ]
        },
    )
    _write_yaml(resources / "tools.yaml", {"tools": []})
    (resources / "robot.urdf").write_text(
        """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base"/><link name="link1"/><link name="link2"/>
  <joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    _write_yaml(
        root / "manifest.yaml",
        {
            "schema_version": 1,
            "plugin_type": "robot_profile",
            "plugin_id": "test_robot",
            "name": name,
            "driver": {
                "source": "deployed",
                "plugin_id": "fake_driver",
                "plugin_class": PLUGIN_CLASS,
            },
            "resources": {
                "driver_params": "resources/driver.yaml",
                "motion_params": "resources/motion.yaml",
                "sdk_config": "resources/sdk.yaml",
                "channel_config": "resources/channels.yaml",
                "tool_config": "resources/tools.yaml",
                "urdf": "resources/robot.urdf",
            },
        },
    )
    return root


def test_deploy_resolve_and_direct_overwrite(tmp_path: Path) -> None:
    plugin_root = tmp_path / "deployed"
    hardware_archive = tmp_path / "hardware.zip"
    robot_archive = tmp_path / "robot.zip"
    replacement_archive = tmp_path / "replacement.zip"
    pack_directory(_hardware_tree(tmp_path / "hardware"), hardware_archive)
    pack_directory(_robot_tree(tmp_path / "robot"), robot_archive)
    pack_directory(
        _robot_tree(tmp_path / "replacement", name="Replacement robot"),
        replacement_archive,
    )

    manifest = validate_archive(hardware_archive)
    assert manifest["plugin_id"] == "fake_driver"
    assert "plugin_version" not in manifest

    with pytest.raises(DeploymentError, match="not deployed"):
        deploy_archive(robot_archive, plugin_root)

    hardware_path = deploy_archive(
        hardware_archive, plugin_root, check_linkage=False
    )
    robot_path = deploy_archive(robot_archive, plugin_root)
    assert hardware_path == plugin_root / "hardware_drivers/fake_driver"
    assert robot_path == plugin_root / "robot_profiles/test_robot"

    deployment = resolve_robot_deployment(plugin_root, "test_robot")
    assert deployment.driver_class == PLUGIN_CLASS
    assert deployment.name == "Test robot"
    assert deployment.driver_plugin_xml_paths[0].is_file()
    assert str(deployment.ament_prefixes[0]) in deployment.environment()["AMENT_PREFIX_PATH"]
    assert str(deployment.library_paths[0]) in deployment.environment()["LD_LIBRARY_PATH"]

    assert deploy_archive(replacement_archive, plugin_root) == robot_path
    assert resolve_robot_deployment(plugin_root, "test_robot").name == "Replacement robot"
    assert list_deployed(plugin_root) == {
        "hardware_drivers": {
            "fake_driver": {"path": str(hardware_path.resolve())},
        },
        "robot_profiles": {
            "test_robot": {"path": str(robot_path.resolve())},
        },
    }
    assert not (plugin_root / "active").exists()
    assert not (plugin_root / "previous").exists()
    assert list((plugin_root / ".staging").iterdir()) == []


def test_robot_cross_configuration_mismatch_is_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "bad-robot.zip"
    with pytest.raises(DeploymentError, match="joint order differ"):
        pack_directory(
            _robot_tree(tmp_path / "bad-robot", sdk_joint_order=["joint2", "joint1"]),
            archive,
        )


def test_archive_path_traversal_is_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "traversal.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("../outside", "unsafe")
    with pytest.raises(DeploymentError, match="unsafe path"):
        validate_archive(archive)
