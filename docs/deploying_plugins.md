# Web 免编译插件部署（调试期）

## 架构边界

目标机只固定安装和编译以下平台核心：

- `humanoid_driver_interface`
- `humanoid_driver_runtime`
- `humanoid_motion_interfaces`
- `humanoid_motion_server`

`humanoid_motion_server` 固定使用核心构建时集成的 `robo_manip`，不从 Web 更新算法库。Web 只部署：

1. `hardware_driver`：在匹配的 ROS 2 Humble 架构上提前编译好的厂商驱动共享库；
2. `robot_profile`：URDF、driver/motion/SDK/channel/tool YAML、可选 mesh 和 teleop 配置。

目标机不运行 `colcon build`。资源包中的 `prefix` 是随 ZIP 携带的最小 ament 前缀，用来解析
`package://` 和 pluginlib 元数据，不要求目标机安装 description 或驱动源码包。Cartesian、FK 和 IK
仍需要包含运动链、关节轴和 frame 的 URDF；visual/collision mesh 可以省略。

## 调试期部署语义

当前没有 release、部署版本、签名、激活或回退概念。默认根目录为
`/var/lib/humanoid-plugins`：

```text
/var/lib/humanoid-plugins/
  hardware_drivers/<plugin-id>/
  robot_profiles/<plugin-id>/
  .staging/                         # 上传校验和目录替换的临时空间
```

`deploy` 校验 ZIP 后直接写入上述目录；再次部署相同 `plugin_id` 会覆盖原目录，不保留旧副本。部署时应
先停止使用该插件的 driver/motion 进程，部署完成后重新启动。代码不做进程内热加载。

manifest 中没有 `plugin_version`。硬件 ZIP 内 `package.xml` 的 `<version>` 仍然存在，因为它是 ROS 2
package manifest 的必填字段；部署器不读取它，也不把它当作 release 版本。

## 硬件驱动包

ZIP 根目录结构：

```text
manifest.yaml
checksums.sha256
prefix/
  lib/libmy_robot_driver.so
  share/my_robot_driver/package.xml
  share/my_robot_driver/plugins/driver_plugins.xml
  share/ament_index/resource_index/packages/my_robot_driver
```

`checksums.sha256` 只用于发现上传损坏或内容缺失，由 `pack` 自动生成，不是签名。manifest 示例：

```yaml
schema_version: 1
plugin_type: hardware_driver
plugin_id: my_robot_driver
name: My robot driver
compatibility:
  ros_distro: humble
  architecture: x86_64       # aarch64 目标使用 aarch64
  driver_interface_abi: 1
package_name: my_robot_driver
ament_prefix: prefix
plugin_xml: prefix/share/my_robot_driver/plugins/driver_plugins.xml
library: prefix/lib/libmy_robot_driver.so
plugin_class: my_robot_driver/MyRobotDriver
```

部署器校验平台/ABI、ELF、pluginlib 基类、包索引和校验和，并默认使用 `ldd -r` 检查动态依赖。插件
所需的非系统私有 `.so` 应放入 `prefix/lib`。启动时使用绝对 `plugin_xml_paths` 加载插件，并只给对应
进程添加包内 `AMENT_PREFIX_PATH` 和 `LD_LIBRARY_PATH`。

## 机器人资源包

资源包不含可执行代码。manifest 示例：

```yaml
schema_version: 1
plugin_type: robot_profile
plugin_id: my_robot_v1
name: My robot v1
driver:
  source: deployed
  plugin_id: my_robot_driver
  plugin_class: my_robot_driver/MyRobotDriver
ament_prefix: prefix             # 仅在 URDF 使用 package:// 时需要
resources:
  driver_params: resources/driver.yaml
  motion_params: resources/motion.yaml
  sdk_config: resources/sdk.yaml
  channel_config: resources/channels.yaml
  tool_config: resources/tools.yaml
  urdf: resources/robot.urdf
  # teleop_config: resources/teleop.toml
```

若使用核心自带的 `MockRobotDriver` 或 `RosTopicRobotDriver`，`driver.source` 写 `core` 并省略
`driver.plugin_id`；其他类必须声明为 `deployed`。部署机器人资源前要先部署其硬件驱动。验证器会交叉
检查 driver、motion、SDK 和 URDF 的逻辑关节集合/顺序，检查 tool/channel frame，并拒绝越界路径、
未知字段和不在包内的 `package://` 引用。

## 打包、部署与启动

开发机生成 ZIP：

```bash
ros2 run humanoid_motion_server humanoid_pluginctl.py pack STAGED_DIR bundle.zip
ros2 run humanoid_motion_server humanoid_pluginctl.py validate bundle.zip
```

目标机或 Web 后端直接部署。硬件包先于引用它的机器人资源包：

```bash
ros2 run humanoid_motion_server humanoid_pluginctl.py deploy my-driver.zip
ros2 run humanoid_motion_server humanoid_pluginctl.py deploy my-robot.zip
ros2 run humanoid_motion_server humanoid_pluginctl.py list
ros2 run humanoid_motion_server humanoid_pluginctl.py resolve my_robot_v1

ros2 launch humanoid_motion_server registered_robot.launch.py robot_id:=my_robot_v1
```

CLI 的 stdout/stderr 为 JSON，`deploy` 成功结果含 `restart_required: true`。Web 后端应限制上传大小、
使用随机临时文件、串行处理同一插件 ID，并以参数数组调用 CLI，不能拼接 shell 命令。上传或部署成功后
不要自动启动实体机器人。

## OpenArmX 参考产物

在开发机完成构建并 source 工作区后：

```bash
python3 src/openarmx_driver/tools/create_deployment_bundle.py \
  "$(ros2 pkg prefix openarmx_driver)" openarmx-driver.zip

python3 src/robot_bringup/tools/create_openarmx_robot_bundle.py \
  openarmx-v10-robot.zip
```

第二个 ZIP 包含预展开 URDF 与 description 资源，不要求目标机安装或编译
`openarmx_description`。默认不包含 `teleop_vr_recv.toml`；仅在目标机另行提供遥操作进程时使用
`--include-teleop`。
