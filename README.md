# humanoid_motion_server

## 环境与编译（首次使用先执行）

运行环境固定为 **Ubuntu 22.04 x86-64 + ROS 2 Humble + GCC 11 + C++17**。
工作区至少需要同时包含 `humanoid_motion_interfaces` 和本仓库；运行 mock 驱动还需要
`humanoid_driver_interface`、`humanoid_driver_runtime`。

本包自带二进制 `robo_manip` SDK，但它依赖固定版本的 Ruckig、TOPPRA、NLopt、
TRAC-IK、eiquadprog、hpp-fcl、Pinocchio 和 OctoMap。不能用 `pip install toppra` 代替
C++ 依赖。

### 1. 从 `alg_dep` 源码安装 SDK 依赖（推荐）

把 `/home/czy/ik_demo/alg_dep` 整个目录复制到目标电脑，然后运行：

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/install_sdk_dependencies_ubuntu2204.sh \
  --source-root /path/to/alg_dep \
  --jobs 2
```

脚本会从 `alg_dep` 读取固定 commit，在临时目录中按正确顺序源码编译，并统一安装到
`/opt/humanoid_motion_server/sdk-deps`。不要对整个脚本使用 `sudo`，也不要复制
`alg_dep/*/build`；脚本需要权限时会自行提示输入 sudo 密码。

没有 `alg_dep` 时，也可以联网下载固定源码：

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/install_sdk_dependencies_ubuntu2204.sh --jobs 2
```

### 2. 验证并编译 ROS 包

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/check_sdk_runtime \
  --sdk-root ./vendor/robo_manip \
  --deps-prefix /opt/humanoid_motion_server/sdk-deps

source /opt/ros/humble/setup.bash
cd <工作区>
rosdep install --from-paths src --ignore-src -r -y

export HUMANOID_MOTION_SDK_DEPS_PREFIX=/opt/humanoid_motion_server/sdk-deps
colcon build \
  --packages-up-to humanoid_motion_server \
  --cmake-clean-cache

source install/setup.bash
```

如果只有最小源码包，`rosdep` 命令可添加
`--skip-keys "humanoid_driver_runtime teleop_vr_recv"`。出现
`toppraConfig.cmake` 报错时，说明依赖未装完整或仍在使用旧 CMake 缓存；重新执行安装脚本，
并保留首次编译命令中的 `--cmake-clean-cache`。

> `vendor/robo_manip` 是内部二进制交付，外部分发授权待确认。GitHub 仓库应保持私有，
> 详情见 [`vendor/robo_manip/NOTICE.md`](vendor/robo_manip/NOTICE.md)。

这个包只负责平台统一的运动控制，不再加载机器人驱动，也不再访问厂商 Topic 或 SDK。

它根据 `channels.yaml` 创建 MoveJ、MoveL、MoveP、ServoJ、ServoP 接口，完成运动学、
轨迹生成、优先级仲裁和安全检查。最终关节命令发布到 `/hc_teleop/joint_cmd`，真实关节
反馈只从 `/hc_teleop/joint_states` 读取。

默认上层接口包括：

- `/teleop/servo_j`
- `/teleop/servo_p`
- `/normal/move_j`
- `/normal/move_l`
- `/normal/move_p`

消息和 Action 的类型定义来自 `humanoid_motion_interfaces`；真正创建并处理这些接口的是
本包中的 `humanoid_motion_control_node`。

Cartesian 通道可以配置 `fk_pose_topic`。节点会用真实
`/hc_teleop/joint_states` 计算 `base_frame -> tip_frame` 的 FK，并以
`geometry_msgs/msg/PoseStamped` 持续发布，供相对遥操作在 deadman 绑定时读取真实末端起点。

## 与驱动层的边界

```text
上层程序
  -> Move/Servo 接口
  -> humanoid_motion_control_node
  -> /hc_teleop/joint_cmd
  -> humanoid_driver_runtime_node
  -> 机器人原生 Topic 或机器人 SDK

机器人真实反馈
  -> humanoid_driver_runtime_node
  -> /hc_teleop/joint_states
  -> humanoid_motion_control_node
```

机器人更换后，本包的接口名和代码不需要修改。机器人原生关节名、方向、单位、零位和
原生通信方式都放在 `humanoid_driver_runtime` 的机器人配置或驱动插件中。

## 启动

使用模拟驱动启动完整链路：

```bash
ros2 launch humanoid_motion_server mock.launch.py
```

使用真实机器人时，为 `driver_params_file` 指定该机器人的驱动配置：

```bash
ros2 launch humanoid_motion_server bringup.launch.py \
  driver_params_file:=/path/to/robot_driver.yaml \
  sdk_config_file:=/path/to/robot_motion.yaml \
  urdf_file:=/path/to/robot.urdf
```

## 注册机器人配置包

生产部署推荐由机器人自己的 ROS 包导出下面的固定目录：

```text
<package-share>/config/humanoid_stack/<profile>/
  profile.yaml
  motion_control.yaml
  sdk.yaml
  channels.yaml
  tools.yaml
  teleop_vr_recv.toml        # 可选
```

`profile.yaml` 只保存资源路径；路径可以相对当前 profile，也可以使用
`package://<package>/<path>` 引用其他包。通用启动器不包含机器人名称或关节名称：

```bash
ros2 launch humanoid_motion_server registered_robot.launch.py \
  robot_package:=robot_bringup \
  robot_profile:=openarmx_v10_bimanual \
  start_teleop:=false
```

OpenArmX 的部署 profile 和 PICO/UDP 参数都安装在 `robot_bringup`，机器人几何模型
仍由 `openarmx_description` 提供。一键仿真遥操作请运行：

```bash
ros2 launch robot_bringup openarmx_v10_bimanual.launch.py
```

该 launch 会启动 MuJoCo、driver runtime、motion server 和 VR frontend 四层。

新增机器人只需安装新的 profile 和资源文件。ROS Topic 型机器人选择现成
`humanoid_driver_runtime/RosTopicRobotDriver`；专有协议驱动则在独立包中实现并导出
`RobotDriverPlugin`，然后在该 profile 引用的 driver YAML 中选择 `plugin_class`。
完整注册 schema、关节命名约束和 OpenArmX 启动步骤见
[`docs/registering_robot.md`](docs/registering_robot.md)。

驱动节点也可以单独启动。接入已有 ROS 2 关节 Topic 的机器人，请看
`humanoid_driver_runtime/docs/adding_ros_topic_robot.md`。
