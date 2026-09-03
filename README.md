# humanoid_motion_server

## 环境与编译（首次使用先执行）

运行环境固定为 **Ubuntu 22.04 x86-64 + ROS 2 Humble + GCC 11 + C++17**。
工作区至少需要同时包含 `humanoid_motion_interfaces` 和本仓库；运行 mock 驱动还需要
`humanoid_driver_interface`、`humanoid_driver_runtime`。

本包自带二进制 `robo_manip` SDK，但它依赖固定版本的 Ruckig、TOPPRA、NLopt、
TRAC-IK、eiquadprog、hpp-fcl、Pinocchio 和 OctoMap。不能用 `pip install toppra` 代替
C++ 依赖。安装脚本还会先安装固定 commit 的 `jrl-cmakemodules`，避免上述项目在 CMake
配置阶段通过 `FetchContent` 发起不受控的临时网络下载。

这些库都是普通系统依赖，不属于 `humanoid_motion_server` 的私有运行环境。源码构建版本按
Linux 的标准本地安装布局写入 `/usr/local/include`、`/usr/local/lib` 及各库自己的
`/usr/local/lib/cmake/<package>`（或 `share/<package>`）目录；安装后运行 `ldconfig`。
不创建专用依赖前缀，也不需要 source 额外的环境脚本。

### 1. 从 `alg_dep` 源码安装 SDK 依赖（推荐）

把 `/home/czy/ik_demo/alg_dep` 整个目录复制到目标电脑，然后运行：

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/install_sdk_dependencies_ubuntu2204.sh \
  --source-root /path/to/alg_dep \
  --jobs 2
```

`--source-root` 是源码输入目录，不是安装目录。脚本会从 `alg_dep` 读取固定 commit，在临时目录中
按正确顺序源码编译，并以普通系统库的形式安装到 `/usr/local`。Ubuntu 22.04 与 ROS 2 Humble
共用系统 Boost 1.74；脚本不会再编译一份 Boost，也不会修改 `/opt/ros/humble`。
脚本仅在安装文件和刷新动态链接器缓存时请求 sudo；不要复制
`alg_dep/*/build`；脚本需要权限时会自行提示输入 sudo 密码。

没有 `alg_dep` 时，也可以联网下载固定源码：

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/install_sdk_dependencies_ubuntu2204.sh --jobs 2
```

### 从旧版私有前缀迁移

旧机器若已经安装到 `/opt/local/humanoid_motion_server/sdk-deps`，不要直接复制其中的文件；CMake
配置可能仍带旧绝对路径。使用迁移脚本重新安装并在干净环境中验证：

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/migrate_sdk_dependencies_to_system.sh \
  --source-root /path/to/alg_dep \
  --jobs 2
```

确认 motion server 清缓存构建通过，并删除 `.bashrc`、systemd unit 或启动脚本中旧
`setup.bash` 的 source 行后，可再次运行并把旧目录改名留作回退：

```bash
./scripts/migrate_sdk_dependencies_to_system.sh \
  --verify-only \
  --archive-old
```

### 2. 验证并编译 ROS 包

```bash
cd <工作区>/src/humanoid_motion_server
./scripts/check_sdk_runtime \
  --sdk-root ./vendor/robo_manip

source /opt/ros/humble/setup.bash
cd <工作区>
rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-up-to humanoid_motion_server \
  --cmake-clean-cache

source install/setup.bash
```

如果只有最小源码包，`rosdep` 命令可添加
`--skip-keys "humanoid_driver_runtime teleop_vr_recv"`。出现
`toppraConfig.cmake` 报错时，说明系统依赖未装完整或仍在使用旧 CMake 缓存；重新执行安装脚本、
确认 `/usr/local/lib/cmake/toppra/toppraConfig.cmake` 存在，并保留首次编译命令中的
`--cmake-clean-cache`。

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

## 注册机器人插件

机器人型号、URDF 和厂商驱动不安装到目标机核心工作区。开发机分别生成预编译
`hardware_driver` 插件、只含资源的 `robot_model` 插件，以及只引用两者 ID 的
`robot_composition` 清单，由 `humanoid_adapter_manager` 校验和部署。通用启动命令只接受部署后的
`robot_id`：

```bash
ros2 launch robot_bringup registered_robot.launch.py \
  robot_id:=my_robot_v1 \
  start_teleop:=false
```

每个真机驱动在独立源码包中实现并预编译为 `RobotDriverPlugin`，驱动参数随对应
`hardware_driver` 插件部署。完整 schema 和关节命名约束见
[`robot_bringup/docs/registering_robot.md`](../robot_bringup/docs/registering_robot.md)。

驱动节点也可以单独启动。接入已有 ROS 2 关节 Topic 的机器人，请看
`humanoid_driver_runtime/docs/adding_ros_topic_robot.md`。
