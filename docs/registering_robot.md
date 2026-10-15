# 注册新机器人

平台不维护需要修改源码的机器人枚举。ROS 2 的 ament package index 就是注册表：机器人包只要安装
`config/humanoid_stack/<profile>/profile.yaml`，通用启动器就能通过包名和 profile 名发现它。

## 1. 在机器人包中导出 profile

推荐把 profile 放在 description、bringup 或部署配置包中：

```text
my_robot_description/
  config/humanoid_stack/my_robot_v1/
    profile.yaml
    motion_control.yaml
    sdk.yaml
    channels.yaml
    tools.yaml
    teleop_vr_recv.toml       # 不需要遥操作时可省略
  urdf/my_robot.urdf
```

CMake 包需安装这些目录：

```cmake
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
```

`profile.yaml` schema 版本 1 只允许下面这些字段：

```yaml
schema_version: 1
name: My Robot v1
resources:
  driver_params: package://my_robot_driver/config/driver.yaml
  motion_params: motion_control.yaml
  sdk_config: sdk.yaml
  channel_config: channels.yaml
  tool_config: tools.yaml
  urdf: package://my_robot_description/urdf/my_robot.urdf
  teleop_config: teleop_vr_recv.toml
```

资源可以写成相对 profile 的路径，也可以使用 `package://<package>/<path>`。启动器会拒绝未知字段、
不存在的文件、绝对路径和越过包目录的路径，避免错误 profile 静默启动。

## 2. 保持四层关节定义一致

下列名称必须描述同一组逻辑关节：

1. 驱动 YAML 的 `joint_names`；
2. `motion_control.yaml` 和 `sdk.yaml` 的 group；
3. URDF 中实际存在的 joint；
4. 运动限位数组的顺序。

厂商名称、单位、方向和零位只放在驱动 mapping 中，不要泄漏到运动服务。URDF 可以包含未接入的
夹爪关节；驱动只反馈手臂时，TF/FK 会保留已知手臂子树并跳过未知夹爪子树。

## 3. 声明控制与 FK 话题

Cartesian channel 可以同时声明控制入口和测量 FK 输出：

```yaml
channels:
  - name: left_arm_servo_p
    kind: ServoP
    topic: /teleop/left_arm/servo_p
    group: left_arm
    base_frame: left_base
    tip_frame: left_tool0
    fk_pose_topic: /teleop/left_arm/fk_pose
    priority: 100
```

`fk_pose_topic` 发布 `geometry_msgs/msg/PoseStamped`，其数据由真实驱动反馈计算，不是命令回显。
`teleop_vr_recv.toml` 订阅同一 FK 话题，并把目标发布到该 channel 的 ServoP 话题。

## 4. 选择驱动，不修改 runtime

- 厂商已经给出 ROS 2 `sensor_msgs/msg/JointState` 状态和位置命令 topic：在 driver YAML 选择
  `humanoid_driver_runtime/RosTopicRobotDriver`，只配置 topic 和关节 mapping。
- 厂商只给 CAN、EtherCAT、串口、专有消息或 SDK：在独立驱动包实现并导出
  `humanoid_driver_interface::RobotDriverPlugin`，driver YAML 的 `plugin_class` 选择该插件。

驱动插件的实现和导出步骤见
`humanoid_driver_runtime/docs/adding_driver_plugin.md`。核心 runtime 不添加机器人判断或新源码。

## 5. 启动与局部启动

完整启动：

```bash
ros2 launch humanoid_motion_server registered_robot.launch.py \
  robot_package:=my_robot_description \
  robot_profile:=my_robot_v1
```

可以用 `start_driver:=false`、`start_motion:=false` 或 `start_teleop:=false` 关闭对应层，便于连接已有
进程或单独调试。若 profile 没有 `teleop_config`，启动时必须设置 `start_teleop:=false`。

## OpenArmX v10 双臂

OpenArmX 仿真按四层独立运行：MuJoCo 只暴露厂商关节 Topic，driver runtime
负责厂商接口与 `/hc_teleop/*` 的转换，motion server 提供 ServoP/FK，
`teleop_vr_recv` 自己持有 PICO UDP 配置。

OpenArmX 的部署 profile 位于 `robot_bringup`，几何模型仍位于
`openarmx_description`。工作区提供一键 launch：

```bash
source /home/czy/teleop_ws/install/setup.bash
ros2 launch robot_bringup openarmx_v10_bimanual.launch.py
```

对应的数据流为：

```text
PICO -> teleop_vr_recv -> /teleop/<arm>/servo_p -> humanoid_motion_server
     -> /hc_teleop/joint_cmd -> humanoid_driver_runtime
     -> /openarmx/vendor/joint_command -> openarmx_mujoco

openarmx_mujoco -> /openarmx/vendor/joint_states -> humanoid_driver_runtime
     -> /hc_teleop/joint_states -> humanoid_motion_server -> FK
```

若只需启动已注册的 driver + motion 层而不启动 VR 前端，可使用：

```bash
source /home/czy/teleop_ws/install/setup.bash
ros2 launch humanoid_motion_server registered_robot.launch.py \
  robot_package:=robot_bringup \
  robot_profile:=openarmx_v10_bimanual \
  start_teleop:=false
```
