# humanoid_motion_server

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

## 构建

本包包含固定版本的 `vendor/robo_manip` 运动 SDK。构建前需准备其 ABI 固定的依赖：

```bash
source /opt/ros/humble/setup.bash
cd /home/czy/teleop_ws
sudo ./install_sdk_dependencies_system.sh
colcon build --packages-up-to humanoid_motion_server
```
