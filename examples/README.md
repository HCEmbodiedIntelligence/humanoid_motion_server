# OpenArmX Python 控制示例

本目录是 ROS 2 客户端示例，可以整体复制到另一台电脑；请保留 `_common.py`。
脚本使用 OpenArmX 的七关节命名和默认接口，不负责启动运动服务或硬件驱动。

## 1. 接收方需要什么

- Ubuntu 22.04、ROS 2 Humble，以及 `rclpy`、`action_msgs`、`geometry_msgs`、`sensor_msgs`。
- 运行 MoveJ / MoveL 还需要与服务端一致、已构建的 `humanoid_motion_interfaces` 包。
- ServoP、ServoJ、joint_cmd 使用 ROS 标准消息，不需要自定义消息包。
- 机器人电脑运行驱动和运动服务，并持续发布有效的 `/hc_teleop/joint_states`。
- MoveL / ServoP 还需对应臂的 `/teleop/{left_arm,right_arm}/fk_pose` 反馈。
- ServoJ 需服务端配置 `/teleop/{left_arm,right_arm}/servo_j` 通道。

接收方已有整套工作区时，直接 source 其 `install/setup.bash`。
若接收方只装了 ROS 2，还应把 **`humanoid_motion_interfaces` 完整源码目录**一并交付，
放入接收方工作区的 `src/`，然后构建（需已安装 colcon 和 ROS 消息生成工具）：

```bash
source /opt/ros/humble/setup.bash
cd ~/motion_demo_ws  # 此工作区的 src/ 中已放入 humanoid_motion_interfaces
colcon build --packages-select humanoid_motion_interfaces
source install/setup.bash
```

客户端不需要安装运动 SDK 或硬件驱动；这些由机器人电脑的服务端运行。

## 2. 设置连接环境

每次打开新终端时执行，路径按接收方自己的目录修改：

```bash
source /opt/ros/humble/setup.bash
source ~/motion_demo_ws/install/setup.bash  # 或已有整套工作区的 install/setup.bash
export ROS_DOMAIN_ID=14                    # 必须与目标服务端一致；本机仿真默认 199
export ROS_LOCALHOST_ONLY=0                # 跨电脑连接时，两端均须允许非本机通信
cd /path/to/examples                       # 改成收到的 examples 目录
```

跨电脑时需 ROS 2/DDS 网络可互通并同步系统时钟；无需在脚本里填写机器人 IP。
本机独立仿真若使用 `ROS_LOCALHOST_ONLY=1`，客户端也使用 1，并使用同一 ROS 域。
仅运行 Topic 示例且未构建消息包时，可以省略上面的工作区 source。

先确认服务端和新鲜反馈：

```bash
ros2 action list -t
ros2 topic info /teleop/left_arm/servo_j -v
ros2 topic echo /hc_teleop/joint_states --once --qos-reliability best_effort
```

ServoJ 应有运动节点订阅者；关节反馈必须有对应臂的关节，并持续更新时间戳。

## 3. 五个示例

| 脚本 | 接口与行为 |
| --- | --- |
| `move_j.py` | MoveJ Action，一次关节目标，等待到位结果 |
| `move_l.py` | MoveL Action，一次末端直线目标，保持当前姿态 |
| `servo_p.py` | ServoP Topic，持续发布末端目标，由服务端平滑和求解 IK |
| `servo_j.py` | ServoJ Topic，持续发布关节目标，由服务端执行原有 RTC 平滑 |
| `joint_cmd.py` | `/hc_teleop/joint_cmd`，直接发送关节位置与速度，示例自身做五次插值 |

以下示例会下发运动目标；根据当前位置选择偏移量，同一臂一次只运行一个控制源：

```bash
python3 move_j.py --arm left --joint 4 --delta 0.02 --scale 0.1
python3 move_l.py --arm left --dx 0.005 --scale 0.1
python3 servo_p.py --arm left --dx 0.005 --rate 100 --duration 3
python3 servo_j.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
# 直接发 joint_cmd 前停止其他关节指令发送源：
python3 joint_cmd.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
```

- `--arm right` 选择右臂；关节角和 `--delta` 单位 rad，`--dx/--dy/--dz` 单位 m。
- MoveJ、ServoJ、joint_cmd 支持 `--positions q1 q2 q3 q4 q5 q6 q7`，为绝对关节角，与 `--delta` 互斥。
- MoveL / ServoP 保留 FK 坐标系及姿态，只在启动时加一次位置偏移，不逐帧累加。
- 不填位移参数时目标为当前实测位置。完整参数可用 `python3 servo_j.py --help` 等查看。
- 切换 Servo 控制源后先等待其 lease 失效；本机当前为 100 ms，并应留出调度余量。
- `joint_cmd` 绕过运动服务的平滑、限位检查和仲裁；通常优先使用带服务端处理的 ServoJ。
- Ctrl+C：Move 示例请求取消并等待终态；Topic 示例停止发布，由服务端或驱动处理断流。
- 默认等待接口/反馈最多 5 s；持续发布时反馈过期或发布间隔超过 100 ms 会退出。Python 发布不保证硬实时。

这五个示例已完成隔离运动学仿真验证；该验证不代表真机运动验收。
