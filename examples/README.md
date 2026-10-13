# OpenArmX Python 控制示例

本目录前五个脚本是 ROS 2 客户端示例，可以整体复制到另一台电脑；请保留 `_common.py`。
它们使用 OpenArmX 的七关节命名和默认接口，不负责启动运动服务或硬件驱动。
新增 `keyboard_servo_p.py` 使用已安装的 `humanoid_motion_server` Python 包，
从机器人配置选择通道和关节名称，可用于新机器的方向检查，使用方法见第 4 节。

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
`JointState.name` 不得为空或重复，`position`、`velocity` 必须与 `name` 等长且数值有限，
与运动服务对驱动反馈的要求一致。

## 3. 五个示例

| 脚本 | 接口与行为 |
| --- | --- |
| `move_j.py` | MoveJ Action，一次关节目标，等待到位结果 |
| `move_l.py` | MoveL Action，一次末端直线目标，可设置位置和姿态偏移 |
| `servo_p.py` | ServoP Topic，持续发布末端目标，由服务端平滑和求解 IK |
| `servo_j.py` | ServoJ Topic，持续发布关节目标，由服务端执行原有 RTC 平滑 |
| `joint_cmd.py` | `/hc_teleop/joint_cmd`，直接发送关节位置与速度，示例自身做五次插值 |

以下示例会下发运动目标；根据当前位置选择偏移量，同一臂一次只运行一个控制源：

```bash
python3 move_j.py --arm left --joint 4 --delta 0.02 --scale 0.1
python3 move_l.py --arm left --dx 0.005 --scale 0.1
python3 servo_p.py --arm left --dx 0.005 --rate 100 --duration 3
# 保持末端位置，绕 FK 参考坐标系 Z 轴转 0.1 rad（约 5.73°）：
python3 servo_p.py --arm left --dyaw 0.1 --rate 100 --duration 3
# 同时改变位置和姿态：
python3 servo_p.py --arm left --dx 0.005 --droll 0.05 --rate 100 --duration 3
python3 servo_j.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
# 直接发 joint_cmd 前停止其他关节指令发送源：
python3 joint_cmd.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
```

- `--arm right` 选择右臂；关节角和 `--delta` 单位 rad，`--dx/--dy/--dz` 单位 m。
- MoveJ、ServoJ、joint_cmd 支持 `--positions q1 q2 q3 q4 q5 q6 q7`，为绝对关节角，与 `--delta` 互斥。
- MoveL / ServoP 保留 FK 的 `header.frame_id`，支持 `--droll/--dpitch/--dyaw` 姿态增量，单位 rad，默认均为 0（保持当前姿态）。
  旋转按右手定则，依次绕该参考坐标系的固定 X、Y、Z 轴进行，不是绕工具自身轴；
  `R_target = Rz(dyaw) Ry(dpitch) Rx(droll) R_fk`，即 `q_target = q_delta ⊗ q_fk`。
  消息发送的是 `pose.orientation` 的四元数 `x/y/z/w`，不是直接把欧拉角填入其中。
- 位置和姿态偏移都只在启动时相对当前 FK 计算一次，不逐帧累加。只填角度时末端目标位置保持不变。
- 角度参数最终生成一个姿态目标，`2π` 与 `0` 表示相同朝向；连续转圈需另外提供连续轨迹。
- 不填任何位移或角度参数时目标为当前实测位姿。完整参数可用 `python3 servo_p.py --help` 等查看。
- ServoP / ServoJ 的 `--duration` 控制持续刷新目标的时长；实际到位时间取决于服务端限速和求解结果。
  Topic 示例退出仅表示结束发布，到位需检查反馈。`joint_cmd` 的 `--duration` 则是客户端插值时长，随后刷新最终位置和零速度 0.2 s。
- 切换 Servo 控制源后先等待其 lease 失效；本机当前为 100 ms，并应留出调度余量。
- `joint_cmd` 绕过运动服务的平滑、限位检查和仲裁；通常优先使用带服务端处理的 ServoJ。
- Ctrl+C：Move 示例请求取消并等待终态；Topic 示例停止发布，由服务端或驱动处理断流。
- 默认等待接口/反馈最多 5 s；持续发布时反馈过期或无效、指令数值无效、订阅者断开，或发布间隔超过 100 ms 会退出。
  卡顿跨过结束时间也会报错；`joint_cmd` 正常结束前必须发出最终目标帧。Python 发布不保证硬实时。

本轮验证通过 40 项离线测试和 19 个独立 ROS 运动学仿真场景，覆盖左右臂的五个示例、
ServoP 三轴姿态偏移、越界拒绝、取消及超时。验证未连接真机，也未覆盖跨电脑 DDS 通信。

## 4. 键盘控制与位姿方向检查

键盘入口为 `ros2 run humanoid_motion_server keyboard_servo_p`，
也可以运行本目录的 `python3 keyboard_servo_p.py`；两者使用同一份实现。
更新源码后先在工作区构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_motion_server
source install/setup.bash
```

以本机 OpenArmX 为例，终端 1 启动仿真和 Meshcat：

```bash
cd ~/workspace/teleop_ws
source install/setup.bash
source src/humanoid_motion_server/.simulation-venv/bin/activate
ros2 launch humanoid_motion_server simulation.launch.py \
  profile:=$PWD/src/humanoid_motion_server/config/simulation/openarmx.yaml \
  domain_id:=199 start_teleop:=false
```

浏览器打开 `http://127.0.0.1:7000/static/`。终端 2 运行键盘脚本：

```bash
cd ~/workspace/teleop_ws
source install/setup.bash
export ROS_DOMAIN_ID=199 ROS_LOCALHOST_ONLY=1
ros2 run humanoid_motion_server keyboard_servo_p --robot-id openarmx_01 --group left_arm
```

使用右臂时改为 `--group right_arm`。新机器在仿真与键盘两端使用相同的
`robot_id` 或 `profile`；通道、FK、参考坐标系、TCP、关节名均从配置读取。
`--list-channels` 列出通道，`--channel NAME` 可精确选择一个 ServoP 通道。
使用自定义 profile 时，键盘传 `--profile /绝对路径/simulation.yaml`。

脚本在交互终端运行，初始暂停；Enter 以当前实测 FK 建立目标。
默认只允许同一 ROS 域中、对应 robot_id 的运动学仿真；
`--allow-hardware` 可显式用于真实机器人接口。脚本本身不启动硬件。

| 按键 | 功能 |
| --- | --- |
| Enter | 从当前 FK 重新开始发布目标 |
| Space | 暂停发布，松开键盘控制源 |
| W / S | 沿 X 正 / 负方向平移 |
| A / D | 沿 Y 正 / 负方向平移 |
| R / F | 沿 Z 正 / 负方向平移 |
| U / J | 绕 X 正 / 负方向旋转 |
| I / K | 绕 Y 正 / 负方向旋转 |
| O / L | 绕 Z 正 / 负方向旋转 |
| T | 切换固定参考轴 / 当前目标工具自身轴 |
| C | 将目标重新对齐当前实测 FK |
| + / - | 加倍 / 减半平移和旋转步长 |
| Q 或 Ctrl+C | 停止发布并退出 |

默认每次按键移动 **5 mm / 2°**，长按使用终端键盘连发；发布频率为 100 Hz。
`--step` 单位 m，`--angle-step` 单位 deg。目标最多领先实测 0.1 m / 60°，
可用 `--max-offset` 和 `--max-angle` 调整；超过上限的按键不会累加目标。
反馈异常、仿真退出或发布中断超过 Servo lease 时自动暂停，需要 Enter 重新开始。
暂停再开始会重新取当前 FK，不继续执行旧目标。

Meshcat 自动显示所有配置了 FK 的 ServoP 通道：

- **蓝色小球 + 短坐标轴**：实测 FK/TCP。
- **橙色长方体 + 小球 + 长坐标轴**：ServoP 目标；小球中心就是目标 TCP，长方体沿目标 +X 伸出。
- **红 X、绿 Y、蓝 Z**：轴箭头指向正方向；参考坐标系原点另有一组更长的轴。
- **黄色线**：实测 TCP 到目标 TCP 的位置误差。
- **灰色目标**：已停止刷新，保留最后目标便于查看；过期的实测 FK 会隐藏。

两种位姿先按各自 `header.frame_id` 转到场景坐标系再显示。
参考坐标系必须能在显示 URDF 或 `tool_config` 中找到；找不到时隐藏标记并在
`/simulation/viewer_status` 的 `pose_markers` 中报告具体错误，不按世界坐标系强行显示。
终端同时显示双方的 XYZ、四元数 **xyzw**、ZYX 约定的 RPY（deg）及位置/角度误差。

检查方向时，先按 W、U 等键，观察固定参考轴下平移和右手旋转是否符合预期；
再按 T 切到工具轴，检查平移是否沿小球自身的 RGB 轴。
参考轴旋转使用 `q_delta ⊗ q_target`，工具轴使用 `q_target ⊗ q_delta`。

检查手柄时，在仿真启动命令中设置 `start_teleop:=true`，先暂停或退出键盘，再启用手柄。
显示器直接订阅实际 ServoP 通道，因此橙色标记会显示手柄经过 `axis_mapping`、相对参考和滤波后的机器人目标。
转动真实手柄并观察橙色 RGB 轴、终端四元数与蓝色 FK 轴，即可比较输入映射和机械臂跟随方向。
键盘暂停时仍可查看手柄目标；同一通道一次使用一个控制源。
