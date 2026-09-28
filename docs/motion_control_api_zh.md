# OpenArmX 运动控制接口与 Python Demo

适用：本机 `openarmx_01`、ROS 2 Humble；核对日期：2026-09-24。
**本机配置已启用带服务端平滑的 ServoJ。`joint_cmd` 是驱动侧的连续关节指令入口，两者处理流程不同。**

## 1. 五个控制入口

| 功能 | 左臂接口（右臂把 left 改成 right） | 类型 |
| --- | --- | --- |
| MoveJ：一次关节目标 | `/motion/left_arm/move_j` | Action：`humanoid_motion_interfaces/action/MoveJ` |
| MoveL：一次末端直线目标 | `/motion/left_arm/move_l` | Action：`humanoid_motion_interfaces/action/MoveL` |
| ServoP：连续末端目标 | `/teleop/left_arm/servo_p` | Topic：`geometry_msgs/msg/PoseStamped` |
| ServoJ：连续关节目标，服务端平滑 | `/teleop/left_arm/servo_j` | Topic：`sensor_msgs/msg/JointState` |
| joint_cmd：连续关节指令 | `/hc_teleop/joint_cmd`（双臂共用） | Topic：`sensor_msgs/msg/JointState` |

遥操作链路：`手柄 → ServoP Topic → 笛卡尔平滑 / IK → 关节 RTC 平滑和限位检查 → joint_cmd Topic → 驱动`。
ServoP 内部用 `StartRealtimeMoveLine / TickRealtimeMoveLine` 连续更新目标，不逐帧发送 MoveJ Action。
上述 SDK/RTC 平滑是原有实现；本次新增通道配置和 Demo，未修改运动服务的平滑算法。
ServoJ 原有调用链：`JointState 目标 → ServoJ 会话 → TickRealtimeMoveJoint 的 RTC 平滑 → 最终关节 RTC → joint_cmd → 驱动`。
需要“连续关节目标 + 服务端平滑”时，使用 ServoJ，无需另写客户端插值来补齐该能力。
本机左右臂 `kind: servo_j` 通道优先级均为 50，与 Move/ServoP 相同；启动运动服务时自动创建。
`joint_cmd` 已在使用，直接送驱动，绕过运动服务的轨迹平滑、限位检查和控制权仲裁。
MoveJ 用于一次到位、回位和关节点动；MoveL 用于一次末端直线运动。
另有 `/motion/{left_arm,right_arm}/move_p`，类型 `MoveP`，Goal 同 MoveL；末端路径不保证直线。

## 2. 消息怎么填

- 左臂 `group_name=left_arm`、`tip_frame=left_tool0`；右臂对应 `right_arm`、`right_tool0`。
- 关节名：`openarmx_left_joint1`～`openarmx_left_joint7`；右臂替换 `left` 为 `right`。
- `JointState.name` 与 `position` 一一对应；位置 **rad**，`velocity` 为 **rad/s**；位置是绝对目标。
- MoveJ Goal：`group_name`、`target: JointState`、`options`；`target.velocity/effort` 不作为运动目标使用。
- ServoJ 直接发 `JointState`，用 `name/position` 表示目标；`velocity/effort` 不作为目标使用，无需发送 `options`。
- MoveL Goal：`group_name`、`tip_frame`、`target_pose: PoseStamped`、`options`。
- ServoP 直接发 `PoseStamped`：位置 **m**，姿态为单位四元数 **x,y,z,w**，均为绝对目标。
- 本机 ServoP/FK 的 `header.frame_id=openarmx_body_link0`；MoveL 也接受该坐标系。
- 保持姿态时复制当前 FK 的四元数；不要把单位四元数当作“保持姿态”，也不要只改坐标系名称。
- `options.velocity_scale/acceleration_scale/jerk_scale` 非零范围 `(0,1]`；**0 表示配置默认上限，不是停止**。
- `options.timeout_sec` 单位 s；0 使用服务端默认 60 s。Demo 默认比例 0.1、超时 30 s。
- Move 成功必须同时满足 Action `SUCCEEDED` 和 `result.status.code == 0`；Goal accepted 不代表到位。

## 3. 反馈、频率与控制交接

- 关节反馈：`/hc_teleop/joint_states`，`JointState`；按 `name` 查找关节，不假设数组顺序。
- 末端反馈：`/teleop/left_arm/fk_pose`、`/teleop/right_arm/fk_pose`，`PoseStamped`；订阅使用 Best Effort。
- ServoP/ServoJ 推荐 **100 Hz**，QoS：Best Effort、Volatile、Keep Last 1；每帧刷新 ROS 时间戳。
- ServoP/ServoJ 当前 lease 为 **100 ms**；包含传输耗时，重复、乱序、过期目标被拒绝；停止发布后 lease 到期失效。
- joint_cmd Demo 使用 **100 Hz**、Reliable、Volatile、Keep Last 1；每帧发送新时间戳和连续关节指令。
- joint_cmd 由驱动检查时间戳和断流；它不提供 ServoP lease 或 Action 到位结果，不能照搬 ServoP 的停止语义。
- 同臂 Move/ServoP/ServoJ 当前优先级相同，新有效命令可以抢占；切换接口前停止旧发送源，等待至少 100 ms 并留调度余量。
- **直接使用 joint_cmd 前停止其他关节指令发送源**；运动服务可保持空闲，但不能同时执行 Move/Servo。
- 直接关节控制需自行保证目标、速度和轨迹符合模型限位；示例插值不能替代完整的运动安全检查。
- Move Demo 按 Ctrl+C 会请求取消并等待终态；Topic Demo 按 Ctrl+C 停止发布。软件结束不等于机械急停。

## 4. Python Demo

先启动对应机器人或仿真；下面命令会实际下发运动目标，偏移量按当前位置选择。
仿真可参考 [仿真说明](simulation_zh.md)，使用 `start_teleop:=false` 避免手柄同时发送目标。

```bash
source /opt/ros/humble/setup.bash
source /home/hc_op/workspace/teleop_ws/install/setup.bash
export ROS_DOMAIN_ID=199   # 仿真默认 199；真机改为实际域
export ROS_LOCALHOST_ONLY=1  # 同机仿真；跨机器须与服务端设置一致
cd /home/hc_op/workspace/teleop_ws/src/humanoid_motion_server/examples

python3 move_j.py --arm left --joint 4 --delta 0.02 --scale 0.1
python3 move_l.py --arm left --dx 0.005 --scale 0.1
python3 servo_p.py --arm left --dx 0.005 --rate 100 --duration 3
python3 servo_j.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
# joint_cmd 单独运行，先确保其他发送源已停止：
python3 joint_cmd.py --arm left --joint 4 --delta 0.02 --rate 100 --duration 3
```

- [move_j.py](../examples/move_j.py)：读取新鲜关节反馈，指定关节加一次 `delta`，发送 MoveJ 并检查结果。
- [move_l.py](../examples/move_l.py)：当前 FK 加一次 `dx/dy/dz`，保持姿态，发送 MoveL 并检查结果。
- [servo_p.py](../examples/servo_p.py)：当前 FK 加一次偏移，持续发布固定目标；不会每帧累加位移。
- [servo_j.py](../examples/servo_j.py)：当前关节角加一次偏移，持续发布目标，由服务端完成平滑；客户端不插值。
- [joint_cmd.py](../examples/joint_cmd.py)：本次新增五次插值示例，连续下发整臂位置与速度，末尾保持 0.2 s；上游已有连续指令时可直接逐帧发布，无需此插值。
- MoveJ、ServoJ 和 joint_cmd 也支持 `--positions q1 q2 q3 q4 q5 q6 q7`，表示绝对角度 rad，与 `--delta` 互斥。
- MoveL 和 ServoP 支持 `--dx/--dy/--dz`，单位 m，沿当前 FK 的参考坐标系偏移；右臂使用 `--arm right`。
- 无位移参数时保持当前目标；完整参数见各脚本 `--help`。这些示例针对 OpenArmX 的七关节命名。
- [共用代码](../examples/_common.py) 负责反馈时效检查、QoS、发布调度和 Action 取消；`joint_cmd` 不依赖 FK。
- Python 定时发布不保证硬实时；Topic 输出遇到反馈过期或发布间隔超过 100 ms 会退出，不补发积压帧。
- 构建安装后，示例同时位于包的 `share/humanoid_motion_server/examples/`，可直接用 Python 运行。

## 5. 配置与源码

接口由 `channel_config_file` 创建；运行时用 `ros2 param get /humanoid_motion_control channel_config_file` 查询。
本机保存资源在 `~/.local/share/humanoid-plugins/robot_models/openarmx_01.model/resources/` 的 `channels.yaml`、`motion.yaml`。
已保存并应用版本 `r-a0056b48654e4dc5`，包含左右臂 ServoJ；配置不会热加载，已运行的旧节点需重启。
真机和仿真沿用相同消息格式；实际 Topic、频率和超时以运行配置为准。
实现：[运动节点](../src/humanoid_motion_control_node.cpp)、[SDK 后端](../src/motion/sdk_motion_backend.cpp)、[驱动接收](../../humanoid_driver_runtime/src/humanoid_driver_runtime_node.cpp)。
