# Meshcat 离线仿真调试

仿真属于 `humanoid_motion_server` 的可选能力。它复用原有运动服务、SDK、URDF、
Move/Servo 通道和遥操作前端，用运动学执行器替代硬件，在浏览器中显示模拟关节反馈。
正常真机启动不导入 Meshcat，也不需要安装它。

```text
PICO / ROS Move、Servo 请求
        ↓
humanoid_motion_control_node（原有 IK、RTC、仲裁、断流检查）
        ↓ /hc_teleop/joint_cmd
simulation_node（最新目标、速度/位置限制、断流保持，100 Hz）
        ↓ /hc_teleop/joint_states
运动服务计算 FK → 遥操作前端绑定与跟踪
        ↓ /simulation/joint_states（含夹爪和 mimic 关节）
meshcat_viewer（最多 30 Hz）→ 浏览器
```

这是**固定基座的运动学仿真**。可验证接口、IK、轨迹、遥操作映射、夹爪开合及断流行为；
不模拟重力、惯量、碰撞接触、摩擦、CAN、真实电机响应或移动底盘。
仿真通过不等于真机时延、碰撞安全或动力学通过。

## 安装与编译

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./src/humanoid_motion_server/scripts/setup_simulation.sh
colcon build --packages-select humanoid_motion_server
source install/setup.bash
source src/humanoid_motion_server/.simulation-venv/bin/activate
```

脚本只在包目录的 `.simulation-venv` 安装 Meshcat 0.3.2 及 Python 依赖，复用系统 NumPy
和 ROS Pinocchio，不安装 pip 的 `pin` 包、不升级系统 SDK。要求系统有 `python3-pip`、
Python `venv` 模块，以及可导入的 ROS Pinocchio（Ubuntu/Humble 包名 `ros-humble-pinocchio`）。
也可以给脚本传一个自定义虚拟环境目录。

若不激活虚拟环境，在 launch 命令后指定
`simulation_python:=/绝对路径/.simulation-venv/bin/python`。

## 当前 OpenArmX

```bash
ros2 launch humanoid_motion_server openarmx_sim.launch.py
```

浏览器打开 **http://127.0.0.1:7000/static/**。使用鼠标旋转、缩放观察双臂和夹爪。
默认加载部署的 `openarmx_01` 的运动/通道/工具配置和 `teleop_home_1` 初始姿态，
仿真 profile 将两侧肘关节 `joint4` 覆盖为 0.99 rad。这样避免当前真实回零配置中
左肘为 0、右肘为 0.99 rad 导致仿真左臂从近奇异、肘部下限位置起步。
初始化是在仿真中直接设置位置，不会执行真机回零。

当前部署 URDF 没有 visual，示例配置从 `openarmx_description` 的完整 URDF 补充网格。
启动时校验显示模型与控制模型的 link、joint、origin、axis 相同，只复制 visual/material，
保留控制模型的限位和运动学。缺失模型资源或结构不匹配会明确报错。

连接 PICO：

```bash
ros2 launch humanoid_motion_server openarmx_sim.launch.py start_teleop:=true
```

使用原有 PICO 配置连接这台电脑；默认 UDP 端口仍为 5005/5006。启动后控制默认关闭，
按 A 恢复控制，再使用原有 clutch/deadman 操作；夹爪继续按已有配置操作。
当前 OpenArmX 配置中，两臂均由**右手握持键**使能，左臂跟随左手姿态。
模型有真实关节限位；某臂贴近腕部/肩部限位后，继续向该方向平移或旋转可能使 SDK
无法满足目标并保持当前位置。松开右握持键、调整手柄到舒适姿态后重新握住，
再向限位内的小幅方向移动。
这个初始姿态改动不会扩大限位，也不保证任意手柄目标都可达。
若已有真机接收端占用 UDP 端口，先关闭它，或同时修改 PICO 发送设置并指定
`pose_port:=15005 discovery_port:=15006`。ROS 域隔离不会隔离 UDP 端口。

**回位手势：左摇杆向左、右摇杆向右，同时推到底（向外拨）。** 先让两根摇杆回中，
再拨动；持续推住只触发一次。回位由本包的 `simulation_home` 节点执行，先暂停遥操、
等待 Servo 控制权释放，再通过 MoveJ 平滑返回本次仿真的启动关节姿态，包含 profile 的
`initial_pose` / `initial_positions` 覆盖，不重置夹爪。完成后清除旧的手柄参考；
**按 A 恢复遥操，再握住右握持键**。A 键本身不回位，回位执行中也不能抢占运动。
已有配置中的暂停键和紧急停止话题可以取消回位；取消或失败后保持遥操关闭。
若未确认所有 MoveJ 已停止，保持控制权锁定并报告错误，需要重启仿真。

此独立入口不启动配置管理器、相机、录制、底盘、厂商驱动或 CAN；临时遥操作配置会关闭
依赖管理器的姿态任务、录制和打标动作，不改写已部署配置。通用机器人需要能覆盖全部
运动关节、相互不重叠的 MoveJ 通道；启动时检查。回位结果可在
`/hc_teleop_recv/status` 的 `last_action` 查看，也会转发给 PICO。

## 没有 PICO 也能测试

保持仿真运行，在第二个终端 source 同一工作区与虚拟环境：

```bash
ros2 run humanoid_motion_server verify_simulation.py \
  --profile "$(ros2 pkg prefix humanoid_motion_server)/share/humanoid_motion_server/config/simulation/openarmx.yaml" \
  --expect-viewer
```

程序先确认 `/simulation/status` 的模式和机器人 ID，然后发送小幅 MoveJ、连续 ServoP
以及夹爪目标，检查反馈变化、命令周期、停止输入后的保持和显示状态。会改变当前仿真姿态。
请在遥操作空闲时运行；测试不会自动启动仿真。

其他 ROS 调试工具也要使用仿真的域：

```bash
export ROS_DOMAIN_ID=199
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /simulation/status
ros2 topic echo /simulation/viewer_status
ros2 action list
```

默认域 199 与当前真机域 14 分离。`domain_id:=其他值` 可切换，第二个终端及验证程序的
`--domain-id` 必须一致。不要把仿真设为正在运行的真机域。
`use_sim_time` 固定关闭；仿真按实际经过的时间运行，反馈使用节点时钟，不发布 `/clock`。

## 通用入口与其他机器人

无机器人部署也可运行内置 6 关节模型（无网格时显示运动学骨架）：

```bash
ros2 launch humanoid_motion_server simulation.launch.py
ros2 run humanoid_motion_server verify_simulation.py --expect-viewer
```

已部署机器人可使用 `simulation.launch.py robot_id:=my_robot`。
只有该模式才导入 `humanoid_manager`；启用遥操作时才启动 `hc_teleop_recv`。
核心算法没有 OpenArmX 关节名，支持 URDF 的 revolute、continuous、prismatic、fixed
及链式 mimic。浮动基座、planar joint 和动力学模型不在本版范围内。

未部署的机器人可以提供完全独立的 YAML，无需硬件驱动插件或管理器：

```yaml
resources:
  motion_params: ./motion.yaml
  sdk_config: ./sdk.yaml
  channel_config: ./channels.yaml
  tool_config: ./tools.yaml
  urdf: ./robot.urdf
  # hc_teleop_config: ./hc_teleop.yaml   # 可选
# visual_urdf: ./robot_with_visuals.urdf # 原 URDF 已有网格则不需要
initial_positions:
  shoulder_joint: 0.3
speed_limit: 3.0                       # 旋转关节 rad/s，移动关节 m/s
watchdog_s: 0.1
# 逻辑夹爪名与 URDF 关节不同才需要显示映射；scale/offset 用于单位/行程转换。
# gripper_joints:
#   left_gripper:
#     joint: left_finger_joint
#     scale: 1.0
#     offset: 0.0
```

```bash
ros2 launch humanoid_motion_server simulation.launch.py profile:=/绝对路径/simulation.yaml
```

路径相对于 profile 文件，支持绝对路径和 `package://`。SDK 文件内部的路径仍按 SDK 自身
配置规则解释。初值优先级为：限位内最接近零的位置 → SDK initial_state → 指定 initial_pose
→ initial_positions。初值越界会拒绝启动；不偷偷修改用户指定的初值。
模拟执行器采用 URDF 与运动组配置限位的交集，并同时限制 URDF 速度及 speed_limit。

夹爪从遥操作配置读取 JointState 接口、开合行程及速度；当前仅支持 JointState 类型。
显示映射不改变逻辑夹爪反馈，另一侧手指由 URDF mimic 自动更新。

## 运行选项和故障行为

| 参数 | 默认值 | 用途 |
|---|---|---|
| `meshcat` | `true` | `false` 无显示运行，不需要 Meshcat/Pinocchio Python 显示依赖 |
| `meshcat_host` | `127.0.0.1` | `0.0.0.0` 可供局域网浏览器访问 |
| `meshcat_port` | `7000` | HTTP 端口，已占用则启动失败 |
| `viewer_rate` | `30` | 显示上限 1–60 Hz，不改变控制频率 |
| `start_teleop` | `false` | 是否启用机器人遥操作前端 |
| `domain_id` | `199` | 独立 ROS 域，子进程使用本机 DDS |

手臂命令必须带非零、递增的新鲜时间戳，按关节覆盖目标，左右臂的部分命令不会互相覆盖。
传输年龄计入 watchdog，停止输入后保持实测模拟位置；夹爪 watchdog 为 250 ms。
进程卡住时不追赶历史目标，也不一次积分跳过很大距离。

显示订阅只保留最新反馈，以固定频率刷新，不阻塞运动服务。反馈超过 250 ms 未更新时，
隐藏机器人并报告 stale，避免把冻结画面当作有效反馈。关闭浏览器不会暂停仿真；
**Ctrl+C 关闭 launch** 才会退出整个仿真。任一核心仿真进程异常退出也会关闭本组进程。
Meshcat 服务是 launch 管理的独立进程，退出后释放端口，临时配置自动清理。

单元回归包含最新目标覆盖、左右臂独立超时、时间戳重放/过期/回退、卡顿保持、限位、
mimic 和显示模型校验；已加入本包 CTest 的 `simulation_unit`。
