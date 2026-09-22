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

也可以通过管理器页面启动：打开“机器人配置”，选择已保存机器人，把运行模式改为
“仿真”，按需选择初始姿态和“启动遥操作”，点击“开启仿真”。状态就绪后点击
“打开仿真画面”。停止、重启都在同一区域，网页自身保持运行。
网页只提交启动和动作请求；仿真反馈、运动计算及姿态执行仍由独立节点负责。
显示的配置版本来自仿真节点的实际状态，进程启动不代表反馈已经就绪。
默认 ROS 域 199、画面端口 7000、遥操作端口 15005/15006，可在页面修改。
页面启动时，仿真与网页自动使用相同的 `ROS_LOCALHOST_ONLY`，否则同域也可能无法发现节点。
独立命令行入口默认 `localhost_only:=true`；需要外部观察节点时应显式匹配其通信范围。
使用命令行的方式保持不变：

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
再拨动；持续推住只触发一次。仿真与真机都运行 `humanoid_pose_runtime`，由它直接接收
手柄动作，使用公共姿态解析器和执行器暂停遥操、等待 Servo 租约释放、执行 MoveJ、
处理取消并发布真实执行结果。网页管理器仅保存配置、显示状态和转发用户请求；
不运行回位状态机，头戴设备回位不依赖网页进程。

默认遵循遥操作配置的 `actions.home_pose_id`，从同一份 `initial_poses.yaml` 解析目标、
速度、加速度、jerk 和超时。仿真 profile 可用 `home_pose` 显式选择另一个保存姿态；
只有明确配置 `home_pose: simulation_initial` 时，才把本次启动位置转换成公共姿态格式，
仍交给同一个执行器。该模式继承 `initial_pose` 的运动参数；没有来源姿态时采用公共
姿态默认值 0.15 / 60 秒。不会强制启用配置中关闭的回位手势。

OpenArmX 示例明确选择 `home_pose: simulation_initial`，所以仍回到两侧肘关节覆盖为
0.99 rad 的仿真启动姿态，不重置夹爪。删除这一选择即可像真机一样回到已保存的
`home_pose_id`，启动位置覆盖不会偷偷改写保存的回位目标。

成功后清除旧参考；如果回位前遥操已使能，恢复使能并用新鲜 FK 重新绑定；原先未使能
则保持关闭。这与真机使用同一策略。A 键不回位，执行中也不能抢占运动。
暂停和紧急停止可取消回位；取消或失败后保持遥操关闭。若未确认全部 MoveJ 停止，
保持控制权锁定并报告错误。

此入口不启动网页、相机、录制、底盘、厂商驱动或 CAN。依赖管理器的录制、打标和姿态
偏好任务仍关闭，已部署配置不变。保存姿态只控制其中指定的关节组；生成启动快照姿态
时才要求相互不重叠的 MoveJ 通道覆盖全部模拟运动关节。
`/motion/pose_status` 发布运行时状态，`/motion/pose_results` 发布执行结果；
手柄结果同时进入 `/hc_teleop_recv/status` 的 `last_action` 并转发给 PICO。

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
核心算法没有 OpenArmX 关节名。显示和模拟执行器解析 revolute、continuous、prismatic、fixed
及链式 mimic；当前运动后端控制组只支持独立的 revolute / continuous 关节。
浮动基座、planar joint 和动力学模型不在本版范围内。

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
# initial_poses_file: ./initial_poses.yaml # 与真机相同的命名姿态文件
# initial_pose: ready                    # 仿真启动时设置的姿态
# home_pose: ready                       # 默认遵循遥操作 home_pose_id
# home_pose: simulation_initial          # 显式选择回到仿真启动状态
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
