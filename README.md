# humanoid_motion_server

## Meshcat 离线仿真

本包提供独立的运动学仿真闭环，可在不启动硬件的情况下调试 Move/Servo、遥操作与夹爪。
完整安装、启动和其他机器人接入方式见 [Meshcat 离线仿真调试](docs/simulation_zh.md)。

```bash
# 先按文档安装可选显示依赖、编译并激活仿真虚拟环境
ros2 launch humanoid_motion_server openarmx_sim.launch.py
# PICO 遥操作：在命令后加 start_teleop:=true
```

浏览器打开 `http://127.0.0.1:7000/static/`。仿真默认使用独立 ROS 域 199。
通用入口为 `simulation.launch.py`，无 profile 时使用内置模型；
这是一套运动学/控制流程仿真，不包含物理碰撞和动力学。

## 上层运动接口文档

给应用开发和同事联调使用的中文接口说明见
[笛卡尔运动控制接口（OpenArmX 双臂）](docs/motion_control_api_zh.md)，
重点介绍 ServoP、MoveP 的接口名称、类型、单位、坐标系和调用示例，附 MoveJ 关节接口。

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

ROS 目录可能同时存在不兼容的同名库（例如 `libruckig.so`）。SDK 检查脚本和运动节点
均优先使用 `/usr/local/lib`、`/usr/local/lib/x86_64-linux-gnu` 中的固定版本。
节点通过自身 RPATH 保持此顺序，不修改 ROS 安装目录或全局 `LD_LIBRARY_PATH`。
更新该策略后必须重新编译安装节点，单独更新检查脚本不会改变旧二进制的加载行为。
CMake 配置也统一使用安装脚本写入 `/usr/local` 的固定依赖，包括 Pinocchio、hpp-fcl 和
OctoMap，并纠正这些包残留的 ROS `*_DIR` 缓存。仅固定版本号不够：ROS 可以提供版本号
相同但构建选项不同的库。若项目已提前导入其他来源的 SDK 依赖目标，配置会明确拒绝混用。

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
`--skip-keys "humanoid_driver_runtime hc_teleop_recv"`。出现
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

若实测关节已经略超出模型限位，合法的 IK 目标经过最终关节 RTC 平滑后，
最初几个输出仍可能在限位外。此时仅允许向限位内返回的过渡：实测值和上一条输出
必须位于同一越界侧，新的输出不得比任一值更向外，速度也不得向外。
IK 候选目标仍必须在模型限位内，最终 RTC 不可绕过；反馈或输出回到限位内后，
不再允许重新越界。不会改写反馈、放宽模型限位或把首条输出直接截到边界。

若恢复中的实测关节比最终 RTC 输出更早向内返回（包括编码器在边界两侧跳一个刻度），
拒绝的旧输出不会发送：只把该关节的 RTC 起点同步到真实反馈，再经过 RTC 生成一次输出并重新校验。
其余关节保留上一条已输出的位置、速度和加速度，Servo 会话保持连续。
向外退步、向外速度、新产生的越界、异常的大幅反馈跳变，以及重算后仍不合法的结果仍会停止。

运动节点参数 `feedback_limit_recovery_margin_rad` 默认 0.1 rad，表示允许尝试恢复的最大实测越界量；
更大的越界反馈按故障处理，不能送入 IK，也不能用于生成保持指令。
该组的末端 FK 发布也暂停，避免前端用异常关节值绑定遥操作起点；其他有效关节组可继续发布。
`feedback_rebase_tolerance_rad` 默认 0.001 rad，表示反馈分辨率余量；重新同步的最大步长为
该余量加上本周期的关节速度上限乘以周期时长。两者都不扩大目标或输出的模型限位。
原始关节反馈始终保留。上述检查不能替代硬件层对真实 CAN 回包和时间戳的有效性检查。

`motion.servo:*` 日志记录实际启动、停止原因和恢复，仅在状态变化时输出，
每个通道最多每秒一条。同一周期内启动后立即失败会合并为一次失败，
持续重复的求解失败不会每帧打印。
边界恢复开始/完成各记一次状态变化；拒绝越界时附带 candidate/final_rtc 阶段、目标值、
限位及可用的实测值和上一条输出，便于区分 IK 目标失败和平滑器恢复失败。

控制定时器使用独立的互斥 callback group；输入回调不等待 IK/RTC 的管线锁。
Servo 订阅的 DDS 队列深度为 1，每个配置通道在线程之间也只保留一个可覆盖的最新目标，
新目标覆盖旧目标，不排队执行历史目标。控制循环每轮取走最新值，正常更新保持 SDK 会话连续。
带时间戳的目标必须递增，重复、乱序和超过 `min(input_stamp_max_age_s, servo_lease_ms / 1000)`
的目标会被丢弃；传输耗时计入 lease。消费时再次检查是否过期，再进行优先级仲裁，
因此过期目标不能续期，也不能抢占正在执行的 Move。零时间戳 Servo 为兼容接口继续支持，
此时只能按本节点的接收顺序和接收时间判定新鲜度；遥操作发送端应始终填写时间戳。

真实关节反馈订阅同样使用 Best Effort 并只保留最新一条，不等待旧帧重传；兼容驱动的 Reliable 发布端。
反馈必须有非零、递增且未过期的源时间戳，
其传输耗时计入 `feedback_max_age_ms`；发布的 FK 保留这条真实反馈的源时间戳。
重复或陈旧的反馈不能通过重新打时间戳延长遥操作。ROS 时钟向后重置时会重新建立时间戳顺序。

每条管线输出带有内部有效期，受输入 lease、实测反馈年龄及控制计算时限约束。
输出还带有按关节授予的一次性发布权限。取消、抢占或新的控制请求会撤销旧权限，
最后一次校验与发布在同一管线锁内完成，避免取消后仍发出已经计算好的旧指令。
Move 工作线程由节点持有并在退出时回收；终态事件只保留给仍在等待结果的 Move 请求。
SDK 调用完成、整批双臂计算完成及实际发布前都会检查有效期；过期结果丢弃并停止对应会话，
下一次有效目标从新鲜实测反馈重新启动。不会尝试中断闭源 SDK 的正在执行调用。
`control_max_period_ms` 默认 100 ms（不小于名义控制周期）：调度间隔超过它时，Servo
从实测状态重启并使用名义周期，Move 返回超时，避免将长时间卡住直接变成一次大步长 RTC 更新。

`servo_retry_interval_ms` 默认 50 ms，只用于 SDK 求解/输出失败后的重试间隔。
失败时停止原会话；间隔内仍接收并覆盖最新目标，重试只取最新目标和新鲜反馈，
避免持续失败时每个输入帧都重建求解器。正常跟踪不引入这个等待。限位、最终 RTC 和
100 ms 默认断流保护保持有效，异常反馈不能用于生成新的保持命令。

`test_servo_freshness_ros.py` 已纳入 CTest：仅使用仓库模型、假反馈、独立本机 ROS 域及随机话题，
检查深历史 Reliable 反馈发布下的双臂连续输出、Best Effort 订阅（DDS 提供深度时同时检查深度为 1）、
重复目标不能续期、旧/冻结反馈不能续期及单臂断流隔离。

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
`robot_composition` 清单，由 `humanoid_manager` 校验和部署。通用启动命令只接受部署后的
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
