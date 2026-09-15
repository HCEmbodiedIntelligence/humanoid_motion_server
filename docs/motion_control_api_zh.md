# 笛卡尔运动控制接口（OpenArmX 双臂）

适用：ROS 2 Humble，当前 OpenArmX 双臂配置。主要使用 **ServoP 连续控制末端位姿**、**MoveP 发送一次末端目标**。MoveJ 关节接口见末尾。

## 1. 接口速查

| 功能 | 左臂接口 | 右臂接口 | 通信方式 / ROS 2 类型 |
| --- | --- | --- | --- |
| 连续末端伺服 ServoP | `/teleop/left_arm/servo_p` | `/teleop/right_arm/servo_p` | Topic：`geometry_msgs/msg/PoseStamped` |
| 末端目标运动 MoveP | `/motion/left_arm/move_p` | `/motion/right_arm/move_p` | Action：`humanoid_motion_interfaces/action/MoveP` |
| 实际末端位姿反馈 | `/teleop/left_arm/fk_pose` | `/teleop/right_arm/fk_pose` | Topic：`geometry_msgs/msg/PoseStamped` |

控制程序向 ServoP 发布目标、向 MoveP 发送 Action Goal，订阅 FK Topic 读取实际末端位姿。运动学求解由服务端完成。

## 2. 笛卡尔数据怎么填

ServoP 直接发送 `PoseStamped`；MoveP 把同样的消息放到 `target_pose` 字段。

| 字段 | 类型 | 单位 / 含义 |
| --- | --- | --- |
| `header.frame_id` | `string` | 目标数值所在坐标系；下文统一使用 `openarmx_body_link0` |
| `header.stamp` | `builtin_interfaces/msg/Time` | ROS 时间，sec 为秒、nanosec 为纳秒；允许全零 |
| `pose.position.x/y/z` | 各 `float64` | **m**，工具末端在参考坐标系下的绝对位置；10 mm = 0.01 m |
| `pose.orientation.x/y/z/w` | 各 `float64` | **无量纲单位四元数**，顺序 x、y、z、w；不是欧拉角 |

| 控制对象 | MoveP 的 `group_name` | 工具末端 `tip_frame` |
| --- | --- | --- |
| 左臂 | `left_arm` | `left_tool0` |
| 右臂 | `right_arm` | `right_tool0` |

- 位置、姿态都表示**绝对目标**。保持当前姿态时，复制 FK 反馈中的四元数；`{x: 0, y: 0, z: 0, w: 1}` 表示单位旋转，不表示保持姿态。
- 沿机身坐标系 X 方向移动 1 cm：读取一次 FK 位姿，保留其姿态和 `frame_id`，将 `position.x` 加 `0.01`。持续发送这个目标，不要每帧累加。
- FK 反馈和 ServoP 默认使用 `openarmx_body_link0`。MoveP 也接受这个坐标系，并在内部换算到左臂 `openarmx_left_link0` / 右臂 `openarmx_right_link0`。复用位姿时保留原 `frame_id`，不要只改坐标系名字。
- `tip_frame` 指工具末端；ServoP 由 Topic 绑定左右工具，MoveP 需显式填写。外部相机坐标不能直接套用，应先换算到机器人已知坐标系。

## 3. ServoP：连续末端伺服

**100 Hz（每 10 ms 一帧）**持续发布最新目标；保持目标时也要刷新。QoS 使用 `best_effort`、`volatile`、`keep_last`，深度 5。

左臂示例（数值仅示报文格式，实际使用时替换为当前 FK 附近的目标）：

```bash
ros2 topic pub -r 100 \
  --qos-reliability best_effort --qos-durability volatile \
  --qos-history keep_last --qos-depth 5 \
  /teleop/left_arm/servo_p geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: openarmx_body_link0},
    pose: {position: {x: 0.30, y: 0.20, z: 0.40},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

右臂替换 Topic 和目标数值即可。ServoP 不提供每帧执行结果，通过对应 FK Topic 观察跟随情况。

停止发布约 **100 ms** 后，伺服会话失效；这不是机械急停时间保证。非零时间戳每帧更新，默认超过 0.5 s 的旧消息会被拒绝。上例省略时间戳，使用全零。

## 4. MoveP：发送一次末端目标

Action Goal 字段如下；一次发一个目标，等待执行结果后再发下一个。

| 字段 | 类型 | 填写方式 |
| --- | --- | --- |
| `group_name` | `string` | `left_arm` 或 `right_arm`，必须与 Action 端点匹配 |
| `tip_frame` | `string` | `left_tool0` 或 `right_tool0` |
| `target_pose` | `geometry_msgs/msg/PoseStamped` | 第 2 节的完整位姿，位置 m、姿态四元数 |
| `options` | `humanoid_motion_interfaces/msg/MotionOptions` | 下述速度比例和超时参数 |

```bash
ros2 action send_goal /motion/left_arm/move_p \
  humanoid_motion_interfaces/action/MoveP \
  '{group_name: left_arm, tip_frame: left_tool0,
    target_pose: {header: {frame_id: openarmx_body_link0},
      pose: {position: {x: 0.30, y: 0.20, z: 0.40},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    options: {velocity_scale: 0.2, acceleration_scale: 0.2,
              jerk_scale: 0.2, timeout_sec: 30.0}}' --feedback
```

右臂同时替换 Action、`group_name`、`tip_frame` 和目标数值；输入坐标系仍可使用 `openarmx_body_link0`。

- `velocity_scale`、`acceleration_scale`、`jerk_scale`：均为 `float64`、无量纲。非零取值 `(0, 1]`，0.2 表示配置上限的 20%；**0 表示使用默认上限，不表示停止**。三个比例独立设置。
- `timeout_sec`：`float64`，单位 **s**；0 使用默认 60 s。当前内部监测也受服务端默认超时限制，仅调大 Goal 超时不能延长该限制。
- 成功条件：Action 终态 `SUCCEEDED`，且 Result 的 `status.code == 0`；失败原因看 `status.message`。`progress` 当前固定为 0，不能用于判断完成。
- 位姿反馈 `actual_pose` 和结果 `final_pose` 均为 `PoseStamped`，位置 m、姿态四元数；其坐标系为对应单臂基座，读取时检查 `header.frame_id`。失败时位姿字段可能未填充。
- **MoveP 接收笛卡尔目标，内部按关节轨迹运动，末端路径不保证直线。** 需要直线路径时使用 `/motion/left_arm/move_l` 或 `/motion/right_arm/move_l`，类型为 `humanoid_motion_interfaces/action/MoveL`，Goal 字段与 MoveP 相同。

## 5. 联调与控制切换

```bash
source /opt/ros/humble/setup.bash
source /home/hc_op/workspace/teleop_ws/install/setup.bash
export ROS_DOMAIN_ID=14
ros2 topic info /teleop/left_arm/servo_p -v
ros2 action list -t
ros2 topic echo /teleop/left_arm/fk_pose --once --qos-reliability best_effort
```

工作区路径和 Domain 按实际机器修改。机器人需已在管理页开启；ServoP 应有运动服务订阅者，MoveP 应有 Action Server。

同臂持续 Servo 更新会抢占 Move 动作，切换前停止 Servo 并等 100 ms lease 到期。取消 Action 时用 GoalHandle 的 `cancel_goal_async()` 并等待最终结果；退出客户端不代表动作已取消。实际关节反馈 `/hc_teleop/joint_states` 应持续更新，默认超过 100 ms 未更新会影响执行。

## 附：MoveJ 关节接口

左右臂 Action 分别为 `/motion/left_arm/move_j`、`/motion/right_arm/move_j`，类型均为 `humanoid_motion_interfaces/action/MoveJ`。

Goal 使用 `group_name`（string）、`target`（`sensor_msgs/msg/JointState`）、`options`（同 MoveP）。`target.name` 为 `string[]`，`target.position` 为对应的 `float64[]`，单位 **rad**；速度和力矩数组留空。关节名为 `openarmx_left_joint1`～`7`、`openarmx_right_joint1`～`7`。可以只给部分关节，其余用当前实测角度补齐。

```bash
ros2 action send_goal /motion/left_arm/move_j \
  humanoid_motion_interfaces/action/MoveJ \
  '{group_name: left_arm,
    target: {name: [openarmx_left_joint4], position: [0.30]},
    options: {velocity_scale: 0.2, acceleration_scale: 0.2,
              jerk_scale: 0.2, timeout_sec: 30.0}}' --feedback
```

当前 OpenArmX 已配置的连续伺服接口是 ServoP，尚未配置 ServoJ Topic。

---

依据：[OpenArmX 通道配置](../../openarmx_driver/deployment/openarmx_v10_bimanual/model/channels.yaml)、[MoveP 定义](../../humanoid_motion_interfaces/action/MoveP.action)、[运动服务实现](../src/humanoid_motion_control_node.cpp)。已核对本机部署配置和示例消息格式，未执行真机运动验证。
