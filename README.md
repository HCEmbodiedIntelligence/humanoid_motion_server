# humanoid_motion_server (Pinocchio Backend)

本仓库提供通用的 ROS 2 人形/双臂机器人运动规划与伺服控制服务，底层由 Pinocchio 动力学/运动学引擎与带框约束差分逆解（CLIK）驱动，替代原先依赖闭源第三方库的方案。

## 架构

- **输入通道（从 `channels.yaml` 动态生成）**：
  - `ServoP`: 订阅 `geometry_msgs/msg/PoseStamped`，实现高频笛卡尔透传控制。
  - `ServoJ`: 订阅 `sensor_msgs/msg/JointState`，实现高频关节透传控制。
  - `MoveJ`: ROS 2 Action (`humanoid_motion_interfaces/action/MoveJ`)，执行五次多项式平滑关节轨迹。
  - `MoveP` / `MoveL`: ROS 2 Action，执行笛卡尔末端轨迹规划与逆解跟踪。
  - `FK 广播`: 为配置了 `fk_pose_topic` 的通道持续计算并发布实际末端位姿。
- **与硬件解耦**：
  - 读取真实状态：订阅 `joint_state_endpoint`（如 `/observation/joint_states` 或 `/internal/robot/joint_states`）。
  - 发布控制指令：发布到 `joint_command_endpoint`（如 `/control/motion/joint_targets`）。
- **仲裁与安全保护**：
  - 租约超时保护 (`servo_lease_ms`)：断连或停止发送后自动保持在当前位置，防止失控。
  - 控制源权限管理：根据 `output_permit_topic`（`/control/motion/permit`）切换 VR 遥操作通道与自主控制通道。
  - 关节软限位与速度裁剪。
