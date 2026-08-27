#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robo_manip/tasks/common/basic_task.hpp"
#include "robo_manip/tasks/move_joint/move_joint.hpp"
#include "types/geometry.hpp"
#include "types/ik.hpp"

namespace robo_manip::tasks {

class Jog : public BasicTask {
 public:
  /// 点动空间。当前支持关节空间和笛卡尔空间连续点动。
  enum class JogSpace {
    kJoint,
    kCartesian,
  };

  /// 点动模式。当前实现 kContinuous。
  enum class JogMode {
    kIncremental,
    kContinuous,
  };

  struct JointJogCommand {
    /// 需要点动的关节名，可只填参与点动的子集。
    std::vector<std::string> joint_names;
    /// 点动数值，与 joint_names 一一对应。
    /// 增量模式表示每个关节本次要移动的角度增量，单位 deg；
    /// 连续模式表示每个关节的目标点动速度，单位 deg/s。
    std::vector<double> values;
  };

  struct CartesianJogCommand {
    std::string link_name;
    std::string base_link{"base_link"};
    /// 笛卡尔点动指令。连续模式下表示 TCP 的目标 twist：
    /// linear 为平移速度，单位 mm/s；angular 为旋转角速度，单位 rad/s。
    /// twist 表达在 base_link 坐标系中。
    motion_control::types::Twist value{};
  };

  struct JogRequest {
    std::string request_id;
    std::string group_name;
    JogSpace space{JogSpace::kJoint};
    JogMode mode{JogMode::kIncremental};
    std::optional<JointJogCommand> joint;
    std::optional<CartesianJogCommand> cartesian;
    /// 可选启动覆盖；留空时 StartJog() 从配置文件的 rmp/rtc 约束读取。
    motion_control::types::MotionLimits limits{};
    motion_control::IkOptions ik_options{};
    /// 可选启动覆盖；<=0 时使用 planning.sample_period_sec 或 rtc.output_rate_hz。
    double dt_sec{0.0};
    /// 连续点动保活超时预留字段；第一版由上层 StopJog() 显式停止。
    double command_timeout_sec{0.1};
  };

  struct JogState {
    bool active{false};
    /// 连续点动中表示“停止完成”：速度命令为 0 且输出速度已降到 0 附近。
    bool reached{false};
    motion_control::types::ResultCode code{
        motion_control::types::ResultCode::kInternalError};
    std::string message;
    MoveJoint::JointPos command_joint{};
    motion_control::types::Pose command_pose{};
  };

  /// Jog 是实时任务入口，不走离线 plan；该接口固定返回 NotImplemented。
  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;

  /// 启动点动，使用当前 RobotState 初始化 RTC 内部状态。
  bool StartJog(const core::MotionContext& context,
                const motion_control::types::RobotState& state,
                const JogRequest& request);

  /// 推进一个点动控制周期，输出本周期应下发的关节流式命令。
  bool TickJog(const motion_control::types::RobotState& state,
               const JogRequest& command,
               JogState& output);

  /// 停止点动；halt_execution=true 时会同时调用底层 driver stop。
  void StopJog(bool halt_execution = true);

 private:
  core::MotionContext context_{};
  JogRequest request_{};
  std::vector<std::string> joint_names_{};
  motion_control::types::RobotState cartesian_reference_state_{};
  motion_control::types::Pose cartesian_command_pose_{};
  motion_control::types::Twist cartesian_command_twist_{};
  std::vector<double> previous_command_positions_{};
  bool previous_command_valid_{false};
  bool active_{false};
};

}  // namespace robo_manip::tasks
