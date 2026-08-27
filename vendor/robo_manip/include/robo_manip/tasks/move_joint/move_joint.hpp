#pragma once

#include <string>
#include <vector>

#include "robo_manip/tasks/common/basic_task.hpp"
#include "rtc.hpp"

namespace robo_manip::tasks {

class MoveJoint : public BasicTask {
 public:
  /// 关节空间点到点运动请求。
  ///
  /// 关节位置使用 motion_control 约定的单位：deg。当前 RobotState 会作为
  /// 轨迹起点；waypoints_deg 是后续需要经过的一个或多个关节点位。
  struct JointRequest {
    std::string request_id;
    std::string group_name;
    std::vector<std::vector<double>> waypoints_deg;
    motion_control::types::MotionLimits limits{};
    motion_control::types::PlanningParameters parameters{};
  };

  /// 实时接口输入的关节目标点，单位 deg。
  using JointTarget = std::vector<double>;

  /// 实时接口输出的关节命令。
  ///
  /// positions/velocities/efforts 与 joint_names 按下标对应；关节角单位为 deg。
  struct JointPos {
    std::string group_name;
    std::vector<std::string> joint_names;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
  };

  /// 实时控制可选参数。
  struct OptParams {
    /// 控制周期，单位 s。
    double dt_sec{0.02};
  };

  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;

  motion_control::types::PlanningResult planJoint(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const JointRequest& request);

  motion_control::types::PlanningResult planJoint(
      const core::MotionContext& context,
      const JointRequest& request);

  /// 启动实时关节点到点运动。
  ///
  /// 该函数会用当前 RobotState 重置 RTC 内部关节状态，并保存
  /// context/request 供 TickRealtimeMoveJoint() 连续递推使用。
  bool StartRealtimeMoveJoint(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const JointRequest& request,
      const OptParams& opt);

  /// 推进一个实时控制周期。
  ///
  /// target 是当前周期的关节目标点。函数内部通过 RTC 生成平滑关节命令
  /// command_joint。reached=true 表示 RTC 已到达目标。
  bool TickRealtimeMoveJoint(const JointTarget& target,
                             JointPos& command_joint,
                             bool& reached,
                             const OptParams& opt);

  /// 停止实时关节点运动并清空内部缓存状态。
  void StopRealtimeMoveJoint();

  static motion_control::types::PlanningRequest toPlanningRequest(
      const core::MotionContext& context,
      const JointRequest& request);

 private:
  /// 实时接口内部缓存。Start 设置，Tick 更新，Stop 清空。
  core::MotionContext realtime_context_{};
  JointRequest realtime_request_{};
  std::vector<std::string> realtime_joint_names_;
  bool realtime_active_{false};
};

}  // namespace robo_manip::tasks
