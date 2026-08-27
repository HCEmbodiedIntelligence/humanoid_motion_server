#pragma once

#include <string>
#include <vector>

#include "robo_manip/tasks/common/basic_task.hpp"
#include "rkd.hpp"
#include "rtc.hpp"

namespace robo_manip::tasks {

class MoveLine : public BasicTask {
 public:
  /// 笛卡尔直线/折线路径请求。
  ///
  /// Pose 位置单位沿用 motion_control 约定：mm；姿态由用户填写
  /// Pose::euler_zyx_deg，单位 deg，顺序为 roll/pitch/yaw，旋转约定为
  /// R = Rz(yaw) * Ry(pitch) * Rx(roll)。Pose::orientation 仅作为内部四元数缓存。
  /// 起点通常由当前 RobotState 通过 RKD 正解得到，goal_pose 为目标点，
  /// via_poses 为可选途径点。
  struct LineRequest {
    std::string request_id;
    /// 参与 IK/关节输出的关节组，例如 left_arm、right_arm、whole_body。
    std::string group_name;
    /// IK/RTC 控制链起点，也是笛卡尔路径的表达坐标系。
    /// 默认从整机 base_link 开始；左臂局部 MOVL 可设为 L_arm_0_Link。
    std::string base_link{"base_link"};
    std::string link_name;
    motion_control::types::Pose start_pose{};
    std::vector<motion_control::types::Pose> via_poses;
    motion_control::types::Pose goal_pose{};
    motion_control::types::MotionLimits limits{};
    motion_control::types::PlanningParameters parameters{};
    motion_control::IkOptions ik_options{};
  };

  /// 实时接口输入的笛卡尔目标位姿：位置单位 mm，姿态使用
  /// Pose::euler_zyx_deg（ZYX 欧拉角，deg）。
  using CartesianPos = motion_control::types::Pose;

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
    /// IK 后端和求解参数；未指定 placo.joint_group 时会使用 LineRequest::group_name。
    motion_control::IkOptions ik_options{};
  };

  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;

  motion_control::types::PlanningResult planLine(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const LineRequest& request);

  motion_control::types::PlanningResult planLine(
      const core::MotionContext& context,
      const LineRequest& request);

  /// 启动实时直线运动。
  ///
  /// 该函数会用当前 RobotState 计算末端起点，重置 RTC 内部状态，并保存
  /// context/state/request 供 TickRealtimeMoveLine() 连续递推使用。
  /// 实时模式需要 context.rtc 和 context.rkd 均有效。
  bool StartRealtimeMoveLine(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const LineRequest& request,
      const OptParams& opt);

  /// 推进一个实时控制周期。
  ///
  /// target 是当前周期的笛卡尔目标。函数内部先通过 RTC 生成平滑笛卡尔命令，
  /// 再通过 RKD IK 转成关节命令 command_joint。reached=true 表示 RTC 已到达目标。
  bool TickRealtimeMoveLine(const CartesianPos& target,
                            JointPos& command_joint,
                            bool& reached,
                            const OptParams& opt);

  /// 停止实时直线运动并清空内部缓存状态。
  void StopRealtimeMoveLine();

  /// 将 LineRequest 转成通用 PlanningRequest，供调度层或 BasicTask 入口使用。
  static motion_control::types::PlanningRequest toPlanningRequest(
      const LineRequest& request);

 private:
  /// 实时接口内部缓存。Start 设置，Tick 更新，Stop 清空。
  core::MotionContext realtime_context_{};
  motion_control::types::RobotState realtime_state_{};
  LineRequest realtime_request_{};
  bool realtime_active_{false};
};

}  // namespace robo_manip::tasks
