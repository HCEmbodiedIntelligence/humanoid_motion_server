#pragma once

#include <optional>
#include <string>

#include "robo_manip/tasks/common/basic_task.hpp"
#include "types/ik.hpp"
#include "types/trajectory.hpp"

namespace robo_manip::tasks {

class MovePath : public BasicTask {
 public:
  /// 通用路径规划请求。
  ///
  /// joint_path 和 cartesian_path 应且只应设置一个。关节路径会直接交给
  /// RMP/TOPPRA 做时间参数化；笛卡尔路径会先通过 RKD IK 离散成关节路径，
  /// 再输出带时间的关节轨迹。
  ///
  /// cartesian_path 中 Pose 的位置单位为 mm；姿态由用户填写
  /// Pose::euler_zyx_deg，单位 deg，顺序为 roll/pitch/yaw，旋转约定为
  /// R = Rz(yaw) * Ry(pitch) * Rx(roll)。Pose::orientation 仅作为内部四元数缓存。
  struct PathRequest {
    std::string request_id;
    std::string group_name;
    motion_control::types::MotionLimits limits{};
    std::optional<motion_control::types::JointTrajectory> joint_path;
    std::optional<motion_control::types::CartesianTrajectory> cartesian_path;
    /// 笛卡尔路径离散成关节路径时使用的 IK 后端和求解参数。
    motion_control::IkOptions ik_options{};
    motion_control::types::PlanningParameters parameters{};
  };

  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;

  motion_control::types::PlanningResult planPath(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const PathRequest& request);

  motion_control::types::PlanningResult planPath(
      const core::MotionContext& context,
      const PathRequest& request);

  static motion_control::types::PlanningRequest toPlanningRequest(
      const PathRequest& request);
};

}  // namespace robo_manip::tasks
