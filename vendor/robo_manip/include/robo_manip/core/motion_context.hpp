#pragma once

#include <memory>
#include <string>
#include <vector>

#include "force_control.hpp"
#include "mpc.hpp"
#include "rkd.hpp"
#include "rmp.hpp"
#include "robo_manip/execution/execution.hpp"
#include "rtc.hpp"
#include "types/ik.hpp"
#include "wbc.hpp"

namespace motion_control {
class ForceControl;
class Mpc;
class Rkd;
class Rmp;
class Rtc;
class Wbc;
}  // namespace motion_control

namespace robo_manip::kinematics {
class Kinematics;
}  // namespace robo_manip::kinematics

namespace robo_manip::core {

struct JogCartesianConfig {
  std::string base_link{"base_link"};
  std::string link_name;
};

struct MotionContext {
  motion_control::types::PlanningParameters default_planning_parameters{
      motion_control::types::OfflinePlanningBackend::kRmp,
      motion_control::types::kDefaultSamplePeriodSec};
  bool has_default_ik_options{false};
  motion_control::IkOptions default_ik_options{};
  /// Jog 专用约束，从 jog.joint / jog.cartesian 读取，不复用 RMP 离线规划约束。
  motion_control::types::MotionLimits jog_limits{};
  /// 笛卡尔 Jog 默认 TCP 配置，从 jog.cartesian 读取；点动方向表达在 base_link 中。
  JogCartesianConfig jog_cartesian{};
  /// 全局限速比例，规划前会乘到 MoveJoint/MoveLine/MovePath 的 limits 上。
  /// 1.0 表示使用请求/配置中的原始限幅；默认 0.3，即 30%。
  double limit_scale{0.3};
  std::shared_ptr<motion_control::Rkd> rkd;
  std::shared_ptr<motion_control::Rmp> rmp;
  std::shared_ptr<motion_control::Rtc> rtc;
  std::shared_ptr<motion_control::Wbc> wbc;
  std::shared_ptr<motion_control::Mpc> mpc;
  std::shared_ptr<motion_control::ForceControl> force_control;
  std::shared_ptr<execution::RobotDriver> robot_driver;
  std::shared_ptr<execution::TrajectoryExecutor> trajectory_executor;
  std::shared_ptr<kinematics::Kinematics> kinematics;
};

inline bool isDefaultTracIkOptions(
    const motion_control::TracIkOptions& options) {
  const motion_control::TracIkOptions defaults;
  return options.timeout == defaults.timeout &&
         options.epsilon == defaults.epsilon &&
         options.solve_type == defaults.solve_type &&
         options.position_tolerance == defaults.position_tolerance &&
         options.orientation_tolerance == defaults.orientation_tolerance &&
         options.joint_names.empty() &&
         options.joint_group.empty() &&
         options.enable_preference_positions ==
             defaults.enable_preference_positions &&
         options.preference_positions.empty() &&
         options.enable_joint_limits == defaults.enable_joint_limits;
}

inline bool isDefaultPlacoIkOptions(
    const motion_control::PlacoIkOptions& options) {
  const motion_control::PlacoIkOptions defaults;
  return options.max_iterations == defaults.max_iterations &&
         options.position_tolerance == defaults.position_tolerance &&
         options.orientation_tolerance == defaults.orientation_tolerance &&
         options.enable_tolerance_check == defaults.enable_tolerance_check &&
         options.frame_task_priority == defaults.frame_task_priority &&
         options.frame_position_weight == defaults.frame_position_weight &&
         options.frame_orientation_weight == defaults.frame_orientation_weight &&
         options.solver_dt_sec == defaults.solver_dt_sec &&
         options.enable_regularization_task ==
             defaults.enable_regularization_task &&
         options.regularization_task_weight ==
             defaults.regularization_task_weight &&
         options.enable_manipulability_task ==
             defaults.enable_manipulability_task &&
         options.manipulability_task_weight ==
             defaults.manipulability_task_weight &&
         options.manipulability_task_lambda ==
             defaults.manipulability_task_lambda &&
         options.enable_joint_task == defaults.enable_joint_task &&
         options.joint_task_weight == defaults.joint_task_weight &&
         options.redundancy_preference_positions.empty() &&
         options.enable_joint_limits == defaults.enable_joint_limits &&
         options.mask_floating_base == defaults.mask_floating_base &&
         options.joint_names.empty() &&
         options.joint_group.empty() &&
         options.enable_arm_angle_task == defaults.enable_arm_angle_task &&
         options.arm_angle_task_weight == defaults.arm_angle_task_weight &&
         options.arm_angle_shoulder_frame.empty() &&
         options.arm_angle_elbow_frame.empty() &&
         options.arm_angle_wrist_frame.empty() &&
         options.arm_angle_elbow_hint_frame ==
             defaults.arm_angle_elbow_hint_frame &&
         options.arm_angle_elbow_hint_axis ==
             defaults.arm_angle_elbow_hint_axis;
}

inline bool isDefaultIkOptions(const motion_control::IkOptions& options) {
  const motion_control::IkOptions defaults;
  return options.solver == defaults.solver &&
         isDefaultTracIkOptions(options.trac_ik) &&
         isDefaultPlacoIkOptions(options.placo);
}

inline motion_control::IkOptions resolveIkOptions(
    const MotionContext& context,
    const motion_control::IkOptions& options) {
  if (context.has_default_ik_options && isDefaultIkOptions(options)) {
    return context.default_ik_options;
  }
  return options;
}

inline motion_control::types::PlanningParameters resolvePlanningParameters(
    const MotionContext& context,
    motion_control::types::PlanningParameters parameters) {
  if (parameters.offline_planning_backend ==
      motion_control::types::OfflinePlanningBackend::kContextDefault) {
    parameters.offline_planning_backend =
        context.default_planning_parameters.offline_planning_backend;
  }
  if (parameters.offline_planning_backend ==
      motion_control::types::OfflinePlanningBackend::kContextDefault) {
    parameters.offline_planning_backend =
        motion_control::types::OfflinePlanningBackend::kRmp;
  }

  if (parameters.sample_period_sec <= 0.0) {
    parameters.sample_period_sec =
        context.default_planning_parameters.sample_period_sec;
  }
  if (parameters.sample_period_sec <= 0.0) {
    parameters.sample_period_sec =
        motion_control::types::kDefaultSamplePeriodSec;
  }
  return parameters;
}

inline void scalePositive(double scale, double* value) {
  if (value != nullptr && *value > 0.0) {
    *value *= scale;
  }
}

inline void scalePositiveVector(double scale, std::vector<double>* values) {
  if (values == nullptr) {
    return;
  }
  for (double& value : *values) {
    if (value > 0.0) {
      value *= scale;
    }
  }
}

inline motion_control::types::MotionLimits scaleMotionLimits(
    const MotionContext& context,
    motion_control::types::MotionLimits limits) {
  const double scale = context.limit_scale > 0.0 ? context.limit_scale : 1.0;
  scalePositiveVector(scale, &limits.joint_max_velocity);
  scalePositiveVector(scale, &limits.joint_max_acceleration);
  scalePositiveVector(scale, &limits.joint_max_jerk);
  scalePositive(scale, &limits.cartesian_max_linear_velocity);
  scalePositive(scale, &limits.cartesian_max_linear_acceleration);
  scalePositive(scale, &limits.cartesian_max_linear_jerk);
  scalePositive(scale, &limits.cartesian_max_angular_velocity);
  scalePositive(scale, &limits.cartesian_max_angular_acceleration);
  scalePositive(scale, &limits.cartesian_max_angular_jerk);
  return limits;
}

}  // namespace robo_manip::core
