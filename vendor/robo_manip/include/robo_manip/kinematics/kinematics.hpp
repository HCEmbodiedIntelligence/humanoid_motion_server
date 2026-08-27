#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rkd.hpp"
#include "robo_manip/core/motion_context.hpp"
#include "types/geometry.hpp"
#include "types/robot_state.hpp"

namespace robo_manip::kinematics {

using CartesianPos = motion_control::types::Pose;
using JointPos = std::vector<double>;
using ToolTransform = motion_control::ToolTransform;

/// 一次 FK/IK 查询对应的运动学链。
///
/// group_name 决定输入/输出关节顺序；base_link 是目标位姿表达坐标系；
/// link_name 可以是 URDF frame/link，也可以是通过 setToolTransform() 注册的工具名。
struct KinematicsRequest {
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
};

/// 逆解参数，语义对齐 HC_ROBOT::OptParams 并补齐 RKD 后端参数。
///
/// 默认先使用 TRAC-IK；失败后继续用 TRAC-IK 沿当前 TCP 到目标 TCP
/// 做插值重试。关节角单位为 deg，位置容差单位为 mm，姿态容差单位为 rad。
struct IkParameters {
  /// true 时使用 q0_deg 或 planning_start_joint 作为参考初值；
  /// false 时优先从 context.robot_driver 读取当前关节状态。
  bool plan_only{false};
  std::vector<double> planning_start_joint;

  /// 启用后把 redundancy_preference_positions 传给 TRAC-IK preference，
  /// 帮助 7 轴冗余臂选择靠近偏好的解。
  bool enable_joint_task{false};
  std::vector<double> redundancy_preference_positions;

  /// 保留给 PlaCo 后端的臂角约束字段；当前 TRAC-IK 插值重试不使用。
  bool enable_arm_angle_constraint{false};
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;
  Eigen::Vector3d arm_angle_elbow_hint_axis = Eigen::Vector3d::UnitZ();

  double position_tolerance{0.5};
  double orientation_tolerance{1e-3};
  double trac_ik_timeout{0.02};

  /// 以下 PlaCo/retry tolerance 字段为兼容旧接口保留；
  /// 当前 TRAC-IK 插值重试仍使用 position_tolerance/orientation_tolerance。
  int placo_max_iterations{1000};
  double retry_position_tolerance{1.0};
  double retry_orientation_tolerance{1e-2};
  int retry_interpolation_steps{80};
  double retry_min_position_distance{5.0};
  double retry_min_orientation_distance{0.025};
  double placo_joint_task_weight{1e-3};
};

/// 逆运动学完整请求。
///
/// parameters 不填时使用 IkParameters 默认值；target_pose 使用 base_link 表达，
/// link_name 命中工具名时按工具 TCP 目标求解。
struct InverseKinematicsRequest {
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  motion_control::types::Pose target_pose{};
  std::vector<double> q0_deg;
  IkParameters parameters{};
};

/// robo_manip 层包装后的 IK 结果。
///
/// positions 与 joint_names 使用 KinematicsRequest::group_name 注册的顺序。
struct IkResult {
  bool success{false};
  std::string message;
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  double position_error{0.0};
  double orientation_error{0.0};
};

class Kinematics {
 public:
  explicit Kinematics(core::MotionContext context);

  /// 正运动学：输入关节角 deg，输出 Pose 位置 mm、姿态 euler_zyx_deg 为 deg。
  motion_control::types::Pose forwardKinematics(
      const KinematicsRequest& request,
      const std::vector<double>& joints_deg) const;

  /// 逆运动学：默认 TRAC-IK，失败后 TRAC-IK 插值重试。
  IkResult inverseKinematics(const InverseKinematicsRequest& request) const;

  /// 工具 TCP 设置/查询接口，内部转调 context.rkd，并与 MoveLine/MovePath 使用
  /// 同一份工具表。
  void setToolTransform(const ToolTransform& tool_transform) const;
  bool getToolTransform(const std::string& tool_name,
                        ToolTransform* tool_transform) const;
  std::vector<ToolTransform> getToolTransforms() const;

 private:
  core::MotionContext context_;
};

}  // namespace robo_manip::kinematics
