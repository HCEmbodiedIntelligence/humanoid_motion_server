#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace motion_control {

struct PlacoIkOptions {
  /// 最大 QP 迭代次数。每次迭代会应用一次 placo 求出的 delta-q。
  int max_iterations{100};
  /// 位置收敛阈值，单位 mm。
  double position_tolerance{1.0};
  /// 姿态收敛阈值，单位 rad。
  double orientation_tolerance{1e-3};
  /// 是否使用位置/姿态容差判断收敛。关闭时接受首次 QP 迭代结果。
  bool enable_tolerance_check{true};
  /// FrameTask 优先级，可取 "soft"、"hard" 或 "scaled"。
  std::string frame_task_priority{"hard"};
  double frame_position_weight{1.0};
  double frame_orientation_weight{1.0};
  /// PlaCo 求解器周期，单位 s。正值会写入 solver.dt。
  double solver_dt_sec{0.0};
  /// 是否增加 delta-q 正则任务。
  bool enable_regularization_task{false};
  double regularization_task_weight{1e-4};
  /// 是否增加末端可操作度任务（position + orientation）。
  bool enable_manipulability_task{false};
  double manipulability_task_weight{1e-3};
  double manipulability_task_lambda{1.0};
  /// 是否增加 JointTask，把解拉向 redundancy_preference_positions，帮助选择冗余解。
  bool enable_joint_task{false};
  /// JointTask 权重，仅在 enable_joint_task=true 时生效。
  double joint_task_weight{1e-3};
  /// JointTask 的冗余解偏好关节角，单位 deg；顺序与本次 IK 输出 joint_names 一致。
  std::vector<double> redundancy_preference_positions;
  /// 是否在 IK 收敛成功后校验模型关节位置限位。
  bool enable_joint_limits{true};
  /// placo 默认使用 floating base；固定底座机械臂通常应保持 mask。
  bool mask_floating_base{true};
  /// 指定求解和返回的关节。为空时优先使用 joint_group，再使用 reference_state
  /// 中 group_name 对应的 RKD 关节组。
  std::vector<std::string> joint_names;
  std::string joint_group;
  /// 是否增加 ArmAngleTask，引导七轴冗余解让肘部靠近期望偏置方向。
  bool enable_arm_angle_task{false};
  /// ArmAngleTask 权重，仅在 enable_arm_angle_task=true 时生效。
  double arm_angle_task_weight{1e-7};
  /// 臂角几何定义用到的肩、肘、腕 frame。为空时按末端名推断常见左/右臂或测试模型。
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;
  /// 期望肘部偏置方向由某个 frame 下的局部轴给出；内部会转成世界坐标并投影到垂直肩腕轴的平面。
  /// frame 为 "world" 时 arm_angle_elbow_hint_axis 直接按世界坐标解释。
  std::string arm_angle_elbow_hint_frame{"world"};
  Eigen::Vector3d arm_angle_elbow_hint_axis = Eigen::Vector3d::UnitZ();
};

enum class TracIkSolveType {
  kSpeed,
  kDistance,
  kManip1,
  kManip2,
};

struct TracIkOptions {
  /// 每次 TRAC-IK 求解允许的最长时间，单位 s。
  double timeout{0.005};
  /// TRAC-IK 内部收敛精度，单位 m/rad。
  double epsilon{1e-5};
  TracIkSolveType solve_type{TracIkSolveType::kSpeed};
  /// 输出结果的外层校验阈值，位置单位 mm，姿态单位 rad。
  double position_tolerance{1.0};
  double orientation_tolerance{1e-3};
  /// 指定求解和返回的关节。为空时优先使用 joint_group，再使用 reference_state
  /// 中 group_name 对应的 RKD 关节组，最后使用 KDL chain 中的可动关节。
  std::vector<std::string> joint_names;
  std::string joint_group;
  /// 是否优先使用 preference_positions 作为 seed，并用 Distance 模式求解。
  /// 若该模式失败，会自动回退到 reference_state seed + Speed 模式。
  bool enable_preference_positions{false};
  /// 偏好关节角，单位 deg；顺序与本次 TRAC-IK 输出 joint_names 一致。
  std::vector<double> preference_positions;
  /// 是否在 IK 成功后校验 TRAC-IK/KDL 返回的关节限位。
  bool enable_joint_limits{true};
};

enum class IkSolverType {
  kTracIk,
  kPlaco,
};

struct IkOptions {
  IkSolverType solver{IkSolverType::kTracIk};
  TracIkOptions trac_ik;
  PlacoIkOptions placo;
};

}  // namespace motion_control
