#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core_module.hpp"
#include "types/geometry.hpp"
#include "types/task.hpp"

namespace motion_control {

class RtcJointRuckigContext;
class RtcCartesianRuckigContext;

/// 笛卡尔空间目标变化时的轨迹模式。
enum class RtcCartesianMode {
  /// 始终沿当前位置到目标点的严格直线规划。
  kStrictLine,
  /// 目标变化后生成 Bezier 过渡路径，并保证末段为直线。
  kSmoothRetargetThenLine,
  /// 始终用 XYZ 三轴一起规划，速度更连续但不保证直线。
  kSmoothXYZ,
};

enum class RtcCartesianPhase {
  kStrictLine,
  kRetargetBlend,
};

/// 实时轨迹控制器的输入输出限幅和滤波配置。
struct RtcConfig {
  /// 输入目标低通滤波截止频率，单位 Hz。
  double input_cutoff_hz{10.0};
  /// 控制器输出频率，单位 Hz。
  double output_rate_hz{250.0};
  /// 输出最大速度。
  double max_velocity{0.0};
  /// 输出最大加速度。
  double max_acceleration{0.0};
  /// 输出最大 jerk；<= 0 时不限制 jerk。
  double max_jerk{0.0};
  /// 笛卡尔线速度限制，单位 mm/s。
  double cartesian_max_linear_velocity{0.0};
  /// 笛卡尔线加速度限制，单位 mm/s^2。
  double cartesian_max_linear_acceleration{0.0};
  /// 笛卡尔线 jerk 限制，单位 mm/s^3；<= 0 时不限制 jerk。
  double cartesian_max_linear_jerk{0.0};
  /// 笛卡尔角速度限制，单位 rad/s。
  double cartesian_max_angular_velocity{0.0};
  /// 笛卡尔角加速度限制，单位 rad/s^2。
  double cartesian_max_angular_acceleration{0.0};
  /// 笛卡尔角 jerk 限制，单位 rad/s^3；<= 0 时不限制 jerk。
  double cartesian_max_angular_jerk{0.0};
  /// 笛卡尔目标变化时的轨迹模式。
  RtcCartesianMode cartesian_mode{RtcCartesianMode::kStrictLine};
  /// 平滑转向至少持续的时间，单位 s。
  double cartesian_retarget_min_blend_time{0.05};
  /// 切回严格直线时允许的位置到新直线距离，单位 mm。
  double cartesian_retarget_position_tolerance{0.5};
  /// 切回严格直线时允许的速度方向夹角，单位 rad。
  double cartesian_retarget_angle_tolerance{0.05235987755982989};
  /// 切回严格直线时允许的横向速度，单位 mm/s。
  double cartesian_retarget_lateral_velocity_tolerance{1.0};

  bool validForJointTrajectory() const;
  std::string jointTrajectoryValidationError() const;
  bool validForCartesianTrajectory() const;
  std::string cartesianTrajectoryValidationError() const;
};

/// 实时轨迹规划状态。
enum class RtcStatus {
  kRunning,
  kReached,
  kInvalidInput,
  kNotConfigured,
  kFailed,
};

/// 关节空间实时目标。
///
/// positions 使用 motion_control 约定的 deg，velocities 为 deg/s，
/// accelerations 为 deg/s^2。velocities/accelerations 可为空，由实现决定默认值。
struct RtcJointTarget {
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

/// 关节空间实时速度目标。
///
/// velocities 使用 motion_control 约定的 deg/s，accelerations 为 deg/s^2。
/// accelerations 可为空，由实现按零目标加速度处理。
/// 该目标用于 Jog 等“按住按钮持续运动”的速度保活场景。
struct RtcJointVelocityTarget {
  std::vector<std::string> joint_names;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

/// 关节空间实时规划输出。
struct RtcJointState {
  RtcStatus status{RtcStatus::kFailed};
  std::string message;
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

/// 笛卡尔空间实时目标。
///
/// pose.position 使用 motion_control 约定的 mm，twist.linear 为 mm/s，
/// twist.angular 为 rad/s。base_link 是 IK/控制参考起点，link_name 是被控制末端。
struct RtcCartesianTarget {
  std::string base_link{"base_link"};
  std::string link_name;
  types::Pose pose{};
  types::Twist twist{};
  types::Twist acceleration{};
};

/// 笛卡尔空间实时规划输出。
struct RtcCartesianState {
  RtcStatus status{RtcStatus::kFailed};
  std::string message;
  std::string base_link{"base_link"};
  std::string link_name;
  types::Pose pose{};
  types::Twist twist{};
  types::Twist acceleration{};
};

/// 将离散目标转换为平滑实时目标的控制模块。
class Rtc : public CoreModule {
 public:
  Rtc();
  ~Rtc() override;

  Rtc(const Rtc&) = delete;
  Rtc& operator=(const Rtc&) = delete;
  Rtc(Rtc&&) noexcept;
  Rtc& operator=(Rtc&&) noexcept;

  std::string name() const override { return "RTC"; }

  virtual void configure(const RtcConfig& config);
  virtual const RtcConfig& config() const;

  /// 重置控制器内部状态。
  virtual void reset(const types::Target& target);
  /// 根据输入目标和时间步长更新实时目标。
  virtual types::Target update(const types::Target& target, double dt_sec);

  /// 重置关节空间实时规划器状态。
  virtual void resetJointTarget(const RtcJointTarget& target);
  /// 输入实时关节目标，输出当前控制周期应跟踪的平滑关节状态。
  virtual RtcJointState updateJointTarget(const RtcJointTarget& target,
                                          double dt_sec);
  /// 重置关节空间速度规划器状态。
  virtual void resetJointVelocityTarget(const RtcJointVelocityTarget& target);
  /// 输入实时关节速度目标，输出当前控制周期应跟踪的平滑关节状态。
  virtual RtcJointState updateJointVelocityTarget(
      const RtcJointVelocityTarget& target,
      double dt_sec);

  /// 重置笛卡尔空间实时规划器状态。
  virtual void resetCartesianTarget(const RtcCartesianTarget& target);
  /// 输入实时笛卡尔目标，输出当前控制周期应跟踪的平滑笛卡尔状态。
  virtual RtcCartesianState updateCartesianTarget(
      const RtcCartesianTarget& target,
      double dt_sec);

 private:
  RtcConfig config_{};
  RtcJointTarget last_joint_target_{};
  RtcJointVelocityTarget last_joint_velocity_target_{};
  RtcCartesianTarget last_cartesian_target_{};
  RtcJointState current_joint_state_{};
  bool joint_state_initialized_{false};
  std::unique_ptr<RtcJointRuckigContext> joint_context_;
  RtcCartesianState current_cartesian_state_{};
  bool cartesian_state_initialized_{false};
  std::unique_ptr<RtcCartesianRuckigContext> cartesian_context_;
  bool cartesian_command_target_initialized_{false};
  RtcCartesianPhase cartesian_phase_{RtcCartesianPhase::kStrictLine};
  types::Pose cartesian_retarget_start_pose_{};
  types::Pose cartesian_retarget_target_pose_{};
  std::vector<types::Vector3> cartesian_retarget_path_points_{};
  std::vector<double> cartesian_retarget_path_lengths_{};
  double cartesian_retarget_path_progress_{0.0};
  double cartesian_retarget_elapsed_sec_{0.0};
};

}  // 命名空间 motion_control
