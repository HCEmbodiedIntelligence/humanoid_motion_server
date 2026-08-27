#pragma once

#include <optional>
#include <string>
#include <vector>

#include "types/geometry.hpp"
#include "types/ik.hpp"
#include "types/trajectory.hpp"

namespace motion_control::types {

inline constexpr double kDefaultSamplePeriodSec = 0.02;

/// 规划任务在调度流程中的生命周期状态。
enum class TaskStatus {
  kIdle,
  kPlanning,
  kRunning,
  kSucceeded,
  kFailed,
  kCanceled,
};

/// 规划或控制模块返回给上层的结果码。
enum class ResultCode {
  kOk,
  kInvalidRequest,
  kNotConfigured,
  kNotImplemented,
  kIkFailed,
  kPlanningFailed,
  kLimitViolation,
  kSingularityRisk,
  kCollisionRisk,
  kTimeout,
  kCanceled,
  kInternalError,
};

/// 轨迹规划使用的运动限制。
struct MotionLimits {
  /// 关节最大速度，单位 deg/s。大小应与参与规划的关节数一致。
  std::vector<double> joint_max_velocity;
  /// 关节最大加速度，单位 deg/s^2。留空时不启用关节加速度约束。
  std::vector<double> joint_max_acceleration;
  /// 关节最大 jerk，单位 deg/s^3。当前仅作为扩展字段保留。
  std::vector<double> joint_max_jerk;
  /// 笛卡尔最大线速度，单位 mm/s。
  double cartesian_max_linear_velocity{0.0};
  /// 笛卡尔最大线加速度，单位 mm/s^2。
  double cartesian_max_linear_acceleration{0.0};
  /// 笛卡尔最大线 jerk，单位 mm/s^3；<= 0 时不限制 jerk。
  double cartesian_max_linear_jerk{0.0};
  /// 笛卡尔最大角速度，单位 rad/s。
  double cartesian_max_angular_velocity{0.0};
  /// 笛卡尔最大角加速度，单位 rad/s^2。
  double cartesian_max_angular_acceleration{0.0};
  /// 笛卡尔最大角 jerk，单位 rad/s^3；<= 0 时不限制 jerk。
  double cartesian_max_angular_jerk{0.0};
};

/// 关节路径的几何插值方式，后续再交给 TOPPRA 做时间参数化。
enum class JointPathGeometryMode {
  kCubicHermite = 0,
  kNotAKnotCubicSpline = 1,
};

/// 基础任务离线规划后端。
enum class OfflinePlanningBackend {
  kContextDefault = 0,
  kRmp = 1,
  kRtc = 2,
};

/// 规划器可调参数。这里保持结构化字段，避免用字符串 key 传参。
struct PlanningParameters {
  /// MoveJoint/MoveLine 离线规划使用的后端；默认继承系统配置。
  OfflinePlanningBackend offline_planning_backend{
      OfflinePlanningBackend::kContextDefault};

  /// 输出轨迹采样周期，单位 s。
  /// <= 0 表示继承系统配置；系统配置缺省时使用 kDefaultSamplePeriodSec。
  double sample_period_sec{0.0};
  /// 关节路径的几何插值方式。
  JointPathGeometryMode joint_path_geometry_mode{
      JointPathGeometryMode::kNotAKnotCubicSpline};

  /// 是否让 TOPPRA 根据路径曲率自动生成网格点。
  bool auto_gridpoints{true};
  /// 自动网格点的最大插值误差阈值。
  double gridpoint_max_err_threshold{1e-2};
  /// 自动网格点搜索的最大迭代次数。
  int gridpoint_max_iteration{100};
  /// 自动网格点的最小点数。
  int gridpoint_min_nb_points{80};
  /// 自动网格点的最大分段长度；<= 0 时由点数自动推导。
  double gridpoint_max_seg_length{0.0};

  /// 是否对 TOPPRA 的路径速度平方结果做路径域平滑。
  bool smooth_parametrization{false};
  /// 路径域平滑窗口大小。
  int smooth_parametrization_window{9};
  /// 是否导出路径域平滑调试 CSV。
  bool debug_parametrization_smoothing{false};

  /// 是否对最终采样后的关节轨迹做移动平均平滑。
  bool smooth_output_trajectory{true};
  /// 输出轨迹移动平均窗口大小。
  int smooth_output_trajectory_window{11};
  /// 输出轨迹平滑后允许的终点误差。
  double smooth_output_trajectory_tolerance{1e-3};
  /// 为收敛到终点最多追加的采样点数。
  int smooth_output_trajectory_max_extra_points{100};

  /// 笛卡尔 B 样条离散采样点数；<=0 时按控制路径长度和姿态角度自适应，
  /// 约 1mm 或 2deg 一个间隔，内部默认最多 500 点。
  int cartesian_spline_sample_count{0};
};

/// 单个末端目标，link_name 对应机器人模型中的末端 frame/link。
struct Target {
  /// IK 链起点 link；默认保持历史行为，从 base_link 到 link_name。
  std::string base_link{"base_link"};
  std::string link_name;
  Pose pose{};
  std::optional<Twist> twist;
};

struct PlanningRequest {
  std::string request_id;
  std::string group_name;
  MotionLimits limits{};
  /// 可选的关节空间几何路径，由 RMP/TOPPRA 重新做时间参数化。
  ///
  /// 当前状态会作为路径起点；points 中的位置是后续要经过的路点，单位 deg。
  /// 输入点上的时间戳只作为提示，规划器会根据运动限制重新计算时间。
  std::optional<JointTrajectory> joint_path;
  /// 可选的笛卡尔空间几何路径，由 RMP/TOPPRA 重新做时间参数化。
  ///
  /// points 表示当前状态之后的笛卡尔途径点和目标点，最后一个点为目标点；
  /// 路径起点由当前 RobotState 通过 cartesian_path.link_name 正解得到。
  /// 位姿位置使用 motion_control 层统一的单位约定，目前为 mm。
  std::optional<CartesianTrajectory> cartesian_path;
  /// 笛卡尔路径离散成关节路径时使用的 IK 后端和求解参数。
  motion_control::IkOptions ik_options{};
  PlanningParameters parameters{};
};

/// 规划结果，可同时携带关节轨迹和笛卡尔参考轨迹。
struct PlanningResult {
  bool success{false};
  ResultCode code{ResultCode::kInternalError};
  TaskStatus status{TaskStatus::kFailed};
  std::string message;
  /// 规划结果对应的关节组，执行层可用它选择硬件执行通道。
  std::string group_name;
  double stamp_sec{0.0};
  double duration_sec{0.0};
  std::optional<JointTrajectory> joint_trajectory;
  std::optional<CartesianTrajectory> cartesian_trajectory;
};

}  // 命名空间 motion_control::types
