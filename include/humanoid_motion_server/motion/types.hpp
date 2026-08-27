#ifndef HUMANOID_MOTION_SERVER__TYPES_HPP_
#define HUMANOID_MOTION_SERVER__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace humanoid_motion_server::motion
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

enum class MotionKind
{
  MOVE_J,
  MOVE_L,
  MOVE_P,
  SERVO_J,
  SERVO_P,
};

enum class StatusCode
{
  OK,
  INVALID_ARGUMENT,
  NOT_CONFIGURED,
  REJECTED,
  PREEMPTED,
  CANCELED,
  TIMEOUT,
  STALE_FEEDBACK,
  SDK_ERROR,
  LIMIT_VIOLATION,
  CONTROLLED_STOP,
  INTERNAL_ERROR,
};

struct MotionStatus
{
  StatusCode code{StatusCode::OK};
  std::string message;
  /// Exact SDK entry point that produced sdk_code/message, when applicable.
  std::string sdk_api;
  /// Numeric value of the native SDK result enum. -1 means the SDK API only
  /// returned bool and supplied no native numeric code.
  std::int64_t sdk_code{-1};

  MotionStatus() = default;
  MotionStatus(
    StatusCode status_code, std::string status_message,
    std::string native_api = {}, std::int64_t native_code = -1)
  : code(status_code), message(std::move(status_message)),
    sdk_api(std::move(native_api)), sdk_code(native_code)
  {
  }

  bool ok() const {return code == StatusCode::OK;}

  static MotionStatus Ok() {return {};}
};

/// Public Cartesian pose in SI units. Position is metres and orientation is a
/// unit quaternion in x, y, z, w order.
struct Pose
{
  std::array<double, 3> position_m{{0.0, 0.0, 0.0}};
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

/// A named joint target in rad, rad/s and rad/s^2.
struct JointTarget
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<double> accelerations_rad_s2;
};

/// Real robot feedback. received_at is populated by the receiving process from
/// its steady clock; it is deliberately unrelated to a transport timestamp.
struct JointFeedback
{
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  SteadyTime received_at{};
};

struct MotionLimits
{
  std::vector<double> joint_max_velocity_rad_s;
  std::vector<double> joint_max_acceleration_rad_s2;
  std::vector<double> joint_max_jerk_rad_s3;
  double cartesian_max_linear_velocity_m_s{0.0};
  double cartesian_max_linear_acceleration_m_s2{0.0};
  double cartesian_max_linear_jerk_m_s3{0.0};
  double cartesian_max_angular_velocity_rad_s{0.0};
  double cartesian_max_angular_acceleration_rad_s2{0.0};
  double cartesian_max_angular_jerk_rad_s3{0.0};
};

struct JointGroupModel
{
  std::string name;
  std::vector<std::string> joint_names;
  std::vector<double> lower_position_rad;
  std::vector<double> upper_position_rad;
};

struct MoveJRequest
{
  std::string request_id;
  std::string group_name;
  JointTarget target;
  MotionLimits limits;
};

struct MoveLRequest
{
  std::string request_id;
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  Pose target;
  MotionLimits limits;
};

struct MovePRequest
{
  std::string request_id;
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  std::vector<Pose> waypoints;
  MotionLimits limits;
};

struct ServoJRequest
{
  std::string request_id;
  std::string group_name;
  JointTarget target;
  MotionLimits limits;
};

struct ServoPRequest
{
  std::string request_id;
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  Pose target;
  MotionLimits limits;
};

using SessionRequest =
  std::variant<MoveJRequest, MoveLRequest, MovePRequest, ServoJRequest, ServoPRequest>;
using DynamicTarget = std::variant<JointTarget, Pose>;

struct JointCommand
{
  std::string group_name;
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<double> accelerations_rad_s2;
  /// True only when produced by ISdkMotionBackend::updateFinalJointTarget.
  bool passed_final_sdk_rtc{false};
};

struct BackendTick
{
  MotionStatus status;
  JointCommand candidate;
  bool path_reached{false};
};

enum class SessionState
{
  PENDING,
  RUNNING,
  SUCCEEDED,
  PREEMPTED,
  CANCELED,
  ABORTED,
};

struct SessionEvent
{
  std::string session_id;
  SessionState state{SessionState::PENDING};
  MotionStatus status;
};

struct PipelineTick
{
  std::vector<JointCommand> commands;
  std::vector<SessionEvent> events;
};

inline MotionKind motion_kind(const SessionRequest & request)
{
  return std::visit(
    [](const auto & value) -> MotionKind {
      using RequestT = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<RequestT, MoveJRequest>) {
        return MotionKind::MOVE_J;
      } else if constexpr (std::is_same_v<RequestT, MoveLRequest>) {
        return MotionKind::MOVE_L;
      } else if constexpr (std::is_same_v<RequestT, MovePRequest>) {
        return MotionKind::MOVE_P;
      } else if constexpr (std::is_same_v<RequestT, ServoJRequest>) {
        return MotionKind::SERVO_J;
      } else {
        return MotionKind::SERVO_P;
      }
    }, request);
}

inline bool is_servo(const MotionKind kind)
{
  return kind == MotionKind::SERVO_J || kind == MotionKind::SERVO_P;
}

inline bool is_move(const MotionKind kind)
{
  return !is_servo(kind);
}

}  // namespace humanoid_motion_server::motion

#endif  // HUMANOID_MOTION_SERVER__TYPES_HPP_
