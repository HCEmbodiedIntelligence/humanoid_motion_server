#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace humanoid_motion_server::kinematics {

enum class ErrorCode {
  kOk = 0,
  kNotConfigured,
  kInvalidArgument,
  kInvalidGroup,
  kInvalidFrame,
  kInvalidTool,
  kInvalidJointState,
  kSdkInitializationFailed,
  kSdkModelLoadFailed,
  kSdkFailure,
  kInternalError,
};

struct Status {
  ErrorCode code{ErrorCode::kOk};
  std::string message;

  bool ok() const noexcept { return code == ErrorCode::kOk; }
  explicit operator bool() const noexcept { return ok(); }

  static Status Ok() { return {}; }
  static Status Error(ErrorCode error_code, std::string error_message) {
    return {error_code, std::move(error_message)};
  }
};

template <typename T>
struct Result {
  Status status;
  std::optional<T> value;

  bool ok() const noexcept { return status.ok() && value.has_value(); }
  explicit operator bool() const noexcept { return ok(); }

  static Result Success(T result_value) {
    return {Status::Ok(), std::move(result_value)};
  }
  static Result Failure(Status error) {
    return {std::move(error), std::nullopt};
  }
};

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Public poses use SI: translation in metres and a unit quaternion.
struct Pose {
  Vector3 position_m;
  Quaternion orientation;
};

// positions_rad is indexed by joint_names, not by an implicit model order.
struct JointState {
  std::vector<std::string> joint_names;
  std::vector<double> positions_rad;
};

struct JointGroupDefinition {
  std::string group_name;
  std::vector<std::string> joint_names;
};

struct Configuration {
  std::string model_path;
  std::vector<JointGroupDefinition> joint_groups;
};

struct KinematicsRequest {
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
};

struct ForwardKinematicsRequest {
  KinematicsRequest kinematics;
  JointState joint_state;
};

struct IkParameters {
  bool enable_joint_task{false};
  std::optional<JointState> redundancy_preference;

  bool enable_arm_angle_constraint{false};
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;
  Vector3 arm_angle_elbow_hint_axis{0.0, 0.0, 1.0};

  double position_tolerance_m{0.0005};
  double orientation_tolerance_rad{1e-3};
  double trac_ik_timeout_sec{0.02};

  int placo_max_iterations{1000};
  double retry_position_tolerance_m{0.001};
  double retry_orientation_tolerance_rad{1e-2};
  int retry_interpolation_steps{80};
  double retry_min_position_distance_m{0.005};
  double retry_min_orientation_distance_rad{0.025};
  double placo_joint_task_weight{1e-3};
};

struct InverseKinematicsRequest {
  KinematicsRequest kinematics;
  Pose target_pose;

  // An explicit seed wins over current_state. If neither is supplied, the
  // original SDK remains responsible for obtaining live state from its driver.
  std::optional<JointState> seed;
  std::optional<JointState> current_state;
  IkParameters parameters;
};

struct InverseKinematicsResult {
  JointState solution;
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

// Tool orientation is the SDK's ZYX roll/pitch/yaw convention, expressed in
// radians. Translation remains SI metres at this public boundary.
struct ToolTransform {
  std::string name;
  std::string parent_link;
  Vector3 translation_m;
  Vector3 rpy_rad;
};

}  // namespace humanoid_motion_server::kinematics
