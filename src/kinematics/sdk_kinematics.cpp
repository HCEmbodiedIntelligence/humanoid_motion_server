#include "humanoid_motion_server/kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "humanoid_motion_server/kinematics/sdk_api.hpp"

namespace humanoid_motion_server::kinematics {
namespace {

constexpr double kDegreesPerRadian =
    57.295779513082320876798154814105;
constexpr double kRadiansPerDegree =
    0.01745329251994329576923690768489;
constexpr double kMillimetresPerMetre = 1000.0;
constexpr double kQuaternionNormEpsilon = 1e-12;

bool finite(double value) { return std::isfinite(value); }

bool finite(const Vector3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

bool finite(const Quaternion& value) {
  return finite(value.x) && finite(value.y) && finite(value.z) &&
         finite(value.w);
}

Result<Quaternion> normalizedQuaternion(const Quaternion& input) {
  if (!finite(input)) {
    return Result<Quaternion>::Failure(Status::Error(
        ErrorCode::kInvalidArgument, "orientation quaternion is not finite"));
  }
  const double squared_norm = input.x * input.x + input.y * input.y +
                              input.z * input.z + input.w * input.w;
  if (!finite(squared_norm) || squared_norm < kQuaternionNormEpsilon) {
    return Result<Quaternion>::Failure(Status::Error(
        ErrorCode::kInvalidArgument, "orientation quaternion has zero norm"));
  }
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  return Result<Quaternion>::Success(
      {input.x * inverse_norm, input.y * inverse_norm,
       input.z * inverse_norm, input.w * inverse_norm});
}

Vector3 quaternionToEulerZyxDeg(const Quaternion& q) {
  const double sin_roll_cos_pitch = 2.0 * (q.w * q.x + q.y * q.z);
  const double cos_roll_cos_pitch =
      1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

  const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::asin(std::clamp(sin_pitch, -1.0, 1.0));

  const double sin_yaw_cos_pitch = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw_cos_pitch =
      1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

  return {roll * kDegreesPerRadian, pitch * kDegreesPerRadian,
          yaw * kDegreesPerRadian};
}

Quaternion eulerZyxDegToQuaternion(const Vector3& euler_deg) {
  const double half_roll = euler_deg.x * kRadiansPerDegree * 0.5;
  const double half_pitch = euler_deg.y * kRadiansPerDegree * 0.5;
  const double half_yaw = euler_deg.z * kRadiansPerDegree * 0.5;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  return {sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy,
          cr * cp * cy + sr * sp * sy};
}

Status validateNames(const std::vector<std::string>& names,
                     ErrorCode error_code,
                     const std::string& label) {
  std::unordered_set<std::string> unique_names;
  for (const std::string& name : names) {
    if (name.empty()) {
      return Status::Error(error_code, label + " contains an empty name");
    }
    if (!unique_names.insert(name).second) {
      return Status::Error(error_code,
                           label + " contains duplicate name '" + name + "'");
    }
  }
  return Status::Ok();
}

Status validateRequestNames(const KinematicsRequest& request) {
  if (request.group_name.empty()) {
    return Status::Error(ErrorCode::kInvalidGroup, "group_name is empty");
  }
  if (request.base_link.empty()) {
    return Status::Error(ErrorCode::kInvalidFrame, "base_link is empty");
  }
  if (request.link_name.empty()) {
    return Status::Error(ErrorCode::kInvalidFrame, "link_name is empty");
  }
  return Status::Ok();
}

Result<std::vector<double>> reorderJointPositions(
    const JointState& state,
    const std::vector<std::string>& ordered_joint_names) {
  if (state.joint_names.size() != state.positions_rad.size()) {
    return Result<std::vector<double>>::Failure(Status::Error(
        ErrorCode::kInvalidJointState,
        "joint_names and positions_rad have different lengths"));
  }
  Status names_status = validateNames(
      state.joint_names, ErrorCode::kInvalidJointState, "joint state");
  if (!names_status) {
    return Result<std::vector<double>>::Failure(std::move(names_status));
  }

  std::unordered_map<std::string, double> by_name;
  by_name.reserve(state.joint_names.size());
  for (std::size_t index = 0; index < state.joint_names.size(); ++index) {
    if (!finite(state.positions_rad[index])) {
      return Result<std::vector<double>>::Failure(Status::Error(
          ErrorCode::kInvalidJointState,
          "joint position for '" + state.joint_names[index] +
              "' is not finite"));
    }
    by_name.emplace(state.joint_names[index], state.positions_rad[index]);
  }

  if (by_name.size() != ordered_joint_names.size()) {
    return Result<std::vector<double>>::Failure(Status::Error(
        ErrorCode::kInvalidJointState,
        "joint state does not contain exactly the configured group joints"));
  }

  std::vector<double> reordered;
  reordered.reserve(ordered_joint_names.size());
  for (const std::string& joint_name : ordered_joint_names) {
    const auto position = by_name.find(joint_name);
    if (position == by_name.end()) {
      return Result<std::vector<double>>::Failure(Status::Error(
          ErrorCode::kInvalidJointState,
          "joint state is missing configured joint '" + joint_name + "'"));
    }
    reordered.push_back(position->second);
  }
  return Result<std::vector<double>>::Success(std::move(reordered));
}

std::vector<double> radiansToDegrees(const std::vector<double>& radians) {
  std::vector<double> degrees;
  degrees.reserve(radians.size());
  for (double value : radians) {
    degrees.push_back(value * kDegreesPerRadian);
  }
  return degrees;
}

std::vector<double> degreesToRadians(const std::vector<double>& degrees) {
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (double value : degrees) {
    radians.push_back(value * kRadiansPerDegree);
  }
  return radians;
}

Result<SdkPose> toSdkPose(const Pose& pose) {
  if (!finite(pose.position_m)) {
    return Result<SdkPose>::Failure(Status::Error(
        ErrorCode::kInvalidArgument, "target position is not finite"));
  }
  Result<Quaternion> orientation = normalizedQuaternion(pose.orientation);
  if (!orientation) {
    return Result<SdkPose>::Failure(std::move(orientation.status));
  }
  SdkPose sdk_pose;
  sdk_pose.position_mm = {pose.position_m.x * kMillimetresPerMetre,
                          pose.position_m.y * kMillimetresPerMetre,
                          pose.position_m.z * kMillimetresPerMetre};
  sdk_pose.orientation = *orientation.value;
  sdk_pose.euler_zyx_deg = quaternionToEulerZyxDeg(*orientation.value);
  return Result<SdkPose>::Success(std::move(sdk_pose));
}

Result<Pose> toPublicPose(const SdkPose& sdk_pose) {
  if (!finite(sdk_pose.position_mm) || !finite(sdk_pose.euler_zyx_deg)) {
    return Result<Pose>::Failure(Status::Error(
        ErrorCode::kSdkFailure, "SDK FK returned a non-finite pose"));
  }
  Pose pose;
  pose.position_m = {sdk_pose.position_mm.x / kMillimetresPerMetre,
                     sdk_pose.position_mm.y / kMillimetresPerMetre,
                     sdk_pose.position_mm.z / kMillimetresPerMetre};
  pose.orientation = eulerZyxDegToQuaternion(sdk_pose.euler_zyx_deg);
  return Result<Pose>::Success(std::move(pose));
}

SdkToolTransform toSdkToolTransform(const ToolTransform& transform) {
  return {transform.name,
          transform.parent_link,
          {transform.translation_m.x * kMillimetresPerMetre,
           transform.translation_m.y * kMillimetresPerMetre,
           transform.translation_m.z * kMillimetresPerMetre},
          transform.rpy_rad};
}

ToolTransform toPublicToolTransform(const SdkToolTransform& transform) {
  return {transform.name,
          transform.parent_link,
          {transform.translation_mm.x / kMillimetresPerMetre,
           transform.translation_mm.y / kMillimetresPerMetre,
           transform.translation_mm.z / kMillimetresPerMetre},
          transform.rpy_rad};
}

}  // namespace

class SdkKinematics::Impl {
 public:
  explicit Impl(std::shared_ptr<SdkApi> api) : sdk_api(std::move(api)) {}

  Result<std::vector<std::string>> group(
      const std::string& group_name) const {
    if (!sdk_api) {
      return Result<std::vector<std::string>>::Failure(Status::Error(
          ErrorCode::kInternalError, "SDK API injection is null"));
    }
    if (group_name.empty()) {
      return Result<std::vector<std::string>>::Failure(Status::Error(
          ErrorCode::kInvalidGroup, "group_name is empty"));
    }
    Result<std::vector<std::string>> result = sdk_api->jointGroup(group_name);
    if (!result) {
      return result;
    }
    if (result.value->empty()) {
      return Result<std::vector<std::string>>::Failure(Status::Error(
          ErrorCode::kInvalidGroup,
          "SDK joint group '" + group_name + "' is not registered"));
    }
    Status names_status = validateNames(
        *result.value, ErrorCode::kInvalidGroup, "SDK joint group");
    if (!names_status) {
      return Result<std::vector<std::string>>::Failure(
          std::move(names_status));
    }
    return result;
  }

  std::shared_ptr<SdkApi> sdk_api;
};

SdkKinematics::SdkKinematics(std::shared_ptr<SdkApi> sdk_api)
    : impl_(std::make_unique<Impl>(std::move(sdk_api))) {}

SdkKinematics::~SdkKinematics() = default;
SdkKinematics::SdkKinematics(SdkKinematics&&) noexcept = default;
SdkKinematics& SdkKinematics::operator=(SdkKinematics&&) noexcept = default;

Status SdkKinematics::configure(const Configuration& configuration) {
  if (configuration.model_path.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "model_path is empty");
  }
  Status group_names = Status::Ok();
  std::unordered_set<std::string> groups;
  for (const JointGroupDefinition& group : configuration.joint_groups) {
    if (group.group_name.empty()) {
      return Status::Error(ErrorCode::kInvalidGroup,
                           "configuration contains an empty group_name");
    }
    if (!groups.insert(group.group_name).second) {
      return Status::Error(ErrorCode::kInvalidGroup,
                           "configuration contains duplicate group '" +
                               group.group_name + "'");
    }
    if (group.joint_names.empty()) {
      return Status::Error(ErrorCode::kInvalidGroup,
                           "joint group '" + group.group_name + "' is empty");
    }
    group_names = validateNames(group.joint_names, ErrorCode::kInvalidGroup,
                                "joint group '" + group.group_name + "'");
    if (!group_names) {
      return group_names;
    }
  }
  if (!impl_->sdk_api) {
    return Status::Error(ErrorCode::kInternalError,
                         "SDK API injection is null");
  }
  return impl_->sdk_api->configure(configuration);
}

Status SdkKinematics::load(const std::string& config_path) {
  if (config_path.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "config_path is empty");
  }
  if (!impl_->sdk_api) {
    return Status::Error(ErrorCode::kInternalError,
                         "SDK API injection is null");
  }
  return impl_->sdk_api->load(config_path);
}

Result<Pose> SdkKinematics::forwardKinematics(
    const ForwardKinematicsRequest& request) const {
  Status request_status = validateRequestNames(request.kinematics);
  if (!request_status) {
    return Result<Pose>::Failure(std::move(request_status));
  }
  Result<std::vector<std::string>> group =
      impl_->group(request.kinematics.group_name);
  if (!group) {
    return Result<Pose>::Failure(std::move(group.status));
  }
  Result<std::vector<double>> positions =
      reorderJointPositions(request.joint_state, *group.value);
  if (!positions) {
    return Result<Pose>::Failure(std::move(positions.status));
  }
  const std::vector<double> joints_deg = radiansToDegrees(*positions.value);

  Status frame_status = impl_->sdk_api->validateFrames(
      request.kinematics, *group.value, joints_deg);
  if (!frame_status) {
    return Result<Pose>::Failure(std::move(frame_status));
  }

  Result<SdkPose> sdk_result = impl_->sdk_api->forwardKinematics(
      request.kinematics, joints_deg);
  if (!sdk_result) {
    return Result<Pose>::Failure(std::move(sdk_result.status));
  }
  return toPublicPose(*sdk_result.value);
}

Result<InverseKinematicsResult> SdkKinematics::inverseKinematics(
    const InverseKinematicsRequest& request) const {
  Status request_status = validateRequestNames(request.kinematics);
  if (!request_status) {
    return Result<InverseKinematicsResult>::Failure(
        std::move(request_status));
  }
  Result<std::vector<std::string>> group =
      impl_->group(request.kinematics.group_name);
  if (!group) {
    return Result<InverseKinematicsResult>::Failure(std::move(group.status));
  }
  Result<SdkPose> target = toSdkPose(request.target_pose);
  if (!target) {
    return Result<InverseKinematicsResult>::Failure(std::move(target.status));
  }

  SdkInverseKinematicsRequest sdk_request;
  sdk_request.kinematics = request.kinematics;
  sdk_request.target_pose = *target.value;

  const JointState* selected_seed = nullptr;
  if (request.seed) {
    selected_seed = &*request.seed;
  } else if (request.current_state) {
    selected_seed = &*request.current_state;
  }
  if (selected_seed != nullptr) {
    Result<std::vector<double>> seed =
        reorderJointPositions(*selected_seed, *group.value);
    if (!seed) {
      return Result<InverseKinematicsResult>::Failure(std::move(seed.status));
    }
    sdk_request.q0_deg = radiansToDegrees(*seed.value);
    sdk_request.parameters.plan_only = true;
  }

  const IkParameters& parameters = request.parameters;
  SdkIkParameters& sdk_parameters = sdk_request.parameters;
  sdk_parameters.enable_joint_task = parameters.enable_joint_task;
  sdk_parameters.enable_arm_angle_constraint =
      parameters.enable_arm_angle_constraint;
  sdk_parameters.arm_angle_shoulder_frame =
      parameters.arm_angle_shoulder_frame;
  sdk_parameters.arm_angle_elbow_frame = parameters.arm_angle_elbow_frame;
  sdk_parameters.arm_angle_wrist_frame = parameters.arm_angle_wrist_frame;
  sdk_parameters.arm_angle_elbow_hint_axis =
      parameters.arm_angle_elbow_hint_axis;
  sdk_parameters.position_tolerance_mm =
      parameters.position_tolerance_m * kMillimetresPerMetre;
  sdk_parameters.orientation_tolerance_rad =
      parameters.orientation_tolerance_rad;
  sdk_parameters.trac_ik_timeout_sec = parameters.trac_ik_timeout_sec;
  sdk_parameters.placo_max_iterations = parameters.placo_max_iterations;
  sdk_parameters.retry_position_tolerance_mm =
      parameters.retry_position_tolerance_m * kMillimetresPerMetre;
  sdk_parameters.retry_orientation_tolerance_rad =
      parameters.retry_orientation_tolerance_rad;
  sdk_parameters.retry_interpolation_steps =
      parameters.retry_interpolation_steps;
  sdk_parameters.retry_min_position_distance_mm =
      parameters.retry_min_position_distance_m * kMillimetresPerMetre;
  sdk_parameters.retry_min_orientation_distance_rad =
      parameters.retry_min_orientation_distance_rad;
  sdk_parameters.placo_joint_task_weight =
      parameters.placo_joint_task_weight;

  if (parameters.redundancy_preference) {
    Result<std::vector<double>> preference = reorderJointPositions(
        *parameters.redundancy_preference, *group.value);
    if (!preference) {
      return Result<InverseKinematicsResult>::Failure(
          std::move(preference.status));
    }
    sdk_parameters.redundancy_preference_positions_deg =
        radiansToDegrees(*preference.value);
  }

  SdkInverseKinematicsResult sdk_result =
      impl_->sdk_api->inverseKinematics(sdk_request);
  if (!sdk_result.success) {
    return Result<InverseKinematicsResult>::Failure(Status::Error(
        ErrorCode::kSdkFailure, std::move(sdk_result.message)));
  }
  if (sdk_result.joint_names.size() != sdk_result.positions_deg.size()) {
    return Result<InverseKinematicsResult>::Failure(Status::Error(
        ErrorCode::kSdkFailure,
        "SDK IK returned joint_names/positions with different lengths"));
  }
  Status result_names = validateNames(sdk_result.joint_names,
                                      ErrorCode::kSdkFailure,
                                      "SDK IK result");
  if (!result_names) {
    return Result<InverseKinematicsResult>::Failure(std::move(result_names));
  }
  if (sdk_result.joint_names.size() != group.value->size()) {
    return Result<InverseKinematicsResult>::Failure(Status::Error(
        ErrorCode::kSdkFailure,
        "SDK IK result does not contain the complete requested joint group"));
  }
  for (const std::string& joint_name : sdk_result.joint_names) {
    if (std::find(group.value->begin(), group.value->end(), joint_name) ==
        group.value->end()) {
      return Result<InverseKinematicsResult>::Failure(Status::Error(
          ErrorCode::kSdkFailure,
          "SDK IK returned a joint outside the requested group: '" +
              joint_name + "'"));
    }
  }
  for (double position : sdk_result.positions_deg) {
    if (!finite(position)) {
      return Result<InverseKinematicsResult>::Failure(Status::Error(
          ErrorCode::kSdkFailure,
          "SDK IK returned a non-finite joint position"));
    }
  }
  if (!finite(sdk_result.position_error_mm) ||
      !finite(sdk_result.orientation_error_rad)) {
    return Result<InverseKinematicsResult>::Failure(Status::Error(
        ErrorCode::kSdkFailure, "SDK IK returned a non-finite error"));
  }

  InverseKinematicsResult result;
  result.solution.joint_names = std::move(sdk_result.joint_names);
  result.solution.positions_rad =
      degreesToRadians(sdk_result.positions_deg);
  result.position_error_m =
      sdk_result.position_error_mm / kMillimetresPerMetre;
  result.orientation_error_rad = sdk_result.orientation_error_rad;
  return Result<InverseKinematicsResult>::Success(std::move(result));
}

Status SdkKinematics::setToolTransform(const ToolTransform& transform) {
  if (transform.name.empty()) {
    return Status::Error(ErrorCode::kInvalidTool, "tool name is empty");
  }
  if (transform.parent_link.empty()) {
    return Status::Error(ErrorCode::kInvalidFrame,
                         "tool parent_link is empty");
  }
  if (!finite(transform.translation_m) || !finite(transform.rpy_rad)) {
    return Status::Error(ErrorCode::kInvalidTool,
                         "tool transform contains a non-finite value");
  }
  if (!impl_->sdk_api) {
    return Status::Error(ErrorCode::kInternalError,
                         "SDK API injection is null");
  }
  return impl_->sdk_api->setToolTransform(toSdkToolTransform(transform));
}

Result<ToolTransform> SdkKinematics::getToolTransform(
    const std::string& tool_name) const {
  if (tool_name.empty()) {
    return Result<ToolTransform>::Failure(
        Status::Error(ErrorCode::kInvalidTool, "tool name is empty"));
  }
  if (!impl_->sdk_api) {
    return Result<ToolTransform>::Failure(Status::Error(
        ErrorCode::kInternalError, "SDK API injection is null"));
  }
  Result<SdkToolTransform> sdk_result =
      impl_->sdk_api->getToolTransform(tool_name);
  if (!sdk_result) {
    return Result<ToolTransform>::Failure(std::move(sdk_result.status));
  }
  return Result<ToolTransform>::Success(
      toPublicToolTransform(*sdk_result.value));
}

Result<std::vector<ToolTransform>> SdkKinematics::getToolTransforms() const {
  if (!impl_->sdk_api) {
    return Result<std::vector<ToolTransform>>::Failure(Status::Error(
        ErrorCode::kInternalError, "SDK API injection is null"));
  }
  Result<std::vector<SdkToolTransform>> sdk_result =
      impl_->sdk_api->getToolTransforms();
  if (!sdk_result) {
    return Result<std::vector<ToolTransform>>::Failure(
        std::move(sdk_result.status));
  }
  std::vector<ToolTransform> result;
  result.reserve(sdk_result.value->size());
  for (const SdkToolTransform& transform : *sdk_result.value) {
    result.push_back(toPublicToolTransform(transform));
  }
  return Result<std::vector<ToolTransform>>::Success(std::move(result));
}

Result<std::vector<std::string>> SdkKinematics::jointGroup(
    const std::string& group_name) const {
  return impl_->group(group_name);
}

Status SdkKinematics::validateJointState(const std::string& group_name,
                                         const JointState& state) const {
  Result<std::vector<std::string>> group = impl_->group(group_name);
  if (!group) {
    return std::move(group.status);
  }
  Result<std::vector<double>> reordered =
      reorderJointPositions(state, *group.value);
  return reordered ? Status::Ok() : std::move(reordered.status);
}

}  // namespace humanoid_motion_server::kinematics
