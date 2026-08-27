#pragma once

#include <memory>
#include <string>
#include <vector>

#include "humanoid_motion_server/kinematics/types.hpp"

namespace humanoid_motion_server::kinematics {

// Internal SDK-unit DTOs are public only to support deterministic adapter
// tests. Values named *_deg and *_mm deliberately make the unit boundary
// impossible to overlook.
struct SdkPose {
  Vector3 position_mm;
  Quaternion orientation;
  Vector3 euler_zyx_deg;
};

struct SdkToolTransform {
  std::string name;
  std::string parent_link;
  Vector3 translation_mm;
  Vector3 rpy_rad;
};

struct SdkIkParameters {
  bool plan_only{false};
  std::vector<double> planning_start_joint_deg;
  bool enable_joint_task{false};
  std::vector<double> redundancy_preference_positions_deg;
  bool enable_arm_angle_constraint{false};
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;
  Vector3 arm_angle_elbow_hint_axis{0.0, 0.0, 1.0};
  double position_tolerance_mm{0.5};
  double orientation_tolerance_rad{1e-3};
  double trac_ik_timeout_sec{0.02};
  int placo_max_iterations{1000};
  double retry_position_tolerance_mm{1.0};
  double retry_orientation_tolerance_rad{1e-2};
  int retry_interpolation_steps{80};
  double retry_min_position_distance_mm{5.0};
  double retry_min_orientation_distance_rad{0.025};
  double placo_joint_task_weight{1e-3};
};

struct SdkInverseKinematicsRequest {
  KinematicsRequest kinematics;
  SdkPose target_pose;
  std::vector<double> q0_deg;
  SdkIkParameters parameters;
};

struct SdkInverseKinematicsResult {
  bool success{false};
  std::string message;
  std::vector<std::string> joint_names;
  std::vector<double> positions_deg;
  double position_error_mm{0.0};
  double orientation_error_rad{0.0};
};

class SdkApi {
 public:
  virtual ~SdkApi() = default;

  virtual Status configure(const Configuration& configuration) = 0;
  virtual Status load(const std::string& config_path) = 0;
  virtual Result<std::vector<std::string>> jointGroup(
      const std::string& group_name) const = 0;

  // RKD's structured Jacobian result is used only as a model/frame validity
  // probe before the SDK FK call. No Jacobian is exposed or reimplemented.
  virtual Status validateFrames(const KinematicsRequest& request,
                                const std::vector<std::string>& joint_names,
                                const std::vector<double>& joints_deg) const = 0;
  virtual Result<SdkPose> forwardKinematics(
      const KinematicsRequest& request,
      const std::vector<double>& joints_deg) const = 0;
  virtual SdkInverseKinematicsResult inverseKinematics(
      const SdkInverseKinematicsRequest& request) const = 0;

  virtual Status setToolTransform(const SdkToolTransform& transform) = 0;
  virtual Result<SdkToolTransform> getToolTransform(
      const std::string& tool_name) const = 0;
  virtual Result<std::vector<SdkToolTransform>> getToolTransforms() const = 0;
};

std::shared_ptr<SdkApi> makeRoboManipSdkApi();

}  // namespace humanoid_motion_server::kinematics
