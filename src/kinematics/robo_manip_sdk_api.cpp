#include "humanoid_motion_server/kinematics/sdk_api.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "motion_control/rkd.hpp"
#include "robo_manip/core/motion_context.hpp"
#include "robo_manip/core/system_init.hpp"
#include "robo_manip/kinematics/kinematics.hpp"

namespace humanoid_motion_server::kinematics {
namespace {

motion_control::types::RobotState makeRobotState(
    const std::string& group_name,
    const std::vector<double>& joints_deg) {
  motion_control::types::RobotState state;
  motion_control::types::JointState& group = state.addJointGroup(group_name);
  group.positions = joints_deg;
  return state;
}

SdkPose fromSdkPose(const motion_control::types::Pose& pose) {
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.orientation.x, pose.orientation.y, pose.orientation.z,
           pose.orientation.w},
          {pose.euler_zyx_deg.x, pose.euler_zyx_deg.y,
           pose.euler_zyx_deg.z}};
}

motion_control::types::Pose toSdkPose(const SdkPose& pose) {
  motion_control::types::Pose result;
  result.position = {pose.position_mm.x, pose.position_mm.y,
                     pose.position_mm.z};
  result.orientation = {pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w};
  result.euler_zyx_deg = {pose.euler_zyx_deg.x, pose.euler_zyx_deg.y,
                          pose.euler_zyx_deg.z};
  return result;
}

motion_control::ToolTransform toSdkTool(const SdkToolTransform& transform) {
  motion_control::ToolTransform result;
  result.name = transform.name;
  result.parent_link = transform.parent_link;
  result.x = transform.translation_mm.x;
  result.y = transform.translation_mm.y;
  result.z = transform.translation_mm.z;
  result.rx = transform.rpy_rad.x;
  result.ry = transform.rpy_rad.y;
  result.rz = transform.rpy_rad.z;
  return result;
}

SdkToolTransform fromSdkTool(const motion_control::ToolTransform& transform) {
  return {transform.name,
          transform.parent_link,
          {transform.x, transform.y, transform.z},
          {transform.rx, transform.ry, transform.rz}};
}

robo_manip::kinematics::KinematicsRequest toSdkRequest(
    const KinematicsRequest& request) {
  return {request.group_name, request.base_link, request.link_name};
}

Status exceptionStatus(ErrorCode code, const std::exception& error) {
  return Status::Error(code, error.what());
}

class RoboManipSdkApi final : public SdkApi {
 public:
  Status configure(const Configuration& configuration) override {
    try {
      auto new_rkd = std::make_shared<motion_control::Rkd>();
      if (!new_rkd->loadModel(configuration.model_path)) {
        return Status::Error(
            ErrorCode::kSdkModelLoadFailed,
            "motion_control::Rkd::loadModel failed for '" +
                configuration.model_path + "'");
      }
      for (const JointGroupDefinition& group : configuration.joint_groups) {
        new_rkd->registerJointGroup(group.group_name, group.joint_names);
      }

      robo_manip::core::MotionContext new_context;
      new_context.rkd = std::move(new_rkd);
      auto new_kinematics =
          std::make_shared<robo_manip::kinematics::Kinematics>(new_context);
      new_context.kinematics = new_kinematics;

      context_ = std::move(new_context);
      rkd_ = context_.rkd;
      kinematics_ = std::move(new_kinematics);
      return Status::Ok();
    } catch (const std::exception& error) {
      return exceptionStatus(ErrorCode::kSdkModelLoadFailed, error);
    }
  }

  Status load(const std::string& config_path) override {
    try {
      robo_manip::core::SystemInitResult initialized =
          robo_manip::core::InitializeSystemFromFile(config_path);
      if (!initialized.success) {
        return Status::Error(ErrorCode::kSdkInitializationFailed,
                             std::move(initialized.message));
      }
      if (!initialized.context.rkd) {
        return Status::Error(
            ErrorCode::kSdkInitializationFailed,
            "InitializeSystemFromFile succeeded without an RKD instance");
      }

      context_ = std::move(initialized.context);
      rkd_ = context_.rkd;
      if (context_.kinematics) {
        kinematics_ = context_.kinematics;
      } else {
        kinematics_ =
            std::make_shared<robo_manip::kinematics::Kinematics>(context_);
        context_.kinematics = kinematics_;
      }
      return Status::Ok();
    } catch (const std::exception& error) {
      return exceptionStatus(ErrorCode::kSdkInitializationFailed, error);
    }
  }

  Result<std::vector<std::string>> jointGroup(
      const std::string& group_name) const override {
    if (!rkd_) {
      return Result<std::vector<std::string>>::Failure(Status::Error(
          ErrorCode::kNotConfigured, "RKD is not configured"));
    }
    try {
      return Result<std::vector<std::string>>::Success(
          rkd_->jointGroup(group_name));
    } catch (const std::exception& error) {
      return Result<std::vector<std::string>>::Failure(
          exceptionStatus(ErrorCode::kSdkFailure, error));
    }
  }

  Status validateFrames(const KinematicsRequest& request,
                        const std::vector<std::string>& joint_names,
                        const std::vector<double>& joints_deg) const override {
    if (!rkd_) {
      return Status::Error(ErrorCode::kNotConfigured,
                           "RKD is not configured");
    }
    try {
      const motion_control::JacobianResult probe = rkd_->frameJacobian(
          makeRobotState(request.group_name, joints_deg), request.base_link,
          request.link_name, request.base_link, joint_names);
      if (!probe.success) {
        return Status::Error(ErrorCode::kInvalidFrame, probe.message);
      }
      return Status::Ok();
    } catch (const std::exception& error) {
      return exceptionStatus(ErrorCode::kInvalidFrame, error);
    }
  }

  Result<SdkPose> forwardKinematics(
      const KinematicsRequest& request,
      const std::vector<double>& joints_deg) const override {
    if (!kinematics_) {
      return Result<SdkPose>::Failure(Status::Error(
          ErrorCode::kNotConfigured, "SDK kinematics is not configured"));
    }
    try {
      return Result<SdkPose>::Success(fromSdkPose(
          kinematics_->forwardKinematics(toSdkRequest(request), joints_deg)));
    } catch (const std::exception& error) {
      return Result<SdkPose>::Failure(
          exceptionStatus(ErrorCode::kSdkFailure, error));
    }
  }

  SdkInverseKinematicsResult inverseKinematics(
      const SdkInverseKinematicsRequest& request) const override {
    if (!kinematics_) {
      SdkInverseKinematicsResult result;
      result.success = false;
      result.message = "SDK kinematics is not configured";
      return result;
    }
    try {
      robo_manip::kinematics::InverseKinematicsRequest sdk_request;
      sdk_request.group_name = request.kinematics.group_name;
      sdk_request.base_link = request.kinematics.base_link;
      sdk_request.link_name = request.kinematics.link_name;
      sdk_request.target_pose = toSdkPose(request.target_pose);
      sdk_request.q0_deg = request.q0_deg;

      const SdkIkParameters& input = request.parameters;
      robo_manip::kinematics::IkParameters& output = sdk_request.parameters;
      output.plan_only = input.plan_only;
      output.planning_start_joint = input.planning_start_joint_deg;
      output.enable_joint_task = input.enable_joint_task;
      output.redundancy_preference_positions =
          input.redundancy_preference_positions_deg;
      output.enable_arm_angle_constraint =
          input.enable_arm_angle_constraint;
      output.arm_angle_shoulder_frame = input.arm_angle_shoulder_frame;
      output.arm_angle_elbow_frame = input.arm_angle_elbow_frame;
      output.arm_angle_wrist_frame = input.arm_angle_wrist_frame;
      output.arm_angle_elbow_hint_axis =
          Eigen::Vector3d(input.arm_angle_elbow_hint_axis.x,
                          input.arm_angle_elbow_hint_axis.y,
                          input.arm_angle_elbow_hint_axis.z);
      output.position_tolerance = input.position_tolerance_mm;
      output.orientation_tolerance = input.orientation_tolerance_rad;
      output.trac_ik_timeout = input.trac_ik_timeout_sec;
      output.placo_max_iterations = input.placo_max_iterations;
      output.retry_position_tolerance =
          input.retry_position_tolerance_mm;
      output.retry_orientation_tolerance =
          input.retry_orientation_tolerance_rad;
      output.retry_interpolation_steps = input.retry_interpolation_steps;
      output.retry_min_position_distance =
          input.retry_min_position_distance_mm;
      output.retry_min_orientation_distance =
          input.retry_min_orientation_distance_rad;
      output.placo_joint_task_weight = input.placo_joint_task_weight;

      robo_manip::kinematics::IkResult result =
          kinematics_->inverseKinematics(sdk_request);
      return {result.success,
              std::move(result.message),
              std::move(result.joint_names),
              std::move(result.positions),
              result.position_error,
              result.orientation_error};
    } catch (const std::exception& error) {
      SdkInverseKinematicsResult result;
      result.success = false;
      result.message = error.what();
      return result;
    }
  }

  Status setToolTransform(const SdkToolTransform& transform) override {
    if (!kinematics_) {
      return Status::Error(ErrorCode::kNotConfigured,
                           "SDK kinematics is not configured");
    }
    try {
      kinematics_->setToolTransform(toSdkTool(transform));
      motion_control::ToolTransform confirmed;
      if (!kinematics_->getToolTransform(transform.name, &confirmed)) {
        return Status::Error(
            ErrorCode::kInvalidTool,
            "SDK rejected tool transform '" + transform.name + "'");
      }
      return Status::Ok();
    } catch (const std::exception& error) {
      return exceptionStatus(ErrorCode::kInvalidTool, error);
    }
  }

  Result<SdkToolTransform> getToolTransform(
      const std::string& tool_name) const override {
    if (!kinematics_) {
      return Result<SdkToolTransform>::Failure(Status::Error(
          ErrorCode::kNotConfigured, "SDK kinematics is not configured"));
    }
    try {
      motion_control::ToolTransform transform;
      if (!kinematics_->getToolTransform(tool_name, &transform)) {
        return Result<SdkToolTransform>::Failure(Status::Error(
            ErrorCode::kInvalidTool,
            "SDK tool transform '" + tool_name + "' was not found"));
      }
      return Result<SdkToolTransform>::Success(fromSdkTool(transform));
    } catch (const std::exception& error) {
      return Result<SdkToolTransform>::Failure(
          exceptionStatus(ErrorCode::kInvalidTool, error));
    }
  }

  Result<std::vector<SdkToolTransform>> getToolTransforms() const override {
    if (!kinematics_) {
      return Result<std::vector<SdkToolTransform>>::Failure(Status::Error(
          ErrorCode::kNotConfigured, "SDK kinematics is not configured"));
    }
    try {
      std::vector<SdkToolTransform> result;
      for (const motion_control::ToolTransform& transform :
           kinematics_->getToolTransforms()) {
        result.push_back(fromSdkTool(transform));
      }
      return Result<std::vector<SdkToolTransform>>::Success(
          std::move(result));
    } catch (const std::exception& error) {
      return Result<std::vector<SdkToolTransform>>::Failure(
          exceptionStatus(ErrorCode::kSdkFailure, error));
    }
  }

 private:
  robo_manip::core::MotionContext context_;
  std::shared_ptr<motion_control::Rkd> rkd_;
  std::shared_ptr<robo_manip::kinematics::Kinematics> kinematics_;
};

}  // namespace

std::shared_ptr<SdkApi> makeRoboManipSdkApi() {
  return std::make_shared<RoboManipSdkApi>();
}

std::shared_ptr<SdkKinematics> SdkKinematics::create() {
  return std::make_shared<SdkKinematics>(makeRoboManipSdkApi());
}

}  // namespace humanoid_motion_server::kinematics
