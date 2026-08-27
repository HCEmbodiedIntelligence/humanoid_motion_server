#pragma once

#include <memory>
#include <string>
#include <vector>

#include "humanoid_motion_server/kinematics/types.hpp"

namespace humanoid_motion_server::kinematics {

class SdkApi;

// ROS-independent injection boundary used by motion_core and its tests.
class IKinematics {
 public:
  virtual ~IKinematics() = default;

  // Configures an RKD model and named groups directly through motion_control::Rkd.
  virtual Status configure(const Configuration& configuration) = 0;

  // Loads the SDK's full configuration through InitializeSystemFromFile.
  virtual Status load(const std::string& config_path) = 0;

  virtual Result<Pose> forwardKinematics(
      const ForwardKinematicsRequest& request) const = 0;
  virtual Result<InverseKinematicsResult> inverseKinematics(
      const InverseKinematicsRequest& request) const = 0;

  virtual Status setToolTransform(const ToolTransform& transform) = 0;
  virtual Result<ToolTransform> getToolTransform(
      const std::string& tool_name) const = 0;
  virtual Result<std::vector<ToolTransform>> getToolTransforms() const = 0;

  virtual Result<std::vector<std::string>> jointGroup(
      const std::string& group_name) const = 0;
  virtual Status validateJointState(const std::string& group_name,
                                    const JointState& state) const = 0;
};

// Thin SI/name-mapping adapter. The injected SdkApi is implemented by the real
// robo_manip SDK in the sdk target and can be replaced by a test double.
class SdkKinematics final : public IKinematics {
 public:
  explicit SdkKinematics(std::shared_ptr<SdkApi> sdk_api);
  ~SdkKinematics() override;

  SdkKinematics(const SdkKinematics&) = delete;
  SdkKinematics& operator=(const SdkKinematics&) = delete;
  SdkKinematics(SdkKinematics&&) noexcept;
  SdkKinematics& operator=(SdkKinematics&&) noexcept;

  // Defined in the real SDK integration target. It wires this adapter to
  // robo_manip::kinematics::Kinematics and motion_control::Rkd.
  static std::shared_ptr<SdkKinematics> create();

  Status configure(const Configuration& configuration) override;
  Status load(const std::string& config_path) override;
  Result<Pose> forwardKinematics(
      const ForwardKinematicsRequest& request) const override;
  Result<InverseKinematicsResult> inverseKinematics(
      const InverseKinematicsRequest& request) const override;
  Status setToolTransform(const ToolTransform& transform) override;
  Result<ToolTransform> getToolTransform(
      const std::string& tool_name) const override;
  Result<std::vector<ToolTransform>> getToolTransforms() const override;
  Result<std::vector<std::string>> jointGroup(
      const std::string& group_name) const override;
  Status validateJointState(const std::string& group_name,
                            const JointState& state) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace humanoid_motion_server::kinematics
