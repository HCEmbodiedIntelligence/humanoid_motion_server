#ifndef HUMANOID_MOTION_SERVER__SDK_MOTION_BACKEND_HPP_
#define HUMANOID_MOTION_SERVER__SDK_MOTION_BACKEND_HPP_

#include <memory>
#include <string>
#include <vector>

#include "humanoid_motion_server/motion/kinematics_interface.hpp"
#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion
{

class ISdkMotionBackend
{
public:
  virtual ~ISdkMotionBackend() = default;

  virtual MotionStatus startSession(
    const std::string & session_id, const SessionRequest & request,
    const JointFeedback & feedback, double period_sec) = 0;
  virtual BackendTick tickSession(
    const std::string & session_id, const DynamicTarget * dynamic_target,
    double period_sec) = 0;

  /// Must return success when called repeatedly for the same session.
  virtual MotionStatus stopSession(const std::string & session_id) = 0;

  /// Resets the independent final RTC for group_name from real feedback.
  virtual MotionStatus resetFinalJointTarget(
    const std::string & group_name, const JointFeedback & feedback) = 0;

  /// The only backend operation allowed to create a publishable command.
  virtual BackendTick updateFinalJointTarget(
    const std::string & group_name, const JointCommand & candidate,
    double period_sec) = 0;

  virtual ForwardKinematicsResult forwardKinematics(
    const ForwardKinematicsRequest & request) = 0;
};

using SdkMotionBackendPtr = std::shared_ptr<ISdkMotionBackend>;

/// Concrete adapter around robo_manip::tasks::MoveJoint, MoveLine and the
/// independent motion_control::Rtc instances created by MotionContextFactory.
/// SDK types are hidden behind the pimpl so every public value remains SI.
class SdkMotionBackend final : public ISdkMotionBackend
{
public:
  ~SdkMotionBackend() override;

  MotionStatus startSession(
    const std::string & session_id, const SessionRequest & request,
    const JointFeedback & feedback, double period_sec) override;
  BackendTick tickSession(
    const std::string & session_id, const DynamicTarget * dynamic_target,
    double period_sec) override;
  MotionStatus stopSession(const std::string & session_id) override;
  MotionStatus resetFinalJointTarget(
    const std::string & group_name, const JointFeedback & feedback) override;
  BackendTick updateFinalJointTarget(
    const std::string & group_name, const JointCommand & candidate,
    double period_sec) override;
  ForwardKinematicsResult forwardKinematics(
    const ForwardKinematicsRequest & request) override;

private:
  class Impl;
  explicit SdkMotionBackend(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;

  friend class MotionContextFactory;
};

struct MotionContextFactoryOptions
{
  /// Limits are required because the current SDK's Rkd public ABI exposes group
  /// order but not model position limits. Values remain SI at this API.
  std::vector<JointGroupModel> joint_groups;
  /// Production passes the SDK kinematics adapter public adapter here. If omitted,
  /// the factory creates and loads humanoid_motion_server::kinematics::SdkKinematics.
  HumanoidKinematicsPtr kinematics;
};

struct MotionContextFactoryResult
{
  MotionStatus status;
  SdkMotionBackendPtr backend;
  std::vector<JointGroupModel> joint_groups;
};

class MotionContextFactory
{
public:
  /// Creates the original SDK MotionContext from its YAML, shares only RKD, and
  /// allocates independent mutable RTC state for every group/session.
  static MotionContextFactoryResult createFromSdkYaml(
    const std::string & sdk_yaml_path,
    const MotionContextFactoryOptions & options);
};

}  // namespace humanoid_motion_server::motion

#endif  // HUMANOID_MOTION_SERVER__SDK_MOTION_BACKEND_HPP_
