#ifndef HUMANOID_MOTION_SERVER__KINEMATICS_INTERFACE_HPP_
#define HUMANOID_MOTION_SERVER__KINEMATICS_INTERFACE_HPP_

#include <memory>
#include <string>

#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion
{

struct InverseKinematicsRequest
{
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  Pose target;
  JointTarget seed;
};

struct InverseKinematicsResult
{
  MotionStatus status;
  JointTarget solution;
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

struct ForwardKinematicsRequest
{
  std::string group_name;
  std::string base_link{"base_link"};
  std::string link_name;
  JointTarget joints;
};

struct ForwardKinematicsResult
{
  MotionStatus status;
  Pose pose;
};

/// The actual SDK kinematics adapter public injection boundary. Core owns no
/// IK/FK implementation and converts only between its equivalent SI DTOs.
using HumanoidKinematicsPtr = std::shared_ptr<humanoid_motion_server::kinematics::IKinematics>;

}  // namespace humanoid_motion_server::motion

#endif  // HUMANOID_MOTION_SERVER__KINEMATICS_INTERFACE_HPP_
