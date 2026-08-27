#ifndef SDK_UNITS_HPP_
#define SDK_UNITS_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "humanoid_motion_server/motion/types.hpp"
#include "motion_control/types/geometry.hpp"
#include "motion_control/types/task.hpp"

namespace humanoid_motion_server::motion::sdk_units
{

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kRadToDeg = 180.0 / kPi;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kMToMm = 1000.0;

inline double toDegrees(const double radians) {return radians * kRadToDeg;}
inline double toRadians(const double degrees) {return degrees * kDegToRad;}

inline std::vector<double> scaled(
  const std::vector<double> & values, const double factor)
{
  std::vector<double> result;
  result.reserve(values.size());
  std::transform(
    values.begin(), values.end(), std::back_inserter(result),
    [factor](const double value) {return value * factor;});
  return result;
}

inline motion_control::types::Pose toSdkPose(const Pose & input)
{
  motion_control::types::Pose output;
  output.position.x = input.position_m[0] * kMToMm;
  output.position.y = input.position_m[1] * kMToMm;
  output.position.z = input.position_m[2] * kMToMm;

  Eigen::Quaterniond quaternion(
    input.orientation_xyzw[3], input.orientation_xyzw[0],
    input.orientation_xyzw[1], input.orientation_xyzw[2]);
  quaternion.normalize();
  output.orientation.x = quaternion.x();
  output.orientation.y = quaternion.y();
  output.orientation.z = quaternion.z();
  output.orientation.w = quaternion.w();
  const Eigen::Vector3d yaw_pitch_roll =
    quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  output.euler_zyx_deg.x = toDegrees(yaw_pitch_roll[2]);
  output.euler_zyx_deg.y = toDegrees(yaw_pitch_roll[1]);
  output.euler_zyx_deg.z = toDegrees(yaw_pitch_roll[0]);
  return output;
}

inline motion_control::types::MotionLimits toSdkLimits(
  const MotionLimits & input)
{
  motion_control::types::MotionLimits output;
  output.joint_max_velocity = scaled(input.joint_max_velocity_rad_s, kRadToDeg);
  output.joint_max_acceleration = scaled(
    input.joint_max_acceleration_rad_s2, kRadToDeg);
  output.joint_max_jerk = scaled(input.joint_max_jerk_rad_s3, kRadToDeg);
  output.cartesian_max_linear_velocity =
    input.cartesian_max_linear_velocity_m_s * kMToMm;
  output.cartesian_max_linear_acceleration =
    input.cartesian_max_linear_acceleration_m_s2 * kMToMm;
  output.cartesian_max_linear_jerk =
    input.cartesian_max_linear_jerk_m_s3 * kMToMm;
  output.cartesian_max_angular_velocity = input.cartesian_max_angular_velocity_rad_s;
  output.cartesian_max_angular_acceleration =
    input.cartesian_max_angular_acceleration_rad_s2;
  output.cartesian_max_angular_jerk = input.cartesian_max_angular_jerk_rad_s3;
  return output;
}

inline JointCommand fromSdkJointCommand(
  const std::string & group_name,
  const std::vector<std::string> & names,
  const std::vector<double> & positions_deg,
  const std::vector<double> & velocities_deg_s)
{
  JointCommand output;
  output.group_name = group_name;
  output.joint_names = names;
  output.positions_rad = scaled(positions_deg, kDegToRad);
  output.velocities_rad_s = scaled(velocities_deg_s, kDegToRad);
  return output;
}

}  // namespace humanoid_motion_server::motion::sdk_units

#endif  // SDK_UNITS_HPP_
