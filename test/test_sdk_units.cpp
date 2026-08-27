#include <gtest/gtest.h>

#include <cmath>

#include "sdk_units.hpp"

namespace hmc = humanoid_motion_server::motion;
namespace units = humanoid_motion_server::motion::sdk_units;

namespace
{

TEST(SdkUnits, JointAnglesAndRatesRoundTripAcrossRadiansAndDegrees)
{
  const std::vector<double> radians{0.0, units::kPi / 2.0, -units::kPi};
  const auto degrees = units::scaled(radians, units::kRadToDeg);
  EXPECT_DOUBLE_EQ(degrees[0], 0.0);
  EXPECT_NEAR(degrees[1], 90.0, 1e-12);
  EXPECT_NEAR(degrees[2], -180.0, 1e-12);

  const auto command = units::fromSdkJointCommand(
    "arm", {"j1", "j2", "j3"}, degrees, {180.0, 90.0, 0.0});
  EXPECT_NEAR(command.positions_rad[1], units::kPi / 2.0, 1e-12);
  EXPECT_NEAR(command.velocities_rad_s[0], units::kPi, 1e-12);
}

TEST(SdkUnits, CartesianPoseConvertsMetresAndQuaternionToSdkMillimetresAndDegrees)
{
  hmc::Pose pose;
  pose.position_m = {0.123, -0.004, 1.0};
  const auto half_yaw = units::kPi / 4.0;
  pose.orientation_xyzw = {0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)};

  const auto sdk = units::toSdkPose(pose);
  EXPECT_NEAR(sdk.position.x, 123.0, 1e-12);
  EXPECT_NEAR(sdk.position.y, -4.0, 1e-12);
  EXPECT_NEAR(sdk.position.z, 1000.0, 1e-12);
  EXPECT_NEAR(sdk.euler_zyx_deg.x, 0.0, 1e-12);
  EXPECT_NEAR(sdk.euler_zyx_deg.y, 0.0, 1e-12);
  EXPECT_NEAR(sdk.euler_zyx_deg.z, 90.0, 1e-12);
  EXPECT_NEAR(sdk.orientation.z, pose.orientation_xyzw[2], 1e-12);
}

TEST(SdkUnits, MotionLimitConversionCoversEveryLinearAndAngularBoundary)
{
  hmc::MotionLimits limits;
  limits.joint_max_velocity_rad_s = {units::kPi};
  limits.joint_max_acceleration_rad_s2 = {units::kPi / 2.0};
  limits.joint_max_jerk_rad_s3 = {units::kPi / 4.0};
  limits.cartesian_max_linear_velocity_m_s = 0.2;
  limits.cartesian_max_linear_acceleration_m_s2 = 0.3;
  limits.cartesian_max_linear_jerk_m_s3 = 0.4;
  limits.cartesian_max_angular_velocity_rad_s = 0.5;
  limits.cartesian_max_angular_acceleration_rad_s2 = 0.6;
  limits.cartesian_max_angular_jerk_rad_s3 = 0.7;

  const auto sdk = units::toSdkLimits(limits);
  EXPECT_NEAR(sdk.joint_max_velocity[0], 180.0, 1e-12);
  EXPECT_NEAR(sdk.joint_max_acceleration[0], 90.0, 1e-12);
  EXPECT_NEAR(sdk.joint_max_jerk[0], 45.0, 1e-12);
  EXPECT_NEAR(sdk.cartesian_max_linear_velocity, 200.0, 1e-12);
  EXPECT_NEAR(sdk.cartesian_max_linear_acceleration, 300.0, 1e-12);
  EXPECT_NEAR(sdk.cartesian_max_linear_jerk, 400.0, 1e-12);
  EXPECT_DOUBLE_EQ(sdk.cartesian_max_angular_velocity, 0.5);
  EXPECT_DOUBLE_EQ(sdk.cartesian_max_angular_acceleration, 0.6);
  EXPECT_DOUBLE_EQ(sdk.cartesian_max_angular_jerk, 0.7);
}

}  // namespace
