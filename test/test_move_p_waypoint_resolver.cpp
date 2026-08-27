#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "move_p_waypoint_resolver.hpp"

namespace hmc = humanoid_motion_server::motion;
namespace hkd = humanoid_motion_server::kinematics;

namespace
{

class FakeKinematics final : public hkd::IKinematics
{
public:
  hkd::Status configure(const hkd::Configuration &) override {return hkd::Status::Ok();}
  hkd::Status load(const std::string &) override {return hkd::Status::Ok();}

  hkd::Result<hkd::Pose> forwardKinematics(
    const hkd::ForwardKinematicsRequest &) const override
  {
    return hkd::Result<hkd::Pose>::Success({});
  }

  hkd::Result<hkd::InverseKinematicsResult> inverseKinematics(
    const hkd::InverseKinematicsRequest & request) const override
  {
    requests.push_back(request);
    if (!failure.ok()) {
      return hkd::Result<hkd::InverseKinematicsResult>::Failure(failure);
    }
    return hkd::Result<hkd::InverseKinematicsResult>::Success(
      solutions.at(next_solution++));
  }

  hkd::Status setToolTransform(const hkd::ToolTransform &) override
  {
    return hkd::Status::Ok();
  }

  hkd::Result<hkd::ToolTransform> getToolTransform(
    const std::string &) const override
  {
    return hkd::Result<hkd::ToolTransform>::Failure(
      hkd::Status::Error(hkd::ErrorCode::kInvalidTool, "not used"));
  }

  hkd::Result<std::vector<hkd::ToolTransform>> getToolTransforms() const override
  {
    return hkd::Result<std::vector<hkd::ToolTransform>>::Success({});
  }

  hkd::Result<std::vector<std::string>> jointGroup(
    const std::string &) const override
  {
    return hkd::Result<std::vector<std::string>>::Success({"j1", "j2"});
  }

  hkd::Status validateJointState(
    const std::string &, const hkd::JointState &) const override
  {
    return hkd::Status::Ok();
  }

  mutable std::vector<hkd::InverseKinematicsRequest> requests;
  mutable std::size_t next_solution{0};
  std::vector<hkd::InverseKinematicsResult> solutions;
  hkd::Status failure;
};

TEST(MovePWaypointResolver, CallsHumanoidKinematicsForEveryWaypointAndChainsSeed)
{
  FakeKinematics kinematics;
  hkd::InverseKinematicsResult first;
  first.solution.joint_names = {"j2", "j1"};
  first.solution.positions_rad = {2.0, 1.0};
  hkd::InverseKinematicsResult second;
  second.solution.joint_names = {"j1", "j2"};
  second.solution.positions_rad = {3.0, 4.0};
  kinematics.solutions = {first, second};

  hmc::MovePRequest request;
  request.request_id = "move-p";
  request.group_name = "arm";
  request.base_link = "base";
  request.link_name = "tool";
  hmc::Pose first_pose;
  first_pose.position_m[0] = 0.1;
  hmc::Pose second_pose;
  second_pose.position_m[0] = 0.2;
  request.waypoints = {first_pose, second_pose};
  hmc::JointTarget seed{{"j1", "j2"}, {0.0, 0.0}, {}, {}};

  const auto result = hmc::detail::resolveMovePWaypoints(
    request, seed, {"j1", "j2"}, kinematics);
  ASSERT_TRUE(result.status.ok());
  ASSERT_EQ(kinematics.requests.size(), 2U);
  EXPECT_DOUBLE_EQ(kinematics.requests[0].target_pose.position_m.x, 0.1);
  ASSERT_TRUE(kinematics.requests[1].seed.has_value());
  EXPECT_EQ(
    kinematics.requests[1].seed->joint_names,
    std::vector<std::string>({"j1", "j2"}));
  EXPECT_EQ(
    kinematics.requests[1].seed->positions_rad,
    std::vector<double>({1.0, 2.0}));
  ASSERT_EQ(result.joint_waypoints.size(), 2U);
  EXPECT_EQ(
    result.joint_waypoints[0].positions_rad,
    std::vector<double>({1.0, 2.0}));
  EXPECT_EQ(
    result.joint_waypoints[1].positions_rad,
    std::vector<double>({3.0, 4.0}));
}

TEST(MovePWaypointResolver, PreservesHumanoidKinematicsStructuredFailure)
{
  FakeKinematics kinematics;
  kinematics.failure = hkd::Status::Error(
    hkd::ErrorCode::kSdkFailure, "native IK failed");
  hmc::MovePRequest request;
  request.group_name = "arm";
  request.base_link = "base";
  request.link_name = "tool";
  request.waypoints = {hmc::Pose{}};
  hmc::JointTarget seed{{"j1", "j2"}, {0.0, 0.0}, {}, {}};

  const auto result = hmc::detail::resolveMovePWaypoints(
    request, seed, {"j1", "j2"}, kinematics);
  EXPECT_EQ(result.status.code, hmc::StatusCode::SDK_ERROR);
  EXPECT_EQ(result.status.message, "native IK failed");
  EXPECT_EQ(
    result.status.sdk_api,
    "humanoid_motion_server::kinematics::IKinematics::inverseKinematics");
  EXPECT_EQ(
    result.status.sdk_code,
    static_cast<std::int64_t>(hkd::ErrorCode::kSdkFailure));
}

}  // namespace
