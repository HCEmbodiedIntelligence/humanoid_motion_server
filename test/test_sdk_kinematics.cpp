#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "humanoid_motion_server/kinematics/sdk_api.hpp"

namespace humanoid_motion_server::kinematics {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;

class FakeSdkApi final : public SdkApi {
 public:
  Status configure(const Configuration& configuration) override {
    configured_with = configuration;
    return configure_status;
  }

  Status load(const std::string& config_path) override {
    loaded_path = config_path;
    return load_status;
  }

  Result<std::vector<std::string>> jointGroup(
      const std::string& group_name) const override {
    const auto group = groups.find(group_name);
    if (group == groups.end()) {
      return Result<std::vector<std::string>>::Success({});
    }
    return Result<std::vector<std::string>>::Success(group->second);
  }

  Status validateFrames(const KinematicsRequest& request,
                        const std::vector<std::string>& joint_names,
                        const std::vector<double>& joints_deg) const override {
    frame_request = request;
    frame_joint_names = joint_names;
    frame_joints_deg = joints_deg;
    ++frame_call_count;
    return frame_status;
  }

  Result<SdkPose> forwardKinematics(
      const KinematicsRequest& request,
      const std::vector<double>& joints_deg) const override {
    fk_request = request;
    fk_joints_deg = joints_deg;
    ++fk_call_count;
    if (!fk_status) {
      return Result<SdkPose>::Failure(fk_status);
    }
    return Result<SdkPose>::Success(fk_pose);
  }

  SdkInverseKinematicsResult inverseKinematics(
      const SdkInverseKinematicsRequest& request) const override {
    ik_request = request;
    ++ik_call_count;
    return ik_result;
  }

  Status setToolTransform(const SdkToolTransform& transform) override {
    set_tool = transform;
    if (set_tool_status) {
      tools[transform.name] = transform;
    }
    return set_tool_status;
  }

  Result<SdkToolTransform> getToolTransform(
      const std::string& tool_name) const override {
    const auto tool = tools.find(tool_name);
    if (tool == tools.end()) {
      return Result<SdkToolTransform>::Failure(Status::Error(
          ErrorCode::kInvalidTool, "fake tool missing"));
    }
    return Result<SdkToolTransform>::Success(tool->second);
  }

  Result<std::vector<SdkToolTransform>> getToolTransforms() const override {
    std::vector<SdkToolTransform> result;
    for (const auto& entry : tools) {
      result.push_back(entry.second);
    }
    return Result<std::vector<SdkToolTransform>>::Success(std::move(result));
  }

  std::map<std::string, std::vector<std::string>> groups{
      {"arm", {"joint_1", "joint_2"}}};
  std::map<std::string, SdkToolTransform> tools;
  Status configure_status = Status::Ok();
  Status load_status = Status::Ok();
  mutable Status frame_status = Status::Ok();
  mutable Status fk_status = Status::Ok();
  Status set_tool_status = Status::Ok();
  SdkPose fk_pose;
  SdkInverseKinematicsResult ik_result;

  Configuration configured_with;
  std::string loaded_path;
  SdkToolTransform set_tool;
  mutable KinematicsRequest frame_request;
  mutable std::vector<std::string> frame_joint_names;
  mutable std::vector<double> frame_joints_deg;
  mutable KinematicsRequest fk_request;
  mutable std::vector<double> fk_joints_deg;
  mutable SdkInverseKinematicsRequest ik_request;
  mutable int frame_call_count{0};
  mutable int fk_call_count{0};
  mutable int ik_call_count{0};
};

class SdkKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    fake = std::make_shared<FakeSdkApi>();
    kinematics = std::make_unique<SdkKinematics>(fake);
  }

  std::shared_ptr<FakeSdkApi> fake;
  std::unique_ptr<SdkKinematics> kinematics;
};

TEST_F(SdkKinematicsTest, ConfigureAndLoadMapToSdkLifecycle) {
  Configuration configuration;
  configuration.model_path = "/models/robot.urdf";
  configuration.joint_groups = {{"arm", {"joint_1", "joint_2"}}};

  EXPECT_TRUE(kinematics->configure(configuration));
  EXPECT_EQ(fake->configured_with.model_path, configuration.model_path);
  ASSERT_EQ(fake->configured_with.joint_groups.size(), 1U);
  EXPECT_EQ(fake->configured_with.joint_groups.front().group_name, "arm");

  EXPECT_TRUE(kinematics->load("/config/sdk.yaml"));
  EXPECT_EQ(fake->loaded_path, "/config/sdk.yaml");
}

TEST_F(SdkKinematicsTest, ForwardKinematicsReordersNamesAndConvertsUnits) {
  fake->fk_pose.position_mm = {1000.0, -250.0, 12.5};
  fake->fk_pose.euler_zyx_deg = {0.0, 0.0, 90.0};
  ForwardKinematicsRequest request;
  request.kinematics = {"arm", "torso", "tool"};
  request.joint_state = {{"joint_2", "joint_1"}, {kPi, kPi / 2.0}};

  Result<Pose> result = kinematics->forwardKinematics(request);

  ASSERT_TRUE(result) << result.status.message;
  EXPECT_EQ(fake->frame_request.group_name, "arm");
  EXPECT_EQ(fake->frame_request.base_link, "torso");
  EXPECT_EQ(fake->frame_request.link_name, "tool");
  EXPECT_EQ(fake->frame_joint_names,
            (std::vector<std::string>{"joint_1", "joint_2"}));
  ASSERT_EQ(fake->fk_joints_deg.size(), 2U);
  EXPECT_NEAR(fake->fk_joints_deg[0], 90.0, 1e-12);
  EXPECT_NEAR(fake->fk_joints_deg[1], 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.value->position_m.x, 1.0);
  EXPECT_DOUBLE_EQ(result.value->position_m.y, -0.25);
  EXPECT_DOUBLE_EQ(result.value->position_m.z, 0.0125);
  EXPECT_NEAR(result.value->orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(result.value->orientation.w, std::sqrt(0.5), 1e-12);
  EXPECT_EQ(fake->frame_call_count, 1);
  EXPECT_EQ(fake->fk_call_count, 1);
}

TEST_F(SdkKinematicsTest, FrameFailureDoesNotReturnDefaultPoseAsSuccess) {
  fake->frame_status =
      Status::Error(ErrorCode::kInvalidFrame, "SDK frame missing");
  ForwardKinematicsRequest request{{"arm", "base", "missing"},
                                   {{"joint_1", "joint_2"}, {0.0, 0.0}}};

  Result<Pose> result = kinematics->forwardKinematics(request);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.status.code, ErrorCode::kInvalidFrame);
  EXPECT_EQ(result.status.message, "SDK frame missing");
  EXPECT_EQ(fake->fk_call_count, 0);
}

TEST_F(SdkKinematicsTest, InverseKinematicsMapsPoseSeedParametersAndResult) {
  fake->ik_result = {true,
                     "ok",
                     {"joint_1", "joint_2"},
                     {45.0, -90.0},
                     2.5,
                     0.02};
  InverseKinematicsRequest request;
  request.kinematics = {"arm", "base", "tool"};
  request.target_pose.position_m = {0.1, -0.2, 0.3};
  request.target_pose.orientation = {0.0, 0.0, std::sqrt(0.5),
                                     std::sqrt(0.5)};
  request.seed = JointState{{"joint_2", "joint_1"}, {kPi, kPi / 2.0}};
  request.current_state =
      JointState{{"joint_1", "joint_2"}, {-1.0, -2.0}};
  request.parameters.position_tolerance_m = 0.002;
  request.parameters.retry_min_position_distance_m = 0.01;
  request.parameters.redundancy_preference =
      JointState{{"joint_2", "joint_1"}, {kPi / 4.0, -kPi / 4.0}};

  Result<InverseKinematicsResult> result =
      kinematics->inverseKinematics(request);

  ASSERT_TRUE(result) << result.status.message;
  EXPECT_DOUBLE_EQ(fake->ik_request.target_pose.position_mm.x, 100.0);
  EXPECT_DOUBLE_EQ(fake->ik_request.target_pose.position_mm.y, -200.0);
  EXPECT_DOUBLE_EQ(fake->ik_request.target_pose.position_mm.z, 300.0);
  EXPECT_NEAR(fake->ik_request.target_pose.euler_zyx_deg.z, 90.0, 1e-12);
  ASSERT_EQ(fake->ik_request.q0_deg.size(), 2U);
  EXPECT_NEAR(fake->ik_request.q0_deg[0], 90.0, 1e-12);
  EXPECT_NEAR(fake->ik_request.q0_deg[1], 180.0, 1e-12);
  EXPECT_TRUE(fake->ik_request.parameters.plan_only);
  EXPECT_DOUBLE_EQ(fake->ik_request.parameters.position_tolerance_mm, 2.0);
  EXPECT_DOUBLE_EQ(
      fake->ik_request.parameters.retry_min_position_distance_mm, 10.0);
  ASSERT_EQ(fake->ik_request.parameters
                .redundancy_preference_positions_deg.size(),
            2U);
  EXPECT_NEAR(fake->ik_request.parameters
                  .redundancy_preference_positions_deg[0],
              -45.0, 1e-12);
  EXPECT_NEAR(fake->ik_request.parameters
                  .redundancy_preference_positions_deg[1],
              45.0, 1e-12);
  EXPECT_EQ(result.value->solution.joint_names,
            (std::vector<std::string>{"joint_1", "joint_2"}));
  EXPECT_NEAR(result.value->solution.positions_rad[0], kPi / 4.0, 1e-12);
  EXPECT_NEAR(result.value->solution.positions_rad[1], -kPi / 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.value->position_error_m, 0.0025);
  EXPECT_DOUBLE_EQ(result.value->orientation_error_rad, 0.02);
}

TEST_F(SdkKinematicsTest, CurrentStateGeneratesSeedWhenExplicitSeedIsAbsent) {
  fake->ik_result =
      {true, "ok", {"joint_1", "joint_2"}, {0.0, 0.0}, 0.0, 0.0};
  InverseKinematicsRequest request;
  request.kinematics = {"arm", "base", "tool"};
  request.current_state =
      JointState{{"joint_2", "joint_1"}, {-kPi / 2.0, kPi / 4.0}};

  ASSERT_TRUE(kinematics->inverseKinematics(request));
  ASSERT_EQ(fake->ik_request.q0_deg.size(), 2U);
  EXPECT_NEAR(fake->ik_request.q0_deg[0], 45.0, 1e-12);
  EXPECT_NEAR(fake->ik_request.q0_deg[1], -90.0, 1e-12);
  EXPECT_TRUE(fake->ik_request.parameters.plan_only);
}

TEST_F(SdkKinematicsTest, MissingSeedLeavesSdkCurrentStateSelectionUntouched) {
  fake->ik_result =
      {true, "ok", {"joint_1", "joint_2"}, {0.0, 0.0}, 0.0, 0.0};
  InverseKinematicsRequest request;
  request.kinematics = {"arm", "base", "tool"};

  ASSERT_TRUE(kinematics->inverseKinematics(request));
  EXPECT_TRUE(fake->ik_request.q0_deg.empty());
  EXPECT_FALSE(fake->ik_request.parameters.plan_only);
}

TEST_F(SdkKinematicsTest, SdkIkFailureMessageIsPassedThroughExactly) {
  fake->ik_result = {false, "TRAC-IK timed out"};
  InverseKinematicsRequest request;
  request.kinematics = {"arm", "base", "tool"};

  Result<InverseKinematicsResult> result =
      kinematics->inverseKinematics(request);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.status.code, ErrorCode::kSdkFailure);
  EXPECT_EQ(result.status.message, "TRAC-IK timed out");
}

TEST_F(SdkKinematicsTest, RejectsGroupFrameAndJointStateErrors) {
  EXPECT_EQ(kinematics->jointGroup("missing").status.code,
            ErrorCode::kInvalidGroup);
  EXPECT_EQ(kinematics
                ->validateJointState(
                    "arm", {{"joint_1", "joint_1"}, {0.0, 0.0}})
                .code,
            ErrorCode::kInvalidJointState);

  ForwardKinematicsRequest empty_frame{{"arm", "", "tool"},
                                       {{"joint_1", "joint_2"}, {0.0, 0.0}}};
  EXPECT_EQ(kinematics->forwardKinematics(empty_frame).status.code,
            ErrorCode::kInvalidFrame);

  ForwardKinematicsRequest missing_joint{
      {"arm", "base", "tool"}, {{"joint_1"}, {0.0}}};
  EXPECT_EQ(kinematics->forwardKinematics(missing_joint).status.code,
            ErrorCode::kInvalidJointState);
}

TEST_F(SdkKinematicsTest, ToolTransformsConvertLengthAndPassErrors) {
  ToolTransform tool{"camera",
                     "wrist",
                     {0.1, -0.02, 0.003},
                     {0.1, 0.2, 0.3}};
  ASSERT_TRUE(kinematics->setToolTransform(tool));
  EXPECT_DOUBLE_EQ(fake->set_tool.translation_mm.x, 100.0);
  EXPECT_DOUBLE_EQ(fake->set_tool.translation_mm.y, -20.0);
  EXPECT_DOUBLE_EQ(fake->set_tool.translation_mm.z, 3.0);
  EXPECT_DOUBLE_EQ(fake->set_tool.rpy_rad.z, 0.3);

  Result<ToolTransform> fetched = kinematics->getToolTransform("camera");
  ASSERT_TRUE(fetched);
  EXPECT_DOUBLE_EQ(fetched.value->translation_m.x, 0.1);
  EXPECT_EQ(kinematics->getToolTransform("missing").status.code,
            ErrorCode::kInvalidTool);

  fake->set_tool_status =
      Status::Error(ErrorCode::kInvalidTool, "SDK invalid parent");
  Status failed = kinematics->setToolTransform(
      {"bad", "missing", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  EXPECT_EQ(failed.message, "SDK invalid parent");
}

TEST_F(SdkKinematicsTest, RepeatedCallsReturnIdenticalResults) {
  fake->fk_pose.position_mm = {1.0, 2.0, 3.0};
  fake->fk_pose.euler_zyx_deg = {10.0, 20.0, 30.0};
  ForwardKinematicsRequest request{{"arm", "base", "tool"},
                                   {{"joint_1", "joint_2"}, {0.1, 0.2}}};

  Result<Pose> first = kinematics->forwardKinematics(request);
  Result<Pose> second = kinematics->forwardKinematics(request);

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_DOUBLE_EQ(first.value->position_m.x, second.value->position_m.x);
  EXPECT_DOUBLE_EQ(first.value->orientation.x,
                   second.value->orientation.x);
  EXPECT_DOUBLE_EQ(first.value->orientation.y,
                   second.value->orientation.y);
  EXPECT_DOUBLE_EQ(first.value->orientation.z,
                   second.value->orientation.z);
  EXPECT_DOUBLE_EQ(first.value->orientation.w,
                   second.value->orientation.w);
}

}  // namespace
}  // namespace humanoid_motion_server::kinematics
