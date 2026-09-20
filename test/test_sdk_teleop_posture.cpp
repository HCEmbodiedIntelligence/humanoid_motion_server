#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"

namespace m = humanoid_motion_server::motion;

// Two coaxial joints provide an exact redundant degree of freedom per arm.
// This exercises the shipped SDK without ROS, drivers or real robot assets.
TEST(SdkTeleopPosture, SelectedArmPreferenceChangesRedundantSolution)
{
  char path[] = "/tmp/sdk-posture-test-XXXXXX";
  ASSERT_NE(mkdtemp(path), nullptr);
  const std::filesystem::path root(path);
  struct Cleanup {
    std::filesystem::path root;
    ~Cleanup() {std::filesystem::remove_all(root);}
  } cleanup{root};
  std::ofstream urdf(root / "robot.urdf");
  urdf << "<robot name='posture_test'><link name='base_link'/>";
  m::MotionContextFactoryOptions options;
  auto yaml = YAML::LoadFile(TELEOP_TEST_SDK_CONFIG);
  yaml["model_path"] = (root / "robot.urdf").string();
  yaml.remove("joint_groups");
  yaml.remove("tools");
  std::vector<std::string> all;
  for (const auto & side : {std::string("left"), std::string("right")}) {
    std::string parent = "base_link";
    std::vector<std::string> joints;
    for (int i = 1; i <= 2; ++i) {
      const auto joint = side + std::to_string(i);
      const auto link = joint + "_link";
      urdf << "<link name='" << link << "'/><joint name='" << joint
           << "' type='revolute'><parent link='" << parent << "'/><child link='"
           << link << "'/><axis xyz='0 0 1'/><limit lower='-1' upper='1' "
              "effort='10' velocity='2'/></joint>";
      parent = link;
      joints.push_back(joint);
      all.push_back(joint);
    }
    options.joint_groups.push_back({side + "_arm", joints, {-1., -1.}, {1., 1.}});
    yaml["joint_groups"][side + "_arm"] = joints;
    YAML::Node tool;
    tool["name"] = side + "_tool0";
    tool["parent_link"] = parent;
    tool["x"] = 0.;
    for (const auto & axis : {"y", "z", "rx", "ry", "rz"}) {tool[axis] = 0.;}
    yaml["tools"].push_back(tool);
  }
  urdf << "</robot>";
  urdf.close();
  yaml["joint_groups"]["whole_body"] = all;
  yaml["wbc"]["active_joint_names"] = all;
  auto run = [&](bool preference) {
      if (preference) {
        auto entry = yaml["teleop_posture"]["right_arm"];
        entry["joint_names"] = options.joint_groups[1].joint_names;
        entry["positions_rad"] = std::vector<double>{-.1, 1.};
        entry["weight"] = .001;
      }
      std::ofstream(root / "sdk.yaml") << yaml;
      const auto factory = m::MotionContextFactory::createFromSdkYaml((root / "sdk.yaml").string(), options);
      EXPECT_TRUE(factory.status.ok()) << factory.status.message;
      std::vector<std::vector<double>> result;
      if (!factory.status.ok()) {return result;}
      for (const auto & group : options.joint_groups) {
        m::ForwardKinematicsRequest fk;
        fk.group_name = group.name;
        fk.base_link = "base_link";
        fk.link_name = group.name == "left_arm" ? "left_tool0" : "right_tool0";
        fk.joints = {group.joint_names, {.5, .4}, {}, {}};
        const auto pose = factory.backend->forwardKinematics(fk);
        EXPECT_TRUE(pose.status.ok());
        m::ServoPRequest request;
        request.group_name = group.name;
        request.base_link = fk.base_link;
        request.link_name = fk.link_name;
        request.target = pose.pose;
        request.limits = {{2., 2.}, {8., 8.}, {30., 30.}, .1, .3, 1.5, .5, 2., 10.};
        m::JointFeedback seed{group.joint_names, {.2, .3}, {0., 0.}, m::SteadyClock::now()};
        const auto started = factory.backend->startSession(group.name, request, seed, .01);
        EXPECT_TRUE(started.ok()) << started.message;
        if (!started.ok()) {return result;}
        m::DynamicTarget target = pose.pose;
        m::BackendTick tick;
        for (int i = 0; i < 300; ++i) {
          tick = factory.backend->tickSession(group.name, &target, .01);
          EXPECT_TRUE(tick.status.ok()) << tick.status.message;
          if (!tick.status.ok()) {return result;}
        }
        result.push_back(tick.candidate.positions_rad);
        EXPECT_NEAR(result.back()[0] + result.back()[1], .9, .002);
        factory.backend->stopSession(group.name);
      }
      return result;
    };
  const auto baseline = run(false);
  const auto preferred = run(true);
  ASSERT_EQ(baseline.size(), 2U);
  ASSERT_EQ(preferred.size(), 2U);
  EXPECT_NEAR(baseline[0][0], preferred[0][0], .001);
  EXPECT_NEAR(baseline[0][1], preferred[0][1], .001);
  EXPECT_LT(std::abs(preferred[1][0] + .1), std::abs(baseline[1][0] + .1) - .1);
  // A joint order mismatch must fail before creating a usable backend.
  yaml["teleop_posture"]["right_arm"]["joint_names"] = std::vector<std::string>{"right2", "right1"};
  std::ofstream(root / "sdk.yaml") << yaml;
  const auto invalid = m::MotionContextFactory::createFromSdkYaml((root / "sdk.yaml").string(), options);
  EXPECT_FALSE(invalid.status.ok());
  EXPECT_EQ(invalid.backend, nullptr);
}
