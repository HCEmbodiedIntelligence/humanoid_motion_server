#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include "humanoid_motion_server/public_joint_mapping.hpp"

TEST(PublicJointMapping, ResolvedArtifactRoundtripPreservesStampOptionalFieldsAndPower)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("hc-joint-map-" + std::to_string(getpid()) + ".yaml");
  {
    std::ofstream stream(path);
    stream << "schema: hc-robot-api/v1\njoints:\n"
      "- {model_name: axis_a, public_name: left_joint_1, scale: -2.0, offset: 0.25}\n"
      "- {model_name: axis_b, public_name: left_joint_2, scale: 0.5, offset: -0.2}\n";
  }
  humanoid_motion_server::PublicJointMapping mapping;
  mapping.load(path.string());
  std::filesystem::remove(path);
  sensor_msgs::msg::JointState source;
  source.header.stamp.sec = 123;
  source.header.stamp.nanosec = 456;
  source.name = {"left_joint_2", "left_joint_1"};
  source.position = {0.4, 0.3};
  source.velocity = {0.8, -0.4};
  source.effort = {2.0, 3.0};
  const auto model = mapping.convert(source, true);
  EXPECT_EQ(model.name, (std::vector<std::string>{"axis_a", "axis_b"}));
  EXPECT_NEAR(model.position[0], -0.35, 1e-12);
  EXPECT_NEAR(model.position[1], 0.0, 1e-12);
  EXPECT_NEAR(model.velocity[0], 0.8, 1e-12);
  EXPECT_NEAR(model.effort[0], -1.5, 1e-12);
  EXPECT_NEAR(model.velocity[0] * model.effort[0] + model.velocity[1] * model.effort[1],
    source.velocity[0] * source.effort[0] + source.velocity[1] * source.effort[1], 1e-12);
  const auto roundtrip = mapping.convert(model, false);
  EXPECT_EQ(roundtrip.header, source.header);
  EXPECT_DOUBLE_EQ(roundtrip.position[0], source.position[1]);
  EXPECT_DOUBLE_EQ(roundtrip.position[1], source.position[0]);
  source.velocity.clear(); source.effort.clear();
  EXPECT_TRUE(mapping.convert(source, true).velocity.empty());
  EXPECT_TRUE(mapping.convert(source, true).effort.empty());
  source.name[1] = source.name[0];
  EXPECT_THROW(mapping.convert(source, true), std::invalid_argument);
  source.name = {"axis_b", "axis_a"};
  EXPECT_THROW(mapping.convert(source, true), std::invalid_argument);
}
