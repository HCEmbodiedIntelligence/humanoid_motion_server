#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>

#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"

namespace m = humanoid_motion_server::motion;

// Exercise the delivered RTC library, including SI conversion and joint order.
// All assets are repository fixtures; no ROS process or hardware is created.
TEST(SdkLimitRecovery, RebasedStatePreservesOtherJointVelocityAndRejectsInvalidState)
{
  char path[] = "/tmp/sdk-limit-recovery-XXXXXX";
  ASSERT_NE(mkdtemp(path), nullptr);
  const std::filesystem::path root(path);
  struct Cleanup {
    std::filesystem::path root;
    ~Cleanup() {std::filesystem::remove_all(root);}
  } cleanup{root};
  const std::filesystem::path source(RECOVERY_TEST_SDK_CONFIG);
  auto yaml = YAML::LoadFile(source.string());
  yaml["model_path"] = (source.parent_path() / yaml["model_path"].as<std::string>()).string();
  std::ofstream(root / "sdk.yaml") << yaml;
  m::MotionContextFactoryOptions options;
  options.joint_groups = {{"left_arm", {"left_shoulder_pitch", "left_elbow"},
    {-2.4, -2.5}, {2.4, .1}}};
  auto factory = m::MotionContextFactory::createFromSdkYaml((root / "sdk.yaml").string(), options);
  ASSERT_TRUE(factory.status.ok()) << factory.status.message;

  // The elbow has just returned inside; the other joint is already moving.
  m::JointTarget seed{{"left_elbow", "left_shoulder_pitch"},
    {-2.499808, .2}, {0., .05}, {0., .01}};
  ASSERT_TRUE(factory.backend->rebaseFinalJointTarget("left_arm", seed).ok());
  m::JointCommand candidate{"left_arm", {"left_shoulder_pitch", "left_elbow"},
    {.6, -2.4}, {0., 0.}, {0., 0.}};
  const auto output = factory.backend->updateFinalJointTarget("left_arm", candidate, .01);
  ASSERT_TRUE(output.status.ok()) << output.status.message;
  ASSERT_TRUE(output.candidate.passed_final_sdk_rtc);
  ASSERT_EQ(output.candidate.positions_rad.size(), 2U);
  EXPECT_NEAR(output.candidate.positions_rad[0], .2005, .00002);
  EXPECT_GT(output.candidate.velocities_rad_s[0], .045);
  EXPECT_GE(output.candidate.positions_rad[1], seed.positions_rad[0]);
  EXPECT_LE(output.candidate.positions_rad[1], .1);

  seed.positions_rad[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(factory.backend->rebaseFinalJointTarget("left_arm", seed).code,
    m::StatusCode::INVALID_ARGUMENT);
}
