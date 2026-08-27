// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "humanoid_motion_server/channel_config.hpp"

namespace
{

class TemporaryYaml
{
public:
  explicit TemporaryYaml(const std::string & contents)
  {
    path_ = std::filesystem::temp_directory_path() /
      ("humanoid_channels_" + std::to_string(next_id_++) + ".yaml");
    std::ofstream stream(path_);
    stream << contents;
  }

  ~TemporaryYaml()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
  static inline std::size_t next_id_{0U};
};

const char * kValidConfig =
  R"(
channels:
  - name: normal_move_j
    kind: move_j
    endpoint: /normal/move_j
    priority: 50
  - name: teleop_servo_p
    kind: servo_p
    endpoint: /teleop/servo_p
    priority: 100
    group: left_arm
    base_frame: base_link
    tip_frame: tool0
    fk_pose_topic: /teleop/left_arm/fk_pose
)";

TEST(ChannelConfig, LoadsTheClosedSchema)
{
  TemporaryYaml yaml(kValidConfig);
  const auto channels = humanoid_motion_server::load_channel_config(yaml.path());
  ASSERT_EQ(channels.size(), 2U);
  EXPECT_EQ(channels[0].kind, humanoid_motion_server::ChannelKind::MOVE_J);
  EXPECT_EQ(channels[1].kind, humanoid_motion_server::ChannelKind::SERVO_P);
  EXPECT_EQ(channels[1].priority, 100);
  EXPECT_EQ(channels[1].group, "left_arm");
  EXPECT_EQ(channels[1].fk_pose_topic, "/teleop/left_arm/fk_pose");
}

TEST(ChannelConfig, RejectsUnknownKeys)
{
  TemporaryYaml yaml(
    R"(
channels:
  - name: bad
    kind: move_j
    endpoint: /bad
    priority: 1
    priority_from_message: true
)");
  EXPECT_THROW(humanoid_motion_server::load_channel_config(yaml.path()), std::runtime_error);
}

TEST(ChannelConfig, RejectsServoWithoutGroup)
{
  TemporaryYaml yaml(
    R"(
channels:
  - name: bad
    kind: servo_j
    endpoint: /bad
    priority: 1
)");
  EXPECT_THROW(humanoid_motion_server::load_channel_config(yaml.path()), std::runtime_error);
}

TEST(ChannelConfig, RejectsDuplicateEndpointsAndInvalidPriorityTypes)
{
  TemporaryYaml duplicate(
    R"(
channels:
  - {name: first, kind: move_j, endpoint: /same, priority: 1}
  - {name: second, kind: move_l, endpoint: /same, priority: 2,
     base_frame: base_link, tip_frame: tool0}
)");
  EXPECT_THROW(
    humanoid_motion_server::load_channel_config(duplicate.path()), std::runtime_error);

  TemporaryYaml bad_priority(
    R"(
channels:
  - {name: bad, kind: move_j, endpoint: /bad, priority: high}
)");
  EXPECT_THROW(
    humanoid_motion_server::load_channel_config(bad_priority.path()), std::runtime_error);
}

TEST(ChannelConfig, RejectsFkTopicOnNonCartesianOrDuplicateFkTopics)
{
  TemporaryYaml non_cartesian(
    R"(
channels:
  - {name: bad, kind: servo_j, endpoint: /bad, priority: 1,
     group: left_arm, fk_pose_topic: /fk}
)");
  EXPECT_THROW(
    humanoid_motion_server::load_channel_config(non_cartesian.path()), std::runtime_error);

  TemporaryYaml duplicate(
    R"(
channels:
  - {name: left, kind: servo_p, endpoint: /left, priority: 1,
     group: left_arm, base_frame: base, tip_frame: left_tool, fk_pose_topic: /fk}
  - {name: right, kind: servo_p, endpoint: /right, priority: 1,
     group: right_arm, base_frame: base, tip_frame: right_tool, fk_pose_topic: /fk}
)");
  EXPECT_THROW(
    humanoid_motion_server::load_channel_config(duplicate.path()), std::runtime_error);
}

}  // namespace
