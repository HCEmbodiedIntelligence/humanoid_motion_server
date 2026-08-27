// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_MOTION_SERVER__CHANNEL_CONFIG_HPP_
#define HUMANOID_MOTION_SERVER__CHANNEL_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace humanoid_motion_server
{

enum class ChannelKind
{
  MOVE_J,
  MOVE_L,
  MOVE_P,
  SERVO_J,
  SERVO_P,
};

struct ChannelConfig
{
  std::string name;
  ChannelKind kind;
  std::string endpoint;
  std::int64_t priority;
  std::string group;
  std::string base_frame;
  std::string tip_frame;
  std::string fk_pose_topic;
};

std::string to_string(ChannelKind kind);
bool is_action(ChannelKind kind);
bool is_servo(ChannelKind kind);
bool is_cartesian(ChannelKind kind);

/// Load the channel file using a deliberately closed schema. Unknown keys,
/// missing required values, duplicate identities/endpoints, and invalid types
/// all raise std::runtime_error.
std::vector<ChannelConfig> load_channel_config(const std::string & path);

}  // namespace humanoid_motion_server

#endif  // HUMANOID_MOTION_SERVER__CHANNEL_CONFIG_HPP_
