// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_motion_server/channel_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace humanoid_motion_server
{
namespace
{

const std::set<std::string> kRootKeys{"channels"};
const std::set<std::string> kChannelKeys{
  "name", "kind", "endpoint", "priority", "group", "base_frame", "tip_frame",
  "fk_pose_topic"};

void require_closed_map(
  const YAML::Node & node, const std::set<std::string> & allowed,
  const std::string & context)
{
  if (!node.IsMap()) {
    throw std::runtime_error(context + " must be a YAML mapping");
  }
  for (const auto & entry : node) {
    if (!entry.first.IsScalar()) {
      throw std::runtime_error(context + " contains a non-scalar key");
    }
    const auto key = entry.first.as<std::string>();
    if (allowed.count(key) == 0U) {
      throw std::runtime_error(context + " contains unknown key '" + key + "'");
    }
  }
}

std::string required_string(
  const YAML::Node & node, const std::string & key, const std::string & context)
{
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error(context + "." + key + " must be a non-empty string");
  }
  std::string value;
  try {
    value = node[key].as<std::string>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(context + "." + key + " must be a non-empty string");
  }
  if (value.empty()) {
    throw std::runtime_error(context + "." + key + " must be a non-empty string");
  }
  return value;
}

std::string optional_string(
  const YAML::Node & node, const std::string & key, const std::string & context)
{
  if (!node[key]) {
    return {};
  }
  if (!node[key].IsScalar()) {
    throw std::runtime_error(context + "." + key + " must be a string");
  }
  try {
    return node[key].as<std::string>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(context + "." + key + " must be a string");
  }
}

ChannelKind parse_kind(const std::string & value, const std::string & context)
{
  static const std::unordered_map<std::string, ChannelKind> kinds{
    {"move_j", ChannelKind::MOVE_J}, {"move_l", ChannelKind::MOVE_L},
    {"move_p", ChannelKind::MOVE_P}, {"servo_j", ChannelKind::SERVO_J},
    {"servo_p", ChannelKind::SERVO_P}};
  const auto it = kinds.find(value);
  if (it == kinds.end()) {
    throw std::runtime_error(
            context + ".kind must be one of move_j, move_l, move_p, servo_j, servo_p");
  }
  return it->second;
}

}  // namespace

std::string to_string(const ChannelKind kind)
{
  switch (kind) {
    case ChannelKind::MOVE_J:
      return "move_j";
    case ChannelKind::MOVE_L:
      return "move_l";
    case ChannelKind::MOVE_P:
      return "move_p";
    case ChannelKind::SERVO_J:
      return "servo_j";
    case ChannelKind::SERVO_P:
      return "servo_p";
  }
  throw std::logic_error("unhandled channel kind");
}

bool is_action(const ChannelKind kind)
{
  return kind == ChannelKind::MOVE_J || kind == ChannelKind::MOVE_L ||
         kind == ChannelKind::MOVE_P;
}

bool is_servo(const ChannelKind kind)
{
  return kind == ChannelKind::SERVO_J || kind == ChannelKind::SERVO_P;
}

bool is_cartesian(const ChannelKind kind)
{
  return kind == ChannelKind::MOVE_L || kind == ChannelKind::MOVE_P ||
         kind == ChannelKind::SERVO_P;
}

std::vector<ChannelConfig> load_channel_config(const std::string & path)
{
  if (path.empty()) {
    throw std::runtime_error("channel_config_file parameter is empty");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse channel config '" + path + "': " + error.what());
  }
  require_closed_map(root, kRootKeys, "channel config root");
  if (!root["channels"] || !root["channels"].IsSequence() || root["channels"].size() == 0U) {
    throw std::runtime_error("channel config root.channels must be a non-empty sequence");
  }

  std::set<std::string> names;
  std::set<std::string> endpoints;
  std::set<std::string> fk_pose_topics;
  std::vector<ChannelConfig> result;
  result.reserve(root["channels"].size());

  for (std::size_t index = 0; index < root["channels"].size(); ++index) {
    const auto item = root["channels"][index];
    const auto context = "channels[" + std::to_string(index) + "]";
    require_closed_map(item, kChannelKeys, context);

    ChannelConfig channel;
    channel.name = required_string(item, "name", context);
    channel.kind = parse_kind(required_string(item, "kind", context), context);
    channel.endpoint = required_string(item, "endpoint", context);
    channel.group = optional_string(item, "group", context);
    channel.base_frame = optional_string(item, "base_frame", context);
    channel.tip_frame = optional_string(item, "tip_frame", context);
    channel.fk_pose_topic = optional_string(item, "fk_pose_topic", context);

    if (!item["priority"] || !item["priority"].IsScalar()) {
      throw std::runtime_error(context + ".priority must be an integer");
    }
    try {
      channel.priority = item["priority"].as<std::int64_t>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error(context + ".priority must be an integer");
    }

    if (is_servo(channel.kind) && channel.group.empty()) {
      throw std::runtime_error(context + ".group is required for Servo channels");
    }
    if (is_cartesian(channel.kind) &&
      (channel.base_frame.empty() || channel.tip_frame.empty()))
    {
      throw std::runtime_error(
              context + ".base_frame and tip_frame are required for Cartesian channels");
    }
    if (!channel.fk_pose_topic.empty() &&
      (!is_cartesian(channel.kind) || channel.group.empty()))
    {
      throw std::runtime_error(
              context + ".fk_pose_topic requires a Cartesian channel with a group");
    }
    if (!names.insert(channel.name).second) {
      throw std::runtime_error("duplicate channel name '" + channel.name + "'");
    }
    if (!endpoints.insert(channel.endpoint).second) {
      throw std::runtime_error("duplicate channel endpoint '" + channel.endpoint + "'");
    }
    if (!channel.fk_pose_topic.empty() &&
      !fk_pose_topics.insert(channel.fk_pose_topic).second)
    {
      throw std::runtime_error("duplicate FK pose topic '" + channel.fk_pose_topic + "'");
    }
    result.push_back(std::move(channel));
  }
  return result;
}

}  // namespace humanoid_motion_server
