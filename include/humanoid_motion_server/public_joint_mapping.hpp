#ifndef HUMANOID_MOTION_SERVER__PUBLIC_JOINT_MAPPING_HPP_
#define HUMANOID_MOTION_SERVER__PUBLIC_JOINT_MAPPING_HPP_

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"

namespace humanoid_motion_server
{
// Reads only the resolved artifact produced by HC's single profile parser.
// URDF references, groups/home and vendor/SDK units are not reinterpreted here.
class PublicJointMapping
{
public:
  void load(const std::string & path)
  {
    if (path.empty()) {return;}  // Explicit legacy deployments only.
    const auto root = YAML::LoadFile(path);
    if (root["schema"].as<std::string>() != "hc-robot-api/v1" ||
      !root["joints"].IsSequence() || root["joints"].size() == 0)
    {
      throw std::runtime_error("invalid resolved public joint mapping");
    }
    for (const auto & item : root["joints"]) {
      Entry entry{item["model_name"].as<std::string>(), item["public_name"].as<std::string>(),
        item["scale"].as<double>(), item["offset"].as<double>()};
      if (entry.model.empty() || entry.public_name.empty() ||
        !std::isfinite(entry.scale) || std::abs(entry.scale) < 1e-12 || !std::isfinite(entry.offset) ||
        !model_index_.emplace(entry.model, entries_.size()).second ||
        !public_index_.emplace(entry.public_name, entries_.size()).second)
      {
        throw std::runtime_error("invalid/duplicate public mapping entry");
      }
      entries_.push_back(std::move(entry));
    }
  }

  bool enabled() const {return !entries_.empty();}

  sensor_msgs::msg::JointState convert(
    const sensor_msgs::msg::JointState & input, bool to_model) const
  {
    if (!enabled()) {return input;}
    const auto size = input.name.size();
    if (size == 0 || input.position.size() != size ||
      (!input.velocity.empty() && input.velocity.size() != size) ||
      (!input.effort.empty() && input.effort.size() != size))
    {
      throw std::invalid_argument("JointState array lengths differ");
    }
    const auto & index = to_model ? public_index_ : model_index_;
    std::map<std::size_t, std::size_t> present;
    for (std::size_t i = 0; i < size; ++i) {
      const auto found = index.find(input.name[i]);
      if (found == index.end() || !present.emplace(found->second, i).second) {
        throw std::invalid_argument("unknown/duplicate JointState name: " + input.name[i]);
      }
      if (!std::isfinite(input.position[i]) ||
        (!input.velocity.empty() && !std::isfinite(input.velocity[i])) ||
        (!input.effort.empty() && !std::isfinite(input.effort[i])))
      {
        throw std::invalid_argument("nonfinite JointState value");
      }
    }
    sensor_msgs::msg::JointState output;
    output.header = input.header;
    for (const auto & [entry_index, i] : present) {
      const auto & entry = entries_[entry_index];
      const auto s = entry.scale;
      output.name.push_back(to_model ? entry.model : entry.public_name);
      output.position.push_back(to_model ? s * input.position[i] + entry.offset :
        (input.position[i] - entry.offset) / s);
      if (!input.velocity.empty()) {
        output.velocity.push_back(to_model ? s * input.velocity[i] : input.velocity[i] / s);
      }
      if (!input.effort.empty()) {
        output.effort.push_back(to_model ? input.effort[i] / s : s * input.effort[i]);
      }
    }
    return output;
  }

private:
  struct Entry {std::string model, public_name; double scale, offset;};
  std::vector<Entry> entries_;
  std::map<std::string, std::size_t> model_index_, public_index_;
};
}  // namespace humanoid_motion_server
#endif
