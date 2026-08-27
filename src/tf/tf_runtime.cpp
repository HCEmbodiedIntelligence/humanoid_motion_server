// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_motion_server/tf/tf_runtime.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>

#include "kdl_parser/kdl_parser.hpp"
#include "yaml-cpp/yaml.h"

namespace humanoid_motion_server::tf
{
namespace
{

std::string readTextFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open URDF file: " + path);
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

geometry_msgs::msg::TransformStamped toolTransform(
  const YAML::Node & entry, const rclcpp::Time & stamp)
{
  const auto translation = entry["translation_m"].as<std::vector<double>>();
  const auto rotation = entry["rotation_xyzw"].as<std::vector<double>>();
  if (translation.size() != 3U || rotation.size() != 4U) {
    throw std::runtime_error("tool TF requires three translation and four quaternion values");
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = entry["parent_frame"].as<std::string>();
  transform.child_frame_id = entry["child_frame"].as<std::string>();
  transform.transform.translation.x = translation[0];
  transform.transform.translation.y = translation[1];
  transform.transform.translation.z = translation[2];
  transform.transform.rotation.x = rotation[0];
  transform.transform.rotation.y = rotation[1];
  transform.transform.rotation.z = rotation[2];
  transform.transform.rotation.w = rotation[3];
  return transform;
}

KDL::Frame toolFrame(const YAML::Node & entry)
{
  const auto translation = entry["translation_m"].as<std::vector<double>>();
  const auto rotation = entry["rotation_xyzw"].as<std::vector<double>>();
  return {
    KDL::Rotation::Quaternion(rotation[0], rotation[1], rotation[2], rotation[3]),
    KDL::Vector(translation[0], translation[1], translation[2])};
}

}  // namespace

TfRuntime::TfRuntime(
  rclcpp::Node & node, const std::string & urdf_path,
  const std::string & tool_config_path)
: node_(node),
  broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node_)),
  static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_))
{
  if (urdf_path.empty() || tool_config_path.empty()) {
    throw std::invalid_argument("urdf_file and tool_config_file are required for TF");
  }
  if (!kdl_parser::treeFromString(readTextFile(urdf_path), tree_)) {
    throw std::runtime_error("failed to parse URDF into a KDL tree: " + urdf_path);
  }

  std::vector<geometry_msgs::msg::TransformStamped> fixed;
  collectSegments(tree_.getRootSegment(), dynamic_segments_, fixed);
  try {
    const auto document = YAML::LoadFile(tool_config_path);
    if (!document.IsMap() || !document["tools"] || !document["tools"].IsSequence()) {
      throw std::runtime_error("tool config must contain a tools sequence");
    }
    for (const auto & entry : document["tools"]) {
      fixed.push_back(toolTransform(entry, node_.now()));
      tool_frames_.emplace_back(
        entry["parent_frame"].as<std::string>(),
        entry["child_frame"].as<std::string>(), toolFrame(entry));
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::string("failed to parse tool TF config: ") + error.what());
  }
  if (!fixed.empty()) {
    static_broadcaster_->sendTransform(fixed);
  }
}

void TfRuntime::publishFeedback(const sensor_msgs::msg::JointState & feedback)
{
  if (feedback.name.size() != feedback.position.size()) {
    return;
  }
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < feedback.name.size(); ++index) {
    positions.emplace(feedback.name[index], feedback.position[index]);
  }
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(dynamic_segments_.size());
  const rclcpp::Time stamp(feedback.header.stamp, node_.get_clock()->get_clock_type());
  for (const auto segment : dynamic_segments_) {
    const auto & joint = segment->second.segment.getJoint();
    const auto position = positions.find(joint.getName());
    if (position == positions.end()) {
      continue;
    }
    const auto parent = segment->second.parent;
    transforms.push_back(
      toTransform(
        parent->second.segment.getName(), segment->second.segment.getName(),
        segment->second.segment.pose(position->second), stamp));
  }
  if (!transforms.empty()) {
    broadcaster_->sendTransform(transforms);
  }

  std::map<std::string, KDL::Frame> poses;
  const auto root = tree_.getRootSegment();
  poses.emplace(root->second.segment.getName(), KDL::Frame::Identity());
  if (collectFramePoses(root, KDL::Frame::Identity(), positions, poses)) {
    for (const auto & [parent, child, frame] : tool_frames_) {
      const auto parent_pose = poses.find(parent);
      if (parent_pose != poses.end()) {
        poses[child] = parent_pose->second * frame;
      }
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_poses_ = std::move(poses);
  }
}

bool TfRuntime::transformPose(
  const geometry_msgs::msg::PoseStamped & input, const std::string & target_frame,
  geometry_msgs::msg::PoseStamped & output, std::string & error) const
{
  if (input.header.frame_id.empty() || target_frame.empty()) {
    error = "pose source and target frames must be non-empty";
    return false;
  }
  const auto & p = input.pose.position;
  const auto & q = input.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm) || norm < 1.0e-9)
  {
    error = "pose contains a non-finite value or non-normalizable quaternion";
    return false;
  }
  const KDL::Frame source_pose(
    KDL::Rotation::Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm),
    KDL::Vector(p.x, p.y, p.z));
  KDL::Frame transformed;
  if (input.header.frame_id == target_frame) {
    transformed = source_pose;
  } else {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    const auto source = frame_poses_.find(input.header.frame_id);
    const auto target = frame_poses_.find(target_frame);
    if (source == frame_poses_.end() || target == frame_poses_.end()) {
      error = "measured transform from '" + input.header.frame_id + "' to '" +
        target_frame + "' is unavailable";
      return false;
    }
    transformed = target->second.Inverse() * source->second * source_pose;
  }
  output.header = input.header;
  output.header.frame_id = target_frame;
  output.pose.position.x = transformed.p.x();
  output.pose.position.y = transformed.p.y();
  output.pose.position.z = transformed.p.z();
  transformed.M.GetQuaternion(
    output.pose.orientation.x, output.pose.orientation.y,
    output.pose.orientation.z, output.pose.orientation.w);
  return true;
}

void TfRuntime::collectSegments(
  const KDL::SegmentMap::const_iterator parent,
  std::vector<KDL::SegmentMap::const_iterator> & dynamic_segments,
  std::vector<geometry_msgs::msg::TransformStamped> & fixed_transforms) const
{
  for (const auto child : parent->second.children) {
    const auto & segment = child->second.segment;
    if (segment.getJoint().getType() == KDL::Joint::None) {
      fixed_transforms.push_back(
        toTransform(
          parent->second.segment.getName(), segment.getName(), segment.pose(0.0), node_.now()));
    } else {
      dynamic_segments.push_back(child);
    }
    collectSegments(child, dynamic_segments, fixed_transforms);
  }
}

bool TfRuntime::collectFramePoses(
  const KDL::SegmentMap::const_iterator parent, const KDL::Frame & parent_pose,
  const std::map<std::string, double> & positions,
  std::map<std::string, KDL::Frame> & frame_poses) const
{
  for (const auto child : parent->second.children) {
    const auto & segment = child->second.segment;
    double position = 0.0;
    if (segment.getJoint().getType() != KDL::Joint::None) {
      const auto found = positions.find(segment.getJoint().getName());
      if (found == positions.end()) {
        // A driver may intentionally expose only a controllable subset (for
        // example the arms while the URDF also contains finger joints).  The
        // unknown joint makes only this subtree unavailable; it must not erase
        // measured poses already collected for the rest of the robot.
        continue;
      }
      position = found->second;
    }
    const KDL::Frame pose = parent_pose * segment.pose(position);
    frame_poses[segment.getName()] = pose;
    collectFramePoses(child, pose, positions, frame_poses);
  }
  return true;
}

geometry_msgs::msg::TransformStamped TfRuntime::toTransform(
  const std::string & parent, const std::string & child, const KDL::Frame & frame,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = frame.p.x();
  transform.transform.translation.y = frame.p.y();
  transform.transform.translation.z = frame.p.z();
  frame.M.GetQuaternion(
    transform.transform.rotation.x, transform.transform.rotation.y,
    transform.transform.rotation.z, transform.transform.rotation.w);
  return transform;
}

}  // namespace humanoid_motion_server::tf
