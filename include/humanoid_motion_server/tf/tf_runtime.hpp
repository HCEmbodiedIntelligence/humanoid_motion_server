// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_MOTION_SERVER__TF__TF_RUNTIME_HPP_
#define HUMANOID_MOTION_SERVER__TF__TF_RUNTIME_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kdl/tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace humanoid_motion_server::tf
{

/// Publishes the URDF tree from measured driver positions. This is a ROS-aware
/// helper owned by the one control node; it never creates a node of its own.
class TfRuntime
{
public:
  TfRuntime(
    rclcpp::Node & node, const std::string & urdf_path,
    const std::string & tool_config_path);

  void publishFeedback(const sensor_msgs::msg::JointState & feedback);
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input, const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & output, std::string & error) const;

private:
  void collectSegments(
    KDL::SegmentMap::const_iterator parent,
    std::vector<KDL::SegmentMap::const_iterator> & dynamic_segments,
    std::vector<geometry_msgs::msg::TransformStamped> & fixed_transforms) const;
  static geometry_msgs::msg::TransformStamped toTransform(
    const std::string & parent, const std::string & child, const KDL::Frame & frame,
    const rclcpp::Time & stamp);
  bool collectFramePoses(
    KDL::SegmentMap::const_iterator parent, const KDL::Frame & parent_pose,
    const std::map<std::string, double> & positions,
    std::map<std::string, KDL::Frame> & frame_poses) const;

  rclcpp::Node & node_;
  KDL::Tree tree_;
  std::vector<KDL::SegmentMap::const_iterator> dynamic_segments_;
  std::vector<std::tuple<std::string, std::string, KDL::Frame>> tool_frames_;
  mutable std::mutex frame_mutex_;
  std::map<std::string, KDL::Frame> frame_poses_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

}  // namespace humanoid_motion_server::tf

#endif  // HUMANOID_MOTION_SERVER__TF__TF_RUNTIME_HPP_
