#pragma once

#include <string>
#include <vector>

#include "types/geometry.hpp"

namespace motion_control::types {

/// 关节轨迹上的单个采样点。
struct JointTrajectoryPoint {
  /// 相对轨迹起点的时间，单位 s。
  double time_from_start_sec{0.0};
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

/// 关节空间轨迹，所有点的向量长度应与 joint_names 一致。
struct JointTrajectory {
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

/// 笛卡尔轨迹上的单个采样点。
struct CartesianTrajectoryPoint {
  /// 相对轨迹起点的时间，单位 s。
  double time_from_start_sec{0.0};
  Pose pose{};
  Twist twist{};
};

/// 笛卡尔空间轨迹。
struct CartesianTrajectory {
  /// 这些 Cartesian pose 表示的 link/frame，例如 tool0。
  std::string link_name;
  /// pose 的参考坐标系，也是 IK 链起点。
  std::string base_link{"base_link"};
  std::vector<CartesianTrajectoryPoint> points;
};

}  // 命名空间 motion_control::types
