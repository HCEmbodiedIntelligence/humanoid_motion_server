#pragma once

#include <string>
#include <utility>
#include <vector>

#include "types/geometry.hpp"

namespace motion_control::types {

/// 关节状态，positions/velocities/efforts 与 group_name 对应的关节序列按下标对应。
struct JointState {
  std::string group_name;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

/// 移动底盘状态。
struct MobileBaseState {
  Pose pose{};
  Twist velocity{};
};

/// 单个末端执行器状态。
struct EndEffectorState {
  std::string name;
  Pose pose{};
  Twist twist{};
  Wrench wrench{};
};

/// 机器人整体状态快照。
struct RobotState {
  /// 状态时间戳，单位 s。
  double stamp_sec{0.0};
  MobileBaseState base{};
  std::vector<JointState> joint_groups;
  std::vector<EndEffectorState> end_effectors;

  JointState& addJointGroup(std::string group_name) {
    joint_groups.push_back({});
    joint_groups.back().group_name = std::move(group_name);
    return joint_groups.back();
  }
};

}  // 命名空间 motion_control::types
