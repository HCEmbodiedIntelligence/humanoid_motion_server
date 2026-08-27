#pragma once

namespace motion_control::types {

/// 三维向量，常用于位置、线速度、力等量。
struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// 单位四元数，默认表示无旋转。
/// 对外接口请优先使用 Pose::euler_zyx_deg；该字段主要作为内部插值、
/// 旋转矩阵转换和历史兼容缓存使用。
struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

/// 纯几何位姿；参考坐标系由外层请求/状态中的 base_link 描述。
struct Pose {
  Vector3 position{};
  Quaternion orientation{};
  /// 用户传入/读取姿态使用 ZYX 欧拉角，单位 deg；
  /// x/y/z 分别为 roll/pitch/yaw，
  /// 旋转矩阵约定为 R = Rz(yaw) * Ry(pitch) * Rx(roll)。
  Vector3 euler_zyx_deg{};
};

/// 空间速度，linear 为线速度，angular 为角速度。
struct Twist {
  Vector3 linear{};
  Vector3 angular{};
};

/// 空间力，force 为力，torque 为力矩。
struct Wrench {
  Vector3 force{};
  Vector3 torque{};
};

}  // 命名空间 motion_control::types
