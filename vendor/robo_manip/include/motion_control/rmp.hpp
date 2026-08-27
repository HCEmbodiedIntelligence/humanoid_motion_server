#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "core_module.hpp"
#include "types/robot_state.hpp"
#include "types/task.hpp"

namespace motion_control {

class Rkd;

struct RmpConfig {
  /// Fallback limits used when a request does not provide its own limits and
  /// no group-specific limits are configured.
  types::MotionLimits default_limits{};
  /// Optional limits keyed by PlanningRequest::group_name.
  std::unordered_map<std::string, types::MotionLimits> group_limits;
};

/// 基于几何路径和运动约束生成时间参数化轨迹的规划模块。
class Rmp : public CoreModule {
 public:
  std::string name() const override { return "RMP"; }

  void configure(RmpConfig config);
  const RmpConfig& config() const;

  /// 按“请求值 > group 配置 > 全局配置”的优先级解析最终运动限制。
  types::MotionLimits resolveMotionLimits(
      const std::string& group_name,
      const types::MotionLimits& request_limits) const;

  /// 设置用于 FK、IK 和雅可比计算的运动学模块。
  void setKinematics(std::shared_ptr<const Rkd> rkd);

  /// 对关节空间路径进行时间参数化。
  virtual types::PlanningResult planJointPath(const types::RobotState& state,
                                              const types::PlanningRequest& request) const;
  /// 对笛卡尔路径做 IK 离散化，并生成关节空间时间参数化轨迹。
  /// PlaCo 首次求解失败时，会使用 soft FrameTask、低姿态权重、正则与
  /// 可操作度任务，从初始状态重新计算整段路径一次；重试结果不检查
  /// 位置/姿态容差。
  virtual types::PlanningResult planCartesianPath(const types::RobotState& state,
                                                  const types::PlanningRequest& request) const;

 private:
  std::shared_ptr<const Rkd> rkd_;
  RmpConfig config_;
};

}  // 命名空间 motion_control
