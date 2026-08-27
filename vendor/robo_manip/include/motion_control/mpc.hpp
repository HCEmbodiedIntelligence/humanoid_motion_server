#pragma once

#include "core_module.hpp"
#include "types/robot_state.hpp"
#include "types/task.hpp"

namespace motion_control {

/// 短时域模型预测控制模块。
class Mpc : public CoreModule {
 public:
  std::string name() const override { return "MPC"; }

  /// 基于当前状态和规划请求求解短时域优化结果。
  virtual types::PlanningResult optimizeShortHorizon(const types::RobotState& state,
                                                     const types::PlanningRequest& request) const;
};

}  // 命名空间 motion_control
