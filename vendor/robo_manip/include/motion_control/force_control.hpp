#pragma once

#include "core_module.hpp"
#include "types/geometry.hpp"
#include "types/task.hpp"

namespace motion_control {

/// 导纳/顺应控制参数。
struct ComplianceConfig {
  /// 虚拟刚度。
  double stiffness{0.0};
  /// 虚拟阻尼。
  double damping{0.0};
  /// 虚拟质量。
  double virtual_mass{1.0};
};

/// 根据期望力和实际力修正末端目标的力控模块。
class ForceControl : public CoreModule {
 public:
  std::string name() const override { return "ForceControl"; }

  /// 对 nominal_target 应用导纳修正，返回新的末端目标。
  virtual types::Target applyAdmittance(const types::Target& nominal_target,
                                        const types::Wrench& actual_wrench,
                                        const types::Wrench& desired_wrench,
                                        double dt_sec) const;
};

}  // 命名空间 motion_control
