#pragma once

#include "robo_manip/core/motion_context.hpp"
#include "types/robot_state.hpp"
#include "types/task.hpp"

namespace robo_manip::tasks {

class BasicTask {
 public:
  virtual ~BasicTask() = default;

  virtual motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) = 0;
};

}  // namespace robo_manip::tasks
