#pragma once

#include "robo_manip/tasks/common/basic_task.hpp"

namespace robo_manip::tasks {

class Contact : public BasicTask {
 public:
  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;
};

}  // namespace robo_manip::tasks
