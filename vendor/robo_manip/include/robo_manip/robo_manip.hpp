#pragma once

#include "motion_control.hpp"
#include "robo_manip/core/motion_context.hpp"
#include "robo_manip/core/system_init.hpp"
#include "robo_manip/execution/execution.hpp"
#include "robo_manip/kinematics/kinematics.hpp"
#include "robo_manip/tasks/common/basic_task.hpp"
#include "robo_manip/tasks/contact/contact.hpp"
#include "robo_manip/tasks/dual_arm_coop/dual_arm_coop.hpp"
#include "robo_manip/tasks/jog/jog.hpp"
#include "robo_manip/tasks/move_line/move_line.hpp"
#include "robo_manip/tasks/move_path/move_path.hpp"
#include "robo_manip/tasks/move_joint/move_joint.hpp"
#include "robo_manip/tasks/servo_track/servo_track.hpp"

namespace robo_manip {
namespace types = ::motion_control::types;
}  // namespace robo_manip
