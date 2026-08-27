#pragma once

#include <string>

#include "robo_manip/core/motion_context.hpp"

namespace robo_manip::core {

struct SystemInitResult {
  bool success{false};
  std::string message;
  MotionContext context;
};

// Loads configuration and constructs the motion context without opening robot
// hardware connections. If execution is configured, the returned robot_driver
// and trajectory_executor are ready to use, but callers that need live state or
// command execution must call context.robot_driver->connect() explicitly first.
SystemInitResult InitializeSystemFromFile(const std::string& config_path);

}  // namespace robo_manip::core
