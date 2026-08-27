#include <string>

#include "motion_control/rkd.hpp"
#include "robo_manip/core/system_init.hpp"

int main() {
  motion_control::Rkd rkd;
  using InitializeFunction = robo_manip::core::SystemInitResult (*)(
      const std::string&);
  InitializeFunction volatile initialize =
      &robo_manip::core::InitializeSystemFromFile;
  return rkd.name() == "RKD" && initialize != nullptr ? 0 : 1;
}
