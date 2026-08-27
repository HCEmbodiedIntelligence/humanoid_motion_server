#include <gtest/gtest.h>

#include <dlfcn.h>

#include <cmath>
#include <string>

#include "motion_control/rtc.hpp"

#ifndef HUMANOID_MOTION_CONTROL_SDK_LIBRARY
#error "HUMANOID_MOTION_CONTROL_SDK_LIBRARY must identify libmotion_control.so"
#endif

namespace
{

TEST(SdkSharedObjectIntegration, RealRtcShapesJointTarget)
{
  void * library = dlopen(HUMANOID_MOTION_CONTROL_SDK_LIBRARY, RTLD_NOW | RTLD_LOCAL);
  ASSERT_NE(library, nullptr) << "real SDK runtime load failed: " << dlerror();

  using Constructor = void (*)(motion_control::Rtc *);
  using Destructor = void (*)(motion_control::Rtc *);
  using Configure = void (*)(motion_control::Rtc *, const motion_control::RtcConfig &);
  using Reset = void (*)(motion_control::Rtc *, const motion_control::RtcJointTarget &);
  using Update = motion_control::RtcJointState (*)(
    motion_control::Rtc *, const motion_control::RtcJointTarget &, double);

  const auto constructor = reinterpret_cast<Constructor>(
    dlsym(library, "_ZN14motion_control3RtcC1Ev"));
  const auto destructor = reinterpret_cast<Destructor>(
    dlsym(library, "_ZN14motion_control3RtcD1Ev"));
  const auto configure = reinterpret_cast<Configure>(
    dlsym(library, "_ZN14motion_control3Rtc9configureERKNS_9RtcConfigE"));
  const auto reset = reinterpret_cast<Reset>(
    dlsym(library, "_ZN14motion_control3Rtc16resetJointTargetERKNS_14RtcJointTargetE"));
  const auto update = reinterpret_cast<Update>(
    dlsym(library, "_ZN14motion_control3Rtc17updateJointTargetERKNS_14RtcJointTargetEd"));
  ASSERT_NE(constructor, nullptr);
  ASSERT_NE(destructor, nullptr);
  ASSERT_NE(configure, nullptr);
  ASSERT_NE(reset, nullptr);
  ASSERT_NE(update, nullptr);

  alignas(motion_control::Rtc) unsigned char storage[sizeof(motion_control::Rtc)];
  auto * rtc = reinterpret_cast<motion_control::Rtc *>(storage);
  constructor(rtc);

  motion_control::RtcConfig config;
  config.input_cutoff_hz = 10.0;
  config.output_rate_hz = 100.0;
  config.max_velocity = 180.0;
  config.max_acceleration = 360.0;
  config.max_jerk = 1000.0;
  configure(rtc, config);

  motion_control::RtcJointTarget current;
  current.joint_names = {"joint"};
  current.positions = {0.0};
  current.velocities = {0.0};
  current.accelerations = {0.0};
  reset(rtc, current);

  auto target = current;
  target.positions[0] = 90.0;
  const auto result = update(rtc, target, 0.01);
  EXPECT_TRUE(
    result.status == motion_control::RtcStatus::kRunning ||
    result.status == motion_control::RtcStatus::kReached) << result.message;
  ASSERT_EQ(result.joint_names, current.joint_names);
  ASSERT_EQ(result.positions.size(), 1U);
  EXPECT_TRUE(std::isfinite(result.positions[0]));
  EXPECT_GE(result.positions[0], 0.0);
  EXPECT_LE(result.positions[0], 90.0);

  destructor(rtc);
  dlclose(library);
}

}  // namespace
