#include <gtest/gtest.h>
#include "humanoid_motion_server/motion/servo_state_log.hpp"

namespace m = humanoid_motion_server::motion;
using namespace std::chrono_literals;

TEST(ServoStateLog, BoundaryRecoveryStartAndCompletionLogOnce)
{
  m::ServoStateLog log;
  m::PipelineTick tick;
  tick.events = {{"servo:right", m::SessionState::RUNNING,
    {m::StatusCode::OK, "joint limit recovery started"}}};
  const auto started = log.update(tick, m::SteadyTime{});
  ASSERT_EQ(started.size(), 1U);
  EXPECT_NE(started.front().find("recovery started"), std::string::npos);
  EXPECT_TRUE(log.update(tick, m::SteadyTime{} + 2s).empty());
  tick.events.front().status.message = "joint limit recovery completed";
  const auto completed = log.update(tick, m::SteadyTime{} + 3s);
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_NE(completed.front().find("recovery completed"), std::string::npos);
  EXPECT_TRUE(log.update(tick, m::SteadyTime{} + 5s).empty());
}

TEST(ServoStateLog, RepeatedFailuresDoNotLogEveryRetryAndRecoveryIsLogged)
{
  m::ServoStateLog log;
  m::PipelineTick failed;
  failed.events = {
    {"servo:right", m::SessionState::RUNNING, m::MotionStatus::Ok()},
    {"servo:right", m::SessionState::ABORTED,
      {m::StatusCode::LIMIT_VIOLATION, "SDK command violates model limit: elbow"}}};
  auto first = log.update(failed, m::SteadyTime{});
  ASSERT_EQ(first.size(), 1U);
  EXPECT_NE(first.front().find("elbow"), std::string::npos);
  EXPECT_EQ(first.front().find(" -> running"), std::string::npos);
  for (int i = 1; i < 400; ++i) {
    EXPECT_TRUE(log.update(failed, m::SteadyTime{} + i * 10ms).empty());
  }
  m::PipelineTick recovered;
  recovered.events = {{"servo:right", m::SessionState::RUNNING, m::MotionStatus::Ok()}};
  auto next = log.update(recovered, m::SteadyTime{} + 4s);
  ASSERT_EQ(next.size(), 1U);
  EXPECT_NE(next.front().find(" -> running"), std::string::npos);
  EXPECT_TRUE(log.update(recovered, m::SteadyTime{} + 8s).empty());
}

TEST(ServoStateLog, ShortTransitionsAreRetainedAndRateLimitedPerArm)
{
  m::ServoStateLog log;
  m::PipelineTick tick;
  tick.events = {{"servo:left", m::SessionState::RUNNING, m::MotionStatus::Ok()}};
  ASSERT_EQ(log.update(tick, m::SteadyTime{}).size(), 1U);
  tick.events = {{"servo:left", m::SessionState::ABORTED,
      {m::StatusCode::SDK_ERROR, "IK failed", "SDK::Tick", -1}}};
  EXPECT_TRUE(log.update(tick, m::SteadyTime{} + 10ms).empty());
  tick.events = {{"servo:left", m::SessionState::RUNNING, m::MotionStatus::Ok()},
    {"servo:right", m::SessionState::RUNNING, m::MotionStatus::Ok()}};
  auto other = log.update(tick, m::SteadyTime{} + 20ms);
  ASSERT_EQ(other.size(), 1U);
  EXPECT_NE(other.front().find("servo:right"), std::string::npos);
  auto pending = log.update({}, m::SteadyTime{} + 1s);
  ASSERT_EQ(pending.size(), 1U);
  EXPECT_NE(pending.front().find("IK failed"), std::string::npos);
  EXPECT_NE(pending.front().find("SDK::Tick"), std::string::npos);
  EXPECT_NE(pending.front().find(" -> running; changes=2"), std::string::npos);
  EXPECT_TRUE(log.update({}, m::SteadyTime{} + 10s).empty());
}
