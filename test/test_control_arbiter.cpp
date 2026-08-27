#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "humanoid_motion_server/arbitration/control_arbiter.hpp"

namespace hmc = humanoid_motion_server::motion;
using namespace std::chrono_literals;

namespace
{

hmc::SteadyTime time_ms(const int milliseconds)
{
  return hmc::SteadyTime{} + std::chrono::milliseconds(milliseconds);
}

hmc::CommandClaim claim(
  const std::string & id, const std::string & endpoint,
  const std::string & group, const hmc::MotionKind kind,
  std::vector<std::string> joints)
{
  return {id, endpoint, group, kind, std::move(joints)};
}

bool has(const std::vector<std::string> & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

TEST(ControlArbiter, HighPriorityPreemptsMoveAndMoveNeverReturns)
{
  hmc::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"safety", hmc::MotionKind::SERVO_J, "left", 20}).ok());

  EXPECT_TRUE(
    arbiter.submitMove(
      claim("move-1", "move", "left", hmc::MotionKind::MOVE_J, {"l1", "l2"}),
      time_ms(0)).admitted);
  const auto safety = arbiter.updateServo(
    claim("servo:safety", "safety", "left", hmc::MotionKind::SERVO_J, {"l1", "l2"}),
    time_ms(1));
  ASSERT_EQ(safety.preempted_move_ids, std::vector<std::string>({"move-1"}));
  EXPECT_FALSE(arbiter.contains("move-1"));

  const auto expired = arbiter.evaluate(time_ms(101));
  EXPECT_FALSE(has(expired.winner_session_ids, "move-1"));
}

TEST(ControlArbiter, SamePriorityUsesLatestServerReceiveSequence)
{
  hmc::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint({"a", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"b", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  arbiter.updateServo(
    claim("servo:a", "a", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(0));
  auto latest = arbiter.updateServo(
    claim("servo:b", "b", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(0));
  ASSERT_EQ(latest.winner_session_ids, std::vector<std::string>({"servo:b"}));

  latest = arbiter.updateServo(
    claim("servo:a", "a", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(0));
  EXPECT_EQ(latest.winner_session_ids, std::vector<std::string>({"servo:a"}));
}

TEST(ControlArbiter, DefaultLeaseExpiresAtOneHundredMillisecondsAndFallsBack)
{
  hmc::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint({"low", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"high", hmc::MotionKind::SERVO_J, "left", 20}).ok());
  arbiter.updateServo(
    claim("servo:high", "high", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(0));
  arbiter.updateServo(
    claim("servo:low", "low", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(50));

  const auto before = arbiter.evaluate(time_ms(99));
  EXPECT_EQ(before.winner_session_ids, std::vector<std::string>({"servo:high"}));
  const auto at_lease = arbiter.evaluate(time_ms(100));
  EXPECT_TRUE(has(at_lease.expired_servo_ids, "servo:high"));
  EXPECT_EQ(at_lease.winner_session_ids, std::vector<std::string>({"servo:low"}));
}

TEST(ControlArbiter, LowPriorityMoveIsRejectedWithoutQueueing)
{
  hmc::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint({"servo", hmc::MotionKind::SERVO_J, "left", 20}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  arbiter.updateServo(
    claim("servo:servo", "servo", "left", hmc::MotionKind::SERVO_J, {"l1"}), time_ms(0));
  const auto move = arbiter.submitMove(
    claim("move-1", "move", "left", hmc::MotionKind::MOVE_J, {"l1"}), time_ms(1));
  EXPECT_FALSE(move.admitted);
  EXPECT_EQ(move.status.code, hmc::StatusCode::REJECTED);
  EXPECT_FALSE(arbiter.contains("move-1"));
}

TEST(ControlArbiter, DisjointArmsRunConcurrentlyAndWholeBodyConflicts)
{
  hmc::ControlArbiter arbiter;
  ASSERT_TRUE(arbiter.registerEndpoint({"left", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"right", hmc::MotionKind::MOVE_J, "right", 10}).ok());
  ASSERT_TRUE(arbiter.registerEndpoint({"whole", hmc::MotionKind::SERVO_J, "whole", 20}).ok());
  arbiter.submitMove(
    claim("left-1", "left", "left", hmc::MotionKind::MOVE_J, {"l1", "l2"}), time_ms(0));
  const auto right = arbiter.submitMove(
    claim("right-1", "right", "right", hmc::MotionKind::MOVE_J, {"r1", "r2"}), time_ms(1));
  EXPECT_TRUE(has(right.winner_session_ids, "left-1"));
  EXPECT_TRUE(has(right.winner_session_ids, "right-1"));

  const auto whole = arbiter.updateServo(
    claim(
      "servo:whole", "whole", "whole", hmc::MotionKind::SERVO_J,
      {"l1", "l2", "r1", "r2"}), time_ms(2));
  EXPECT_TRUE(has(whole.preempted_move_ids, "left-1"));
  EXPECT_TRUE(has(whole.preempted_move_ids, "right-1"));
  EXPECT_EQ(whole.winner_session_ids, std::vector<std::string>({"servo:whole"}));
}

}  // namespace
