#include <gtest/gtest.h>

#include <chrono>

#include "humanoid_motion_server/motion/goal_monitor.hpp"

namespace hmc = humanoid_motion_server::motion;
using namespace std::chrono_literals;

namespace
{

hmc::SteadyTime time_ms(const int value)
{
  return hmc::SteadyTime{} + std::chrono::milliseconds(value);
}

hmc::JointTarget goal()
{
  return {{"j1", "j2"}, {0.5, -0.5}, {}, {}};
}

hmc::JointFeedback feedback(
  const double p1, const double p2, const double v1, const double v2)
{
  return {{"j2", "j1"}, {p2, p1}, {v2, v1}, time_ms(0)};
}

TEST(GoalMonitor, RequiresPositionVelocityAndContinuousStableDuration)
{
  hmc::GoalMonitor monitor;
  ASSERT_TRUE(monitor.begin("move", goal(), std::nullopt, time_ms(0)).ok());

  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), std::nullopt, time_ms(0)).observation,
    hmc::GoalObservation::RUNNING);
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.03, 0.0), std::nullopt, time_ms(100)).observation,
    hmc::GoalObservation::RUNNING);
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), std::nullopt, time_ms(150)).observation,
    hmc::GoalObservation::RUNNING);
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), std::nullopt, time_ms(350)).observation,
    hmc::GoalObservation::REACHED);
}

TEST(GoalMonitor, CartesianMoveAlsoRequiresOriginalFkTolerance)
{
  hmc::GoalMonitor monitor;
  hmc::Pose target;
  target.position_m = {0.1, 0.2, 0.3};
  ASSERT_TRUE(monitor.begin("move", goal(), target, time_ms(0)).ok());

  hmc::Pose outside = target;
  outside.position_m[0] += 0.0011;
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), outside, time_ms(0)).observation,
    hmc::GoalObservation::RUNNING);
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), target, time_ms(10)).observation,
    hmc::GoalObservation::RUNNING);
  EXPECT_EQ(
    monitor.observe("move", feedback(0.5, -0.5, 0.0, 0.0), target, time_ms(210)).observation,
    hmc::GoalObservation::REACHED);
}

TEST(GoalMonitor, TimesOutFromSteadyClock)
{
  hmc::GoalMonitor monitor;
  ASSERT_TRUE(monitor.begin("move", goal(), std::nullopt, time_ms(0)).ok());
  const auto result = monitor.observe(
    "move", feedback(0.0, 0.0, 0.0, 0.0), std::nullopt,
    time_ms(60001));
  EXPECT_EQ(result.observation, hmc::GoalObservation::TIMED_OUT);
  EXPECT_EQ(result.status.code, hmc::StatusCode::TIMEOUT);
}

TEST(GoalMonitor, RejectsMissingOrNonFiniteRealFeedback)
{
  hmc::GoalMonitor monitor;
  ASSERT_TRUE(monitor.begin("move", goal(), std::nullopt, time_ms(0)).ok());
  hmc::JointFeedback bad{{"j1"}, {0.5}, {0.0}, time_ms(0)};
  const auto result = monitor.observe("move", bad, std::nullopt, time_ms(1));
  EXPECT_EQ(result.observation, hmc::GoalObservation::INVALID_FEEDBACK);
}

}  // namespace
