#include <gtest/gtest.h>

#include "humanoid_motion_server/motion/input_timestamp.hpp"

namespace m = humanoid_motion_server::motion;
using namespace std::chrono_literals;

TEST(InputTimestamp, SourceAgeConsumesLeaseAndFutureStampCannotExtendIt)
{
  m::InputTimestamp input;
  m::SteadyTime fresh;
  std::string error;
  const auto received = m::SteadyTime{} + 10s;
  EXPECT_TRUE(input.accept(950000000, 1000000000, received, 100ms, 10ms, true, fresh, error));
  EXPECT_EQ(fresh, received - 50ms);
  EXPECT_TRUE(input.accept(1010000000, 1000000000, received, 100ms, 10ms, true, fresh, error));
  EXPECT_EQ(fresh, received);
  EXPECT_FALSE(input.accept(1100000000, 1000000000, received, 100ms, 10ms, true, fresh, error));
}

TEST(InputTimestamp, ReplayOutOfOrderAndStaleFeedbackDoNotRefreshAcceptedTime)
{
  m::InputTimestamp input;
  m::SteadyTime fresh;
  std::string error;
  const auto received = m::SteadyTime{} + 10s;
  ASSERT_TRUE(input.accept(950000000, 1000000000, received, 100ms, 10ms, true, fresh, error));
  const auto accepted = fresh;
  for (const auto stamp : {950000000, 940000000, 900000000, 0, -1}) {
    EXPECT_FALSE(input.accept(stamp, 1000000000, received + 1ms, 100ms, 10ms, true, fresh, error));
    EXPECT_EQ(fresh, accepted);
  }
  EXPECT_TRUE(input.accept(990000000, 1000000000, received + 1ms, 100ms, 10ms, true, fresh, error));
}

TEST(InputTimestamp, LegacyServoAndRosClockResetAreExplicitlySupported)
{
  m::InputTimestamp input;
  m::SteadyTime fresh;
  std::string error;
  const auto received = m::SteadyTime{} + 10s;
  EXPECT_TRUE(input.accept(0, 1000000000, received, 100ms, 10ms, false, fresh, error));
  EXPECT_EQ(fresh, received);
  EXPECT_TRUE(input.accept(950000000, 1000000000, received, 100ms, 10ms, true, fresh, error));
  EXPECT_TRUE(input.accept(450000000, 500000000, received + 1ms, 100ms, 10ms, true, fresh, error));
  EXPECT_EQ(fresh, received + 1ms - 50ms);
}
