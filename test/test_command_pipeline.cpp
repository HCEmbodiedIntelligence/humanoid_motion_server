#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "fake_sdk_motion_backend.hpp"
#include "humanoid_motion_server/motion/command_pipeline.hpp"

namespace hmc = humanoid_motion_server::motion;
namespace hmct = humanoid_motion_server::motion::test;
using namespace std::chrono_literals;

namespace
{

hmc::SteadyTime time_ms(const int value)
{
  return hmc::SteadyTime{} + std::chrono::milliseconds(value);
}

hmc::JointGroupModel left_group()
{
  return {"left", {"l1", "l2"}, {-2.0, -2.0}, {2.0, 2.0}};
}

std::vector<hmc::JointGroupModel> all_groups()
{
  return {
    left_group(),
    {"right", {"r1", "r2"}, {-2.0, -2.0}, {2.0, 2.0}},
    {"whole", {"l1", "l2", "r1", "r2"},
      {-2.0, -2.0, -2.0, -2.0}, {2.0, 2.0, 2.0, 2.0}}};
}

hmc::JointFeedback feedback(const int received_ms = 0)
{
  return {
    {"l1", "l2", "r1", "r2"},
    {0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0}, time_ms(received_ms)};
}

hmc::JointTarget left_target(const double value)
{
  return {{"l1", "l2"}, {value, -value}, {}, {}};
}

hmc::Pose pose(const double x)
{
  hmc::Pose result;
  result.position_m[0] = x;
  return result;
}

bool has_event(
  const hmc::PipelineTick & tick, const std::string & id,
  const hmc::SessionState state)
{
  return std::any_of(
    tick.events.begin(), tick.events.end(),
    [&](const hmc::SessionEvent & event) {
      return event.session_id == id && event.state == state;
    });
}

TEST(CommandPipeline, MoveJMoveLAndMovePUseStartTickAndMandatoryFinalRtc)
{
  const std::vector<hmc::SessionRequest> requests{
    hmc::MoveJRequest{"move-j", "left", left_target(0.2), {}},
    hmc::MoveLRequest{"move-l", "left", "base", "tool", pose(0.2), {}},
    hmc::MovePRequest{"move-p", "left", "base", "tool", {pose(0.1), pose(0.2)}, {}}};
  const std::vector<hmc::MotionKind> kinds{
    hmc::MotionKind::MOVE_J, hmc::MotionKind::MOVE_L, hmc::MotionKind::MOVE_P};

  for (std::size_t index = 0; index < requests.size(); ++index) {
    auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
    hmc::CommandPipeline pipeline(backend, all_groups());
    const std::string endpoint = "move-" + std::to_string(index);
    ASSERT_TRUE(pipeline.registerEndpoint({endpoint, kinds[index], "left", 10}).ok());
    const auto submitted = pipeline.submitMove(endpoint, requests[index], time_ms(0));
    ASSERT_TRUE(submitted.status.ok());

    const auto output = pipeline.tick(feedback(1), time_ms(1));
    ASSERT_EQ(output.commands.size(), 1U);
    EXPECT_TRUE(output.commands.front().passed_final_sdk_rtc);
    EXPECT_EQ(backend->start_kinds.at(submitted.session_id), kinds[index]);
    EXPECT_EQ(backend->start_calls.at(submitted.session_id), 1);
    EXPECT_EQ(backend->tick_calls.at(submitted.session_id), 1);
    EXPECT_EQ(backend->final_rtc_calls.at("left"), 1);
    EXPECT_EQ(backend->reset_calls.at("left"), 1);
  }
}

TEST(CommandPipeline, ServoJAndServoPDynamicallyRetargetWithoutRestart)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(
    pipeline.registerEndpoint(
      {"servo-j", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  ASSERT_TRUE(
    pipeline.updateServo(
      "servo-j", hmc::ServoJRequest{"", "left", left_target(0.1), {}},
      time_ms(0)).status.ok());
  pipeline.tick(feedback(1), time_ms(1));
  ASSERT_TRUE(
    pipeline.updateServo(
      "servo-j", hmc::ServoJRequest{"", "left", left_target(0.4), {}},
      time_ms(2)).status.ok());
  pipeline.tick(feedback(3), time_ms(3));

  ASSERT_EQ(backend->start_calls.at("servo:servo-j"), 1);
  ASSERT_EQ(backend->dynamic_targets.at("servo:servo-j").size(), 2U);
  EXPECT_DOUBLE_EQ(
    std::get<hmc::JointTarget>(
      backend->dynamic_targets.at(
        "servo:servo-j").back()).positions_rad[0],
    0.4);

  auto pose_backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pose_pipeline(pose_backend, all_groups());
  ASSERT_TRUE(
    pose_pipeline.registerEndpoint(
      {"servo-p", hmc::MotionKind::SERVO_P, "left", 10}).ok());
  pose_pipeline.updateServo(
    "servo-p", hmc::ServoPRequest{"", "left", "base", "tool", pose(0.1), {}},
    time_ms(0));
  pose_pipeline.tick(feedback(1), time_ms(1));
  pose_pipeline.updateServo(
    "servo-p", hmc::ServoPRequest{"", "left", "base", "tool", pose(0.3), {}},
    time_ms(2));
  pose_pipeline.tick(feedback(3), time_ms(3));
  ASSERT_EQ(pose_backend->start_calls.at("servo:servo-p"), 1);
  EXPECT_DOUBLE_EQ(
    std::get<hmc::Pose>(pose_backend->dynamic_targets.at("servo:servo-p").back()).position_m[0],
    0.3);
}

TEST(CommandPipeline, UsesSteadyClockElapsedControlPeriodAfterNominalFirstTick)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(
    pipeline.registerEndpoint(
      {"servo", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  pipeline.updateServo(
    "servo", hmc::ServoJRequest{"", "left", left_target(0.2), {}}, time_ms(0));
  pipeline.tick(feedback(1), time_ms(1));
  pipeline.updateServo(
    "servo", hmc::ServoJRequest{"", "left", left_target(0.3), {}}, time_ms(15));
  pipeline.tick(feedback(16), time_ms(16));

  ASSERT_EQ(backend->tick_periods.at("servo:servo").size(), 2U);
  EXPECT_NEAR(backend->tick_periods.at("servo:servo")[0], 0.01, 1e-12);
  EXPECT_NEAR(backend->tick_periods.at("servo:servo")[1], 0.015, 1e-12);
  EXPECT_NEAR(backend->final_periods.at("left")[1], 0.015, 1e-12);
}

TEST(CommandPipeline, FinalRtcCannotBeBypassed)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  backend->bypass_final_rtc = true;
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(pipeline.registerEndpoint({"servo", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  pipeline.updateServo(
    "servo", hmc::ServoJRequest{"", "left", left_target(0.2), {}}, time_ms(0));
  const auto output = pipeline.tick(feedback(1), time_ms(1));
  EXPECT_TRUE(output.commands.empty());
  EXPECT_TRUE(has_event(output, "servo:servo", hmc::SessionState::ABORTED));
  EXPECT_GE(backend->final_rtc_calls.at("left"), 2);
}

TEST(CommandPipeline, HighServoPreemptsMoveAndMoveDoesNotResume)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  ASSERT_TRUE(pipeline.registerEndpoint({"safety", hmc::MotionKind::SERVO_J, "left", 20}).ok());
  pipeline.submitMove(
    "move", hmc::MoveJRequest{"move-1", "left", left_target(0.5), {}}, time_ms(0));
  pipeline.tick(feedback(1), time_ms(1));
  pipeline.updateServo(
    "safety", hmc::ServoJRequest{"", "left", left_target(0.1), {}}, time_ms(2));
  const auto preempted = pipeline.tick(feedback(3), time_ms(3));
  EXPECT_TRUE(has_event(preempted, "move-1", hmc::SessionState::PREEMPTED));
  EXPECT_EQ(backend->stop_calls.at("move-1"), 1);

  pipeline.tick(feedback(102), time_ms(102));
  EXPECT_EQ(backend->start_calls.at("move-1"), 1);
}

TEST(CommandPipeline, ExpiredHighServoFallsBackAndRestartsLowFromRealFeedback)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(pipeline.registerEndpoint({"low", hmc::MotionKind::SERVO_J, "left", 10}).ok());
  ASSERT_TRUE(pipeline.registerEndpoint({"high", hmc::MotionKind::SERVO_J, "left", 20}).ok());
  pipeline.updateServo(
    "low", hmc::ServoJRequest{"", "left", left_target(0.1), {}}, time_ms(0));
  pipeline.tick(feedback(1), time_ms(1));
  pipeline.updateServo(
    "high", hmc::ServoJRequest{"", "left", left_target(0.2), {}}, time_ms(10));
  pipeline.tick(feedback(11), time_ms(11));
  pipeline.updateServo(
    "low", hmc::ServoJRequest{"", "left", left_target(0.3), {}}, time_ms(50));
  pipeline.tick(feedback(111), time_ms(111));

  EXPECT_EQ(backend->start_calls.at("servo:low"), 2);
  EXPECT_EQ(backend->stop_calls.at("servo:low"), 1);
  EXPECT_GE(backend->reset_calls.at("left"), 3);
  EXPECT_EQ(backend->reset_feedback.at("left").received_at, time_ms(111));
}

TEST(CommandPipeline, DisjointArmsProduceOneCommandEach)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(pipeline.registerEndpoint({"left", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  ASSERT_TRUE(pipeline.registerEndpoint({"right", hmc::MotionKind::MOVE_J, "right", 10}).ok());
  pipeline.submitMove(
    "left", hmc::MoveJRequest{"left-1", "left", left_target(0.2), {}}, time_ms(0));
  hmc::JointTarget right{{"r1", "r2"}, {0.3, -0.3}, {}, {}};
  pipeline.submitMove(
    "right", hmc::MoveJRequest{"right-1", "right", right, {}}, time_ms(0));
  const auto output = pipeline.tick(feedback(1), time_ms(1));
  EXPECT_EQ(output.commands.size(), 2U);
}

TEST(CommandPipeline, MoveSucceedsOnlyAfterRealFeedbackIsStable)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  ASSERT_TRUE(
    pipeline.registerEndpoint(
      {"move", hmc::MotionKind::MOVE_J, "left", 10}).ok());
  pipeline.submitMove(
    "move", hmc::MoveJRequest{"closed-loop", "left", left_target(0.2), {}},
    time_ms(0));
  const auto planned = pipeline.tick(feedback(1), time_ms(1));
  EXPECT_FALSE(has_event(planned, "closed-loop", hmc::SessionState::SUCCEEDED));

  auto at_goal = feedback(2);
  at_goal.positions_rad[0] = 0.2;
  at_goal.positions_rad[1] = -0.2;
  EXPECT_FALSE(
    has_event(
      pipeline.tick(at_goal, time_ms(2)), "closed-loop",
      hmc::SessionState::SUCCEEDED));
  at_goal.received_at = time_ms(202);
  EXPECT_TRUE(
    has_event(
      pipeline.tick(at_goal, time_ms(202)), "closed-loop",
      hmc::SessionState::SUCCEEDED));
}

TEST(CommandPipeline, CancelTimeoutSdkFailureStaleStateAndRepeatedStopAreStructured)
{
  {
    auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
    hmc::CommandPipeline pipeline(backend, all_groups());
    pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10});
    pipeline.submitMove(
      "move", hmc::MoveJRequest{"cancel-me", "left", left_target(0.2), {}}, time_ms(0));
    pipeline.tick(feedback(1), time_ms(1));
    EXPECT_TRUE(pipeline.cancel("cancel-me").ok());
    EXPECT_TRUE(pipeline.cancel("cancel-me").ok());
    EXPECT_EQ(backend->stop_calls.at("cancel-me"), 2);
  }
  {
    auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
    hmc::CommandPipelineConfig config;
    config.goal_monitor.move_timeout = 50ms;
    hmc::CommandPipeline pipeline(backend, all_groups(), config);
    pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10});
    pipeline.submitMove(
      "move", hmc::MoveJRequest{"timeout", "left", left_target(0.2), {}}, time_ms(0));
    pipeline.tick(feedback(1), time_ms(1));
    const auto output = pipeline.tick(feedback(51), time_ms(51));
    EXPECT_TRUE(has_event(output, "timeout", hmc::SessionState::ABORTED));
  }
  {
    auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
    backend->fail_tick_ids.insert("sdk-error");
    hmc::CommandPipeline pipeline(backend, all_groups());
    pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10});
    pipeline.submitMove(
      "move", hmc::MoveJRequest{"sdk-error", "left", left_target(0.2), {}}, time_ms(0));
    const auto output = pipeline.tick(feedback(1), time_ms(1));
    const auto event = std::find_if(
      output.events.begin(), output.events.end(),
      [](const hmc::SessionEvent & value) {
        return value.session_id == "sdk-error" &&
        value.state == hmc::SessionState::ABORTED;
      });
    ASSERT_NE(event, output.events.end());
    EXPECT_EQ(event->status.sdk_api, "FakeSdk::Tick");
    EXPECT_EQ(event->status.sdk_code, 73);
    ASSERT_EQ(output.commands.size(), 1U);
    EXPECT_TRUE(output.commands.front().passed_final_sdk_rtc);
  }
  {
    auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
    hmc::CommandPipeline pipeline(backend, all_groups());
    pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10});
    pipeline.submitMove(
      "move", hmc::MoveJRequest{"stale", "left", left_target(0.2), {}}, time_ms(0));
    const auto output = pipeline.tick(feedback(0), time_ms(100));
    const auto event = std::find_if(
      output.events.begin(), output.events.end(),
      [](const hmc::SessionEvent & value) {return value.session_id == "stale";});
    ASSERT_NE(event, output.events.end());
    EXPECT_EQ(event->status.code, hmc::StatusCode::STALE_FEEDBACK);
    ASSERT_EQ(output.commands.size(), 1U);
    EXPECT_TRUE(output.commands.front().passed_final_sdk_rtc);
  }
}

TEST(CommandPipeline, RejectsTargetOutsideModelLimitBeforeSdk)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  hmc::CommandPipeline pipeline(backend, all_groups());
  pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_J, "left", 10});
  const auto result = pipeline.submitMove(
    "move", hmc::MoveJRequest{"bad", "left", left_target(2.1), {}}, time_ms(0));
  EXPECT_EQ(result.status.code, hmc::StatusCode::LIMIT_VIOLATION);
  EXPECT_TRUE(backend->start_calls.empty());
}

TEST(CommandPipeline, RejectsSdkCandidateOutsideModelLimitAndPublishesRtcHold)
{
  auto backend = std::make_shared<hmct::FakeSdkMotionBackend>();
  backend->candidate_overrides["left"] = {
    "left", {"l1", "l2"}, {3.0, 0.0}, {0.0, 0.0}, {}, false};
  hmc::CommandPipeline pipeline(backend, all_groups());
  pipeline.registerEndpoint({"move", hmc::MotionKind::MOVE_L, "left", 10});
  pipeline.submitMove(
    "move", hmc::MoveLRequest{
      "bad-sdk-output", "left", "base", "tool", pose(0.2), {}},
    time_ms(0));
  const auto output = pipeline.tick(feedback(1), time_ms(1));
  const auto event = std::find_if(
    output.events.begin(), output.events.end(),
    [](const hmc::SessionEvent & value) {
      return value.session_id == "bad-sdk-output" &&
      value.state == hmc::SessionState::ABORTED;
    });
  ASSERT_NE(event, output.events.end());
  EXPECT_EQ(event->status.code, hmc::StatusCode::LIMIT_VIOLATION);
  ASSERT_EQ(output.commands.size(), 1U);
  EXPECT_TRUE(output.commands.front().passed_final_sdk_rtc);
  EXPECT_EQ(output.commands.front().positions_rad, std::vector<double>({0.0, 0.0}));
}

}  // namespace
