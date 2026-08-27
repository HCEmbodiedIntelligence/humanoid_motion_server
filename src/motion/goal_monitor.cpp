#include "humanoid_motion_server/motion/goal_monitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <utility>

namespace humanoid_motion_server::motion
{
namespace
{

bool finite_pose(const Pose & pose)
{
  const bool values_finite = std::all_of(
    pose.position_m.begin(), pose.position_m.end(),
    [](const double value) {return std::isfinite(value);}) &&
    std::all_of(
    pose.orientation_xyzw.begin(), pose.orientation_xyzw.end(),
    [](const double value) {return std::isfinite(value);});
  double quaternion_norm_squared = 0.0;
  for (const auto value : pose.orientation_xyzw) {
    quaternion_norm_squared += value * value;
  }
  return values_finite && quaternion_norm_squared > 1e-12;
}

double orientation_distance(const Pose & left, const Pose & right)
{
  double left_norm_sq = 0.0;
  double right_norm_sq = 0.0;
  double dot = 0.0;
  for (std::size_t index = 0; index < 4U; ++index) {
    left_norm_sq += left.orientation_xyzw[index] * left.orientation_xyzw[index];
    right_norm_sq += right.orientation_xyzw[index] * right.orientation_xyzw[index];
    dot += left.orientation_xyzw[index] * right.orientation_xyzw[index];
  }
  if (left_norm_sq <= 0.0 || right_norm_sq <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  dot /= std::sqrt(left_norm_sq * right_norm_sq);
  dot = std::clamp(std::abs(dot), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

GoalMonitor::GoalMonitor(GoalMonitorConfig config)
: config_(std::move(config))
{
}

MotionStatus GoalMonitor::begin(
  const std::string & session_id, const JointTarget & joint_goal,
  const std::optional<Pose> & cartesian_goal, const SteadyTime now)
{
  const std::array<double, 4> tolerances{{
    config_.joint_position_tolerance_rad,
    config_.joint_velocity_tolerance_rad_s,
    config_.cartesian_position_tolerance_m,
    config_.cartesian_orientation_tolerance_rad}};
  if (!std::all_of(
      tolerances.begin(), tolerances.end(),
      [](const double value) {return std::isfinite(value) && value >= 0.0;}) ||
    config_.stable_duration < std::chrono::milliseconds::zero() ||
    config_.move_timeout <= std::chrono::milliseconds::zero())
  {
    return {StatusCode::INVALID_ARGUMENT, "goal monitor configuration is invalid"};
  }
  if (session_id.empty() || joint_goal.joint_names.empty() ||
    joint_goal.joint_names.size() != joint_goal.positions_rad.size())
  {
    return {StatusCode::INVALID_ARGUMENT, "goal identity/names/positions are invalid"};
  }
  const std::set<std::string> unique(
    joint_goal.joint_names.begin(), joint_goal.joint_names.end());
  if (unique.size() != joint_goal.joint_names.size() || unique.count("") != 0U ||
    !std::all_of(
      joint_goal.positions_rad.begin(), joint_goal.positions_rad.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return {StatusCode::INVALID_ARGUMENT, "goal joints must be unique and finite"};
  }
  if (cartesian_goal && !finite_pose(*cartesian_goal)) {
    return {StatusCode::INVALID_ARGUMENT, "Cartesian goal is not finite"};
  }
  goals_[session_id] = GoalState{joint_goal, cartesian_goal, now, std::nullopt};
  return MotionStatus::Ok();
}

GoalObservationResult GoalMonitor::observe(
  const std::string & session_id, const JointFeedback & feedback,
  const std::optional<Pose> & fk_pose, const SteadyTime now)
{
  const auto found = goals_.find(session_id);
  if (found == goals_.end()) {
    return {
      GoalObservation::INVALID_FEEDBACK,
      {StatusCode::INTERNAL_ERROR, "goal monitor has no session: " + session_id}};
  }
  auto & goal = found->second;
  if (now - goal.started_at > config_.move_timeout) {
    return {
      GoalObservation::TIMED_OUT,
      {StatusCode::TIMEOUT, "Move exceeded its steady-clock timeout"}};
  }
  if (feedback.joint_names.size() != feedback.positions_rad.size() ||
    feedback.joint_names.size() != feedback.velocities_rad_s.size())
  {
    goal.stable_since.reset();
    return {
      GoalObservation::INVALID_FEEDBACK,
      {StatusCode::INVALID_ARGUMENT, "feedback names/positions/velocities length mismatch"}};
  }

  std::map<std::string, std::size_t> feedback_index;
  for (std::size_t index = 0; index < feedback.joint_names.size(); ++index) {
    if (feedback.joint_names[index].empty() ||
      !feedback_index.emplace(feedback.joint_names[index], index).second ||
      !std::isfinite(feedback.positions_rad[index]) ||
      !std::isfinite(feedback.velocities_rad_s[index]))
    {
      goal.stable_since.reset();
      return {
        GoalObservation::INVALID_FEEDBACK,
        {StatusCode::INVALID_ARGUMENT, "feedback joint data is duplicate or non-finite"}};
    }
  }

  bool within_tolerance = true;
  for (std::size_t index = 0; index < goal.joint_goal.joint_names.size(); ++index) {
    const auto feedback_joint = feedback_index.find(goal.joint_goal.joint_names[index]);
    if (feedback_joint == feedback_index.end()) {
      goal.stable_since.reset();
      return {
        GoalObservation::INVALID_FEEDBACK,
        {StatusCode::INVALID_ARGUMENT, "feedback is missing a goal joint"}};
    }
    const auto feedback_position = feedback.positions_rad[feedback_joint->second];
    const auto feedback_velocity = feedback.velocities_rad_s[feedback_joint->second];
    within_tolerance = within_tolerance &&
      std::abs(feedback_position - goal.joint_goal.positions_rad[index]) <=
      config_.joint_position_tolerance_rad &&
      std::abs(feedback_velocity) <= config_.joint_velocity_tolerance_rad_s;
  }

  if (goal.cartesian_goal) {
    if (!fk_pose || !finite_pose(*fk_pose)) {
      goal.stable_since.reset();
      return {
        GoalObservation::INVALID_FEEDBACK,
        {StatusCode::SDK_ERROR, "Cartesian Move requires a valid original-SDK FK result"}};
    }
    double squared_distance = 0.0;
    for (std::size_t index = 0; index < 3U; ++index) {
      const auto delta = fk_pose->position_m[index] - goal.cartesian_goal->position_m[index];
      squared_distance += delta * delta;
    }
    within_tolerance = within_tolerance &&
      std::sqrt(squared_distance) <= config_.cartesian_position_tolerance_m &&
      orientation_distance(*fk_pose, *goal.cartesian_goal) <=
      config_.cartesian_orientation_tolerance_rad;
  }

  if (!within_tolerance) {
    goal.stable_since.reset();
    return {GoalObservation::RUNNING, MotionStatus::Ok()};
  }
  if (!goal.stable_since) {
    goal.stable_since = now;
  }
  if (now - *goal.stable_since >= config_.stable_duration) {
    return {GoalObservation::REACHED, MotionStatus::Ok()};
  }
  return {GoalObservation::RUNNING, MotionStatus::Ok()};
}

void GoalMonitor::erase(const std::string & session_id)
{
  goals_.erase(session_id);
}

bool GoalMonitor::contains(const std::string & session_id) const
{
  return goals_.count(session_id) != 0U;
}

}  // namespace humanoid_motion_server::motion
