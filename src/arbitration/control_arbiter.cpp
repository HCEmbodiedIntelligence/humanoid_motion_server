#include "humanoid_motion_server/arbitration/control_arbiter.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace humanoid_motion_server::motion
{
namespace
{

bool overlaps(
  const std::vector<std::string> & left,
  const std::vector<std::string> & right)
{
  std::set<std::string> names(left.begin(), left.end());
  return std::any_of(
    right.begin(), right.end(),
    [&names](const std::string & name) {return names.count(name) != 0U;});
}

bool contains_id(
  const std::vector<std::string> & values, const std::string & id)
{
  return std::find(values.begin(), values.end(), id) != values.end();
}

}  // namespace

MotionStatus ControlArbiter::registerEndpoint(const EndpointPolicy & policy)
{
  if (policy.endpoint_name.empty() || policy.group_name.empty()) {
    return {StatusCode::INVALID_ARGUMENT, "endpoint and group names must be non-empty"};
  }
  if (is_servo(policy.kind) && policy.servo_lease <= std::chrono::milliseconds::zero()) {
    return {StatusCode::INVALID_ARGUMENT, "servo lease must be positive"};
  }
  const auto existing = policies_.find(policy.endpoint_name);
  if (existing != policies_.end()) {
    return {StatusCode::INVALID_ARGUMENT, "duplicate endpoint policy: " + policy.endpoint_name};
  }
  policies_.emplace(policy.endpoint_name, policy);
  return MotionStatus::Ok();
}

MotionStatus ControlArbiter::validateClaim(
  const CommandClaim & claim, const bool servo) const
{
  if (claim.session_id.empty() || claim.endpoint_name.empty() || claim.group_name.empty()) {
    return {StatusCode::INVALID_ARGUMENT, "claim identity and group must be non-empty"};
  }
  if (claim.joint_names.empty()) {
    return {StatusCode::INVALID_ARGUMENT, "claim joint_names must not be empty"};
  }
  const std::set<std::string> unique(claim.joint_names.begin(), claim.joint_names.end());
  if (unique.size() != claim.joint_names.size() || unique.count("") != 0U) {
    return {StatusCode::INVALID_ARGUMENT, "claim joint_names must be non-empty and unique"};
  }
  const auto policy = policies_.find(claim.endpoint_name);
  if (policy == policies_.end()) {
    return {StatusCode::NOT_CONFIGURED, "no policy for endpoint: " + claim.endpoint_name};
  }
  if (policy->second.kind != claim.kind || policy->second.group_name != claim.group_name) {
    return {StatusCode::INVALID_ARGUMENT, "claim kind/group does not match endpoint policy"};
  }
  if (servo != is_servo(claim.kind)) {
    return {StatusCode::INVALID_ARGUMENT, servo ? "expected Servo claim" : "expected Move claim"};
  }
  return MotionStatus::Ok();
}

std::vector<std::string> ControlArbiter::computeWinners(const SteadyTime now) const
{
  std::vector<const ClaimState *> candidates;
  candidates.reserve(claims_.size());
  for (const auto & entry : claims_) {
    const auto & state = entry.second;
    if (is_servo(state.claim.kind) && now - state.received_at >= state.lease) {
      continue;
    }
    candidates.push_back(&state);
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const ClaimState * left, const ClaimState * right) {
      if (left->priority != right->priority) {
        return left->priority > right->priority;
      }
      return left->receive_sequence > right->receive_sequence;
    });

  std::vector<std::string> winners;
  std::vector<const ClaimState *> selected;
  for (const auto * candidate : candidates) {
    const bool conflict = std::any_of(
      selected.begin(), selected.end(),
      [candidate](const ClaimState * winner) {
        return overlaps(candidate->claim.joint_names, winner->claim.joint_names);
      });
    if (!conflict) {
      selected.push_back(candidate);
      winners.push_back(candidate->claim.session_id);
    }
  }
  return winners;
}

std::vector<std::string> ControlArbiter::removePreemptedMoves(
  const std::vector<std::string> & old_winners,
  const std::vector<std::string> & new_winners)
{
  std::vector<std::string> result;
  for (const auto & id : old_winners) {
    if (contains_id(new_winners, id)) {
      continue;
    }
    const auto claim = claims_.find(id);
    if (claim != claims_.end() && is_move(claim->second.claim.kind)) {
      result.push_back(id);
    }
  }
  for (const auto & id : result) {
    claims_.erase(id);
  }
  return result;
}

std::vector<std::string> ControlArbiter::removeExpiredServos(const SteadyTime now)
{
  std::vector<std::string> expired;
  for (auto it = claims_.begin(); it != claims_.end(); ) {
    const auto & state = it->second;
    if (is_servo(state.claim.kind) && now - state.received_at >= state.lease) {
      expired.push_back(it->first);
      it = claims_.erase(it);
    } else {
      ++it;
    }
  }
  return expired;
}

ArbitrationResult ControlArbiter::snapshot(
  MotionStatus status, const bool admitted,
  std::vector<std::string> preempted,
  std::vector<std::string> expired,
  const SteadyTime now) const
{
  return {
    std::move(status), admitted, computeWinners(now),
    std::move(preempted), std::move(expired)};
}

ArbitrationResult ControlArbiter::submitMove(
  const CommandClaim & claim, const SteadyTime now)
{
  const auto validation = validateClaim(claim, false);
  if (!validation.ok()) {
    return snapshot(validation, false, {}, {}, now);
  }
  if (claims_.count(claim.session_id) != 0U) {
    return snapshot(
      {StatusCode::INVALID_ARGUMENT, "duplicate session_id: " + claim.session_id},
      false, {}, {}, now);
  }

  const auto expired = removeExpiredServos(now);
  const auto old_winners = computeWinners(now);
  const auto & policy = policies_.at(claim.endpoint_name);
  claims_.emplace(
    claim.session_id,
    ClaimState{claim, policy.priority, next_sequence_++, now, policy.servo_lease});

  auto new_winners = computeWinners(now);
  auto preempted = removePreemptedMoves(old_winners, new_winners);
  new_winners = computeWinners(now);
  if (!contains_id(new_winners, claim.session_id)) {
    claims_.erase(claim.session_id);
    return snapshot(
      {StatusCode::REJECTED, "Move conflicts with an equal or higher priority command"},
      false, std::move(preempted), expired, now);
  }
  return snapshot(MotionStatus::Ok(), true, std::move(preempted), expired, now);
}

ArbitrationResult ControlArbiter::updateServo(
  const CommandClaim & claim, const SteadyTime now)
{
  const auto validation = validateClaim(claim, true);
  if (!validation.ok()) {
    return snapshot(validation, false, {}, {}, now);
  }

  const auto expired = removeExpiredServos(now);
  const auto old_winners = computeWinners(now);
  const auto & policy = policies_.at(claim.endpoint_name);
  claims_[claim.session_id] =
    ClaimState{claim, policy.priority, next_sequence_++, now, policy.servo_lease};
  const auto provisional = computeWinners(now);
  auto preempted = removePreemptedMoves(old_winners, provisional);
  const auto current = computeWinners(now);
  return snapshot(
    MotionStatus::Ok(), contains_id(current, claim.session_id),
    std::move(preempted), expired, now);
}

ArbitrationResult ControlArbiter::evaluate(const SteadyTime now)
{
  auto expired = removeExpiredServos(now);
  return snapshot(MotionStatus::Ok(), false, {}, std::move(expired), now);
}

void ControlArbiter::release(const std::string & session_id)
{
  claims_.erase(session_id);
}

bool ControlArbiter::contains(const std::string & session_id) const
{
  return claims_.count(session_id) != 0U;
}

std::vector<std::string> ControlArbiter::winners(const SteadyTime now) const
{
  return computeWinners(now);
}

}  // namespace humanoid_motion_server::motion
