#ifndef FAKE_SDK_MOTION_BACKEND_HPP_
#define FAKE_SDK_MOTION_BACKEND_HPP_

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"

namespace humanoid_motion_server::motion::test
{

class FakeSdkMotionBackend final : public ISdkMotionBackend
{
public:
  struct Active
  {
    SessionRequest request;
    JointFeedback feedback;
  };

  MotionStatus startSession(
    const std::string & session_id, const SessionRequest & request,
    const JointFeedback & feedback, double period_sec) override
  {
    ++start_calls[session_id];
    start_periods[session_id].push_back(period_sec);
    start_kinds[session_id] = motion_kind(request);
    if (!start_status.ok()) {
      return start_status;
    }
    active[session_id] = {request, feedback};
    return MotionStatus::Ok();
  }

  BackendTick tickSession(
    const std::string & session_id, const DynamicTarget * dynamic_target,
    double period_sec) override
  {
    ++tick_calls[session_id];
    tick_periods[session_id].push_back(period_sec);
    if (dynamic_target != nullptr) {
      dynamic_targets[session_id].push_back(*dynamic_target);
    }
    if (fail_tick_ids.count(session_id) != 0U) {
      return {{
        StatusCode::SDK_ERROR, "native fake failure",
        "FakeSdk::Tick", 73}, {}, false};
    }
    const auto found = active.find(session_id);
    if (found == active.end()) {
      return {{StatusCode::NOT_CONFIGURED, "not active", {}, -1}, {}, false};
    }

    JointCommand command;
    command.group_name = std::visit(
      [](const auto & request) {return request.group_name;}, found->second.request);
    if (dynamic_target != nullptr && std::holds_alternative<JointTarget>(*dynamic_target)) {
      const auto & target = std::get<JointTarget>(*dynamic_target);
      command.joint_names = target.joint_names;
      command.positions_rad = target.positions_rad;
      command.velocities_rad_s.assign(target.joint_names.size(), 0.0);
    } else {
      command.joint_names = found->second.feedback.joint_names;
      command.positions_rad = found->second.feedback.positions_rad;
      command.velocities_rad_s.assign(command.joint_names.size(), 0.0);
    }
    const auto override = candidate_overrides.find(command.group_name);
    if (override != candidate_overrides.end()) {
      command = override->second;
    }
    return {MotionStatus::Ok(), std::move(command), path_reached};
  }

  MotionStatus stopSession(const std::string & session_id) override
  {
    ++stop_calls[session_id];
    active.erase(session_id);
    return MotionStatus::Ok();
  }

  MotionStatus resetFinalJointTarget(
    const std::string & group_name, const JointFeedback & feedback) override
  {
    ++reset_calls[group_name];
    reset_feedback[group_name] = feedback;
    return reset_status;
  }

  BackendTick updateFinalJointTarget(
    const std::string & group_name, const JointCommand & candidate,
    double period_sec) override
  {
    ++final_rtc_calls[group_name];
    final_periods[group_name].push_back(period_sec);
    if (!final_status.ok()) {
      return {final_status, {}, false};
    }
    auto result = candidate;
    result.passed_final_sdk_rtc = !bypass_final_rtc;
    return {MotionStatus::Ok(), std::move(result), true};
  }

  ForwardKinematicsResult forwardKinematics(
    const ForwardKinematicsRequest &) override
  {
    ++fk_calls;
    return {fk_status, fk_pose};
  }

  MotionStatus start_status;
  MotionStatus reset_status;
  MotionStatus final_status;
  MotionStatus fk_status;
  Pose fk_pose;
  bool path_reached{true};
  bool bypass_final_rtc{false};
  int fk_calls{0};
  std::set<std::string> fail_tick_ids;
  std::map<std::string, Active> active;
  std::map<std::string, int> start_calls;
  std::map<std::string, std::vector<double>> start_periods;
  std::map<std::string, int> tick_calls;
  std::map<std::string, std::vector<double>> tick_periods;
  std::map<std::string, int> stop_calls;
  std::map<std::string, MotionKind> start_kinds;
  std::map<std::string, std::vector<DynamicTarget>> dynamic_targets;
  std::map<std::string, int> reset_calls;
  std::map<std::string, JointFeedback> reset_feedback;
  std::map<std::string, int> final_rtc_calls;
  std::map<std::string, std::vector<double>> final_periods;
  std::map<std::string, JointCommand> candidate_overrides;
};

}  // namespace humanoid_motion_server::motion::test

#endif  // FAKE_SDK_MOTION_BACKEND_HPP_
