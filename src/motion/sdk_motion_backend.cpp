#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

#include "motion_control/rtc.hpp"
#include "move_p_waypoint_resolver.hpp"
#include "robo_manip/core/system_init.hpp"
#include "robo_manip/tasks/move_joint/move_joint.hpp"
#include "robo_manip/tasks/move_line/move_line.hpp"
#include "sdk_units.hpp"

namespace humanoid_motion_server::motion
{
namespace
{

using sdk_units::scaled;

bool finite_values(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](const double value) {return std::isfinite(value);});
}

MotionStatus sdk_bool_failure(const std::string & api)
{
  return {
    StatusCode::SDK_ERROR, api + " returned false", api, -1};
}

Pose from_kinematics_pose(const humanoid_motion_server::kinematics::Pose & input)
{
  Pose output;
  output.position_m = {
    input.position_m.x, input.position_m.y, input.position_m.z};
  output.orientation_xyzw = {
    input.orientation.x, input.orientation.y,
    input.orientation.z, input.orientation.w};
  return output;
}

std::vector<double> reorder(
  const std::vector<std::string> & source_names,
  const std::vector<double> & source_values,
  const std::vector<std::string> & target_names)
{
  if (source_names.size() != source_values.size()) {
    throw std::invalid_argument("joint names/value length mismatch");
  }
  std::map<std::string, double> values;
  for (std::size_t index = 0; index < source_names.size(); ++index) {
    if (!values.emplace(source_names[index], source_values[index]).second) {
      throw std::invalid_argument("duplicate joint name: " + source_names[index]);
    }
  }
  std::vector<double> result;
  result.reserve(target_names.size());
  for (const auto & name : target_names) {
    const auto found = values.find(name);
    if (found == values.end()) {
      throw std::invalid_argument("missing joint: " + name);
    }
    result.push_back(found->second);
  }
  return result;
}

motion_control::types::RobotState to_sdk_state(
  const JointFeedback & feedback, const JointGroupModel & group)
{
  motion_control::types::RobotState state;
  auto & joint_group = state.addJointGroup(group.name);
  joint_group.positions = scaled(
    reorder(feedback.joint_names, feedback.positions_rad, group.joint_names),
    sdk_units::kRadToDeg);
  if (!feedback.velocities_rad_s.empty()) {
    joint_group.velocities = scaled(
      reorder(feedback.joint_names, feedback.velocities_rad_s, group.joint_names),
      sdk_units::kRadToDeg);
  }
  return state;
}

JointTarget feedback_target(
  const JointFeedback & feedback, const JointGroupModel & group)
{
  JointTarget target;
  target.joint_names = group.joint_names;
  target.positions_rad = reorder(
    feedback.joint_names, feedback.positions_rad, group.joint_names);
  if (!feedback.velocities_rad_s.empty()) {
    target.velocities_rad_s = reorder(
      feedback.joint_names, feedback.velocities_rad_s, group.joint_names);
  }
  return target;
}

const std::string & request_group(const SessionRequest & request)
{
  return std::visit(
    [](
      const auto & value) -> const std::string & {return value.group_name;}, request);
}

}  // namespace

class SdkMotionBackend::Impl
{
public:
  struct Session
  {
    MotionKind kind{MotionKind::MOVE_J};
    std::string group_name;
    std::vector<std::string> joint_names;
    std::unique_ptr<robo_manip::tasks::MoveJoint> move_joint;
    std::unique_ptr<robo_manip::tasks::MoveLine> move_line;
    std::vector<std::vector<double>> joint_waypoints_deg;
    std::size_t waypoint_index{0};
    motion_control::types::Pose cartesian_target;
  };

  Impl(
    robo_manip::core::MotionContext context,
    std::vector<JointGroupModel> groups,
    HumanoidKinematicsPtr kinematics)
  : base_context(std::move(context)), kinematics(std::move(kinematics))
  {
    for (auto & group : groups) {
      group_models.emplace(group.name, std::move(group));
    }
    for (const auto & entry : group_models) {
      auto rtc = std::make_shared<motion_control::Rtc>();
      rtc->configure(base_context.rtc->config());
      final_rtcs.emplace(entry.first, std::move(rtc));
    }
  }

  robo_manip::core::MotionContext makeSessionContext() const
  {
    auto result = base_context;
    result.rtc = std::make_shared<motion_control::Rtc>();
    result.rtc->configure(base_context.rtc->config());
    // Realtime MoveJoint/MoveLine require only the shared RKD model and their
    // private RTC. Do not accidentally retain unrelated mutable execution or
    // planning modules in a session context.
    result.rmp.reset();
    result.wbc.reset();
    result.mpc.reset();
    result.force_control.reset();
    result.robot_driver.reset();
    result.trajectory_executor.reset();
    result.kinematics.reset();
    return result;
  }

  const JointGroupModel * group(const std::string & name) const
  {
    const auto found = group_models.find(name);
    return found == group_models.end() ? nullptr : &found->second;
  }

  robo_manip::core::MotionContext base_context;
  std::map<std::string, JointGroupModel> group_models;
  std::map<std::string, std::shared_ptr<motion_control::Rtc>> final_rtcs;
  HumanoidKinematicsPtr kinematics;
  std::map<std::string, Session> sessions;
  std::set<std::string> stopped_sessions;
};

SdkMotionBackend::SdkMotionBackend(std::unique_ptr<Impl> impl)
: impl_(std::move(impl))
{
}

SdkMotionBackend::~SdkMotionBackend() = default;

MotionStatus SdkMotionBackend::startSession(
  const std::string & session_id, const SessionRequest & request,
  const JointFeedback & feedback, const double period_sec)
{
  if (session_id.empty() || period_sec <= 0.0 || !std::isfinite(period_sec)) {
    return {StatusCode::INVALID_ARGUMENT, "session id and period must be valid"};
  }
  if (impl_->sessions.count(session_id) != 0U) {
    return {StatusCode::INVALID_ARGUMENT, "session is already active: " + session_id};
  }
  const auto * group = impl_->group(request_group(request));
  if (group == nullptr) {
    return {StatusCode::NOT_CONFIGURED, "unknown joint group: " + request_group(request)};
  }

  try {
    auto context = impl_->makeSessionContext();
    const auto state = to_sdk_state(feedback, *group);
    Impl::Session session;
    session.kind = motion_kind(request);
    session.group_name = group->name;
    session.joint_names = group->joint_names;

    if (session.kind == MotionKind::MOVE_J || session.kind == MotionKind::SERVO_J ||
      session.kind == MotionKind::MOVE_P)
    {
      session.move_joint = std::make_unique<robo_manip::tasks::MoveJoint>();
      robo_manip::tasks::MoveJoint::JointRequest sdk_request;
      sdk_request.request_id = session_id;
      sdk_request.group_name = group->name;
      MotionLimits limits;

      if (const auto * move = std::get_if<MoveJRequest>(&request)) {
        session.joint_waypoints_deg.push_back(
          scaled(
            reorder(
              move->target.joint_names, move->target.positions_rad,
              group->joint_names), sdk_units::kRadToDeg));
        limits = move->limits;
      } else if (const auto * servo = std::get_if<ServoJRequest>(&request)) {
        session.joint_waypoints_deg.push_back(
          scaled(
            reorder(
              servo->target.joint_names, servo->target.positions_rad,
              group->joint_names), sdk_units::kRadToDeg));
        limits = servo->limits;
      } else {
        const auto & move = std::get<MovePRequest>(request);
        if (!impl_->kinematics) {
          return {StatusCode::NOT_CONFIGURED, "MoveP requires SDK kinematics adapter"};
        }
        auto seed = feedback_target(feedback, *group);
        const auto resolution = detail::resolveMovePWaypoints(
          move, std::move(seed), group->joint_names, *impl_->kinematics);
        if (!resolution.status.ok()) {
          return resolution.status;
        }
        for (const auto & solution : resolution.joint_waypoints) {
          session.joint_waypoints_deg.push_back(
            scaled(solution.positions_rad, sdk_units::kRadToDeg));
        }
        limits = move.limits;
      }
      sdk_request.waypoints_deg = session.joint_waypoints_deg;
      sdk_request.limits = sdk_units::toSdkLimits(limits);
      robo_manip::tasks::MoveJoint::OptParams options;
      options.dt_sec = period_sec;
      if (!session.move_joint->StartRealtimeMoveJoint(context, state, sdk_request, options)) {
        session.move_joint->StopRealtimeMoveJoint();
        return sdk_bool_failure(
          "robo_manip::tasks::MoveJoint::StartRealtimeMoveJoint");
      }
    } else {
      session.move_line = std::make_unique<robo_manip::tasks::MoveLine>();
      robo_manip::tasks::MoveLine::LineRequest sdk_request;
      sdk_request.request_id = session_id;
      sdk_request.group_name = group->name;
      MotionLimits limits;
      if (const auto * move = std::get_if<MoveLRequest>(&request)) {
        sdk_request.base_link = move->base_link;
        sdk_request.link_name = move->link_name;
        sdk_request.goal_pose = sdk_units::toSdkPose(move->target);
        session.cartesian_target = sdk_request.goal_pose;
        limits = move->limits;
      } else {
        const auto & servo = std::get<ServoPRequest>(request);
        sdk_request.base_link = servo.base_link;
        sdk_request.link_name = servo.link_name;
        sdk_request.goal_pose = sdk_units::toSdkPose(servo.target);
        session.cartesian_target = sdk_request.goal_pose;
        limits = servo.limits;
      }
      sdk_request.limits = sdk_units::toSdkLimits(limits);
      robo_manip::tasks::MoveLine::OptParams options;
      options.dt_sec = period_sec;
      if (!session.move_line->StartRealtimeMoveLine(context, state, sdk_request, options)) {
        session.move_line->StopRealtimeMoveLine();
        return sdk_bool_failure(
          "robo_manip::tasks::MoveLine::StartRealtimeMoveLine");
      }
    }
    impl_->stopped_sessions.erase(session_id);
    impl_->sessions.emplace(session_id, std::move(session));
    return MotionStatus::Ok();
  } catch (const std::exception & error) {
    return {StatusCode::SDK_ERROR, error.what(), "SdkMotionBackend::startSession", -1};
  }
}

BackendTick SdkMotionBackend::tickSession(
  const std::string & session_id, const DynamicTarget * dynamic_target,
  const double period_sec)
{
  const auto found = impl_->sessions.find(session_id);
  if (found == impl_->sessions.end()) {
    return {{StatusCode::NOT_CONFIGURED, "session is not active: " + session_id}, {}, false};
  }
  if (period_sec <= 0.0 || !std::isfinite(period_sec)) {
    return {{StatusCode::INVALID_ARGUMENT, "period must be positive and finite"}, {}, false};
  }
  auto & session = found->second;
  try {
    if (session.move_joint) {
      std::vector<double> target_deg;
      if (dynamic_target != nullptr && std::holds_alternative<JointTarget>(*dynamic_target) &&
        session.kind != MotionKind::MOVE_P)
      {
        const auto & target = std::get<JointTarget>(*dynamic_target);
        target_deg = scaled(
          reorder(target.joint_names, target.positions_rad, session.joint_names),
          sdk_units::kRadToDeg);
      } else {
        target_deg = session.joint_waypoints_deg.at(session.waypoint_index);
      }
      robo_manip::tasks::MoveJoint::JointPos command;
      bool reached = false;
      robo_manip::tasks::MoveJoint::OptParams options;
      options.dt_sec = period_sec;
      if (!session.move_joint->TickRealtimeMoveJoint(target_deg, command, reached, options)) {
        return {
          sdk_bool_failure("robo_manip::tasks::MoveJoint::TickRealtimeMoveJoint"), {}, false};
      }
      bool path_reached = reached;
      if (session.kind == MotionKind::MOVE_P && reached &&
        session.waypoint_index + 1U < session.joint_waypoints_deg.size())
      {
        ++session.waypoint_index;
        path_reached = false;
      }
      auto output = sdk_units::fromSdkJointCommand(
        command.group_name.empty() ? session.group_name : command.group_name,
        command.joint_names, command.positions, command.velocities);
      return {MotionStatus::Ok(), std::move(output), path_reached};
    }

    if (dynamic_target != nullptr && std::holds_alternative<Pose>(*dynamic_target)) {
      session.cartesian_target = sdk_units::toSdkPose(
        std::get<Pose>(*dynamic_target));
    }
    robo_manip::tasks::MoveLine::JointPos command;
    bool reached = false;
    robo_manip::tasks::MoveLine::OptParams options;
    options.dt_sec = period_sec;
    if (!session.move_line->TickRealtimeMoveLine(
        session.cartesian_target, command, reached, options))
    {
      return {
        sdk_bool_failure("robo_manip::tasks::MoveLine::TickRealtimeMoveLine"), {}, false};
    }
    auto output = sdk_units::fromSdkJointCommand(
      command.group_name.empty() ? session.group_name : command.group_name,
      command.joint_names, command.positions, command.velocities);
    return {MotionStatus::Ok(), std::move(output), reached};
  } catch (const std::exception & error) {
    return {{StatusCode::SDK_ERROR, error.what(), "SdkMotionBackend::tickSession", -1}, {}, false};
  }
}

MotionStatus SdkMotionBackend::stopSession(const std::string & session_id)
{
  const auto found = impl_->sessions.find(session_id);
  if (found == impl_->sessions.end()) {
    impl_->stopped_sessions.insert(session_id);
    return MotionStatus::Ok();
  }
  if (found->second.move_joint) {
    found->second.move_joint->StopRealtimeMoveJoint();
  }
  if (found->second.move_line) {
    found->second.move_line->StopRealtimeMoveLine();
  }
  impl_->sessions.erase(found);
  impl_->stopped_sessions.insert(session_id);
  return MotionStatus::Ok();
}

MotionStatus SdkMotionBackend::resetFinalJointTarget(
  const std::string & group_name, const JointFeedback & feedback)
{
  const auto * group = impl_->group(group_name);
  const auto rtc = impl_->final_rtcs.find(group_name);
  if (group == nullptr || rtc == impl_->final_rtcs.end()) {
    return {StatusCode::NOT_CONFIGURED, "no final RTC for group: " + group_name};
  }
  try {
    const auto current = feedback_target(feedback, *group);
    motion_control::RtcJointTarget target;
    target.joint_names = current.joint_names;
    target.positions = scaled(current.positions_rad, sdk_units::kRadToDeg);
    target.velocities = scaled(current.velocities_rad_s, sdk_units::kRadToDeg);
    rtc->second->resetJointTarget(target);
    return MotionStatus::Ok();
  } catch (const std::exception & error) {
    return {
      StatusCode::SDK_ERROR, error.what(),
      "motion_control::Rtc::resetJointTarget", -1};
  }
}

BackendTick SdkMotionBackend::updateFinalJointTarget(
  const std::string & group_name, const JointCommand & candidate,
  const double period_sec)
{
  const auto * group = impl_->group(group_name);
  const auto rtc = impl_->final_rtcs.find(group_name);
  if (group == nullptr || rtc == impl_->final_rtcs.end()) {
    return {{StatusCode::NOT_CONFIGURED, "no final RTC for group: " + group_name}, {}, false};
  }
  if (period_sec <= 0.0 || !std::isfinite(period_sec)) {
    return {{StatusCode::INVALID_ARGUMENT, "period must be positive and finite"}, {}, false};
  }
  try {
    motion_control::RtcJointTarget target;
    target.joint_names = group->joint_names;
    target.positions = scaled(
      reorder(candidate.joint_names, candidate.positions_rad, group->joint_names),
      sdk_units::kRadToDeg);
    if (!candidate.velocities_rad_s.empty()) {
      target.velocities = scaled(
        reorder(
          candidate.joint_names, candidate.velocities_rad_s,
          group->joint_names), sdk_units::kRadToDeg);
    }
    if (!candidate.accelerations_rad_s2.empty()) {
      target.accelerations = scaled(
        reorder(
          candidate.joint_names, candidate.accelerations_rad_s2,
          group->joint_names), sdk_units::kRadToDeg);
    }
    const auto state = rtc->second->updateJointTarget(target, period_sec);
    if (state.status != motion_control::RtcStatus::kRunning &&
      state.status != motion_control::RtcStatus::kReached)
    {
      return {{
        StatusCode::SDK_ERROR, state.message,
        "motion_control::Rtc::updateJointTarget",
        static_cast<std::int64_t>(state.status)}, {}, false};
    }
    JointCommand output;
    output.group_name = group_name;
    output.joint_names = state.joint_names;
    output.positions_rad = scaled(state.positions, sdk_units::kDegToRad);
    output.velocities_rad_s = scaled(state.velocities, sdk_units::kDegToRad);
    output.accelerations_rad_s2 = scaled(
      state.accelerations, sdk_units::kDegToRad);
    output.passed_final_sdk_rtc = true;
    return {
      MotionStatus::Ok(), std::move(output),
      state.status == motion_control::RtcStatus::kReached};
  } catch (const std::exception & error) {
    return {{
      StatusCode::SDK_ERROR, error.what(),
      "motion_control::Rtc::updateJointTarget", -1}, {}, false};
  }
}

ForwardKinematicsResult SdkMotionBackend::forwardKinematics(
  const ForwardKinematicsRequest & request)
{
  if (!impl_->kinematics) {
    return {{StatusCode::NOT_CONFIGURED, "SDK kinematics adapter is not configured"}, {}};
  }
  humanoid_motion_server::kinematics::ForwardKinematicsRequest adapted;
  adapted.kinematics.group_name = request.group_name;
  adapted.kinematics.base_link = request.base_link;
  adapted.kinematics.link_name = request.link_name;
  adapted.joint_state.joint_names = request.joints.joint_names;
  adapted.joint_state.positions_rad = request.joints.positions_rad;
  try {
    const auto result = impl_->kinematics->forwardKinematics(adapted);
    if (!result.ok()) {
      return {{
        StatusCode::SDK_ERROR, result.status.message,
        "humanoid_motion_server::kinematics::IKinematics::forwardKinematics",
        static_cast<std::int64_t>(result.status.code)}, {}};
    }
    return {MotionStatus::Ok(), from_kinematics_pose(*result.value)};
  } catch (const std::exception & error) {
    return {{
      StatusCode::SDK_ERROR, error.what(),
      "humanoid_motion_server::kinematics::IKinematics::forwardKinematics", -1}, {}};
  }
}

MotionContextFactoryResult MotionContextFactory::createFromSdkYaml(
  const std::string & sdk_yaml_path,
  const MotionContextFactoryOptions & options)
{
  if (sdk_yaml_path.empty()) {
    return {{StatusCode::INVALID_ARGUMENT, "SDK YAML path must be non-empty"}, {}, {}};
  }
  if (options.joint_groups.empty()) {
    return {{StatusCode::INVALID_ARGUMENT, "at least one joint group model is required"}, {}, {}};
  }

  robo_manip::core::SystemInitResult initialized;
  try {
    initialized = robo_manip::core::InitializeSystemFromFile(sdk_yaml_path);
  } catch (const std::exception & error) {
    return {{
      StatusCode::SDK_ERROR, error.what(),
      "robo_manip::core::InitializeSystemFromFile", -1}, {}, {}};
  }
  if (!initialized.success) {
    return {{
      StatusCode::SDK_ERROR, initialized.message,
      "robo_manip::core::InitializeSystemFromFile", -1}, {}, {}};
  }
  if (!initialized.context.rkd || !initialized.context.rtc) {
    return {{
      StatusCode::NOT_CONFIGURED,
      "SDK MotionContext must provide RKD and RTC",
      "robo_manip::core::InitializeSystemFromFile", -1}, {}, {}};
  }

  std::vector<JointGroupModel> resolved = options.joint_groups;
  std::set<std::string> names;
  for (auto & group : resolved) {
    if (group.name.empty() || !names.insert(group.name).second) {
      return {{StatusCode::INVALID_ARGUMENT, "joint group names must be non-empty and unique"}, {},
        {}};
    }
    const auto sdk_names = initialized.context.rkd->jointGroup(group.name);
    if (sdk_names.empty()) {
      return {{StatusCode::NOT_CONFIGURED, "SDK RKD has no group: " + group.name}, {}, {}};
    }
    if (group.joint_names.empty()) {
      group.joint_names = sdk_names;
    } else if (group.joint_names != sdk_names) {
      return {{
        StatusCode::INVALID_ARGUMENT,
        "configured joint order differs from SDK RKD group: " + group.name}, {}, {}};
    }
    if (group.lower_position_rad.size() != group.joint_names.size() ||
      group.upper_position_rad.size() != group.joint_names.size() ||
      !finite_values(group.lower_position_rad) || !finite_values(group.upper_position_rad))
    {
      return {{
        StatusCode::INVALID_ARGUMENT,
        "model position limits are required in SI for every joint in " + group.name}, {}, {}};
    }
    for (std::size_t index = 0; index < group.joint_names.size(); ++index) {
      if (group.joint_names[index].empty() ||
        group.lower_position_rad[index] > group.upper_position_rad[index])
      {
        return {{StatusCode::INVALID_ARGUMENT, "invalid model joint limits/order"}, {}, {}};
      }
    }
  }

  try {
    auto kinematics = options.kinematics;
    if (!kinematics) {
      auto sdk_kinematics = humanoid_motion_server::kinematics::SdkKinematics::create();
      const auto load_status = sdk_kinematics->load(sdk_yaml_path);
      if (!load_status.ok()) {
        return {{
          StatusCode::SDK_ERROR, load_status.message,
          "humanoid_motion_server::kinematics::SdkKinematics::load",
          static_cast<std::int64_t>(load_status.code)}, {}, {}};
      }
      kinematics = std::move(sdk_kinematics);
    }
    auto impl = std::make_unique<SdkMotionBackend::Impl>(
      initialized.context, resolved, std::move(kinematics));
    auto backend = std::shared_ptr<SdkMotionBackend>(
      new SdkMotionBackend(std::move(impl)));
    return {MotionStatus::Ok(), std::move(backend), std::move(resolved)};
  } catch (const std::exception & error) {
    return {{
      StatusCode::SDK_ERROR, error.what(),
      "MotionContextFactory::createFromSdkYaml", -1}, {}, {}};
  }
}

}  // namespace humanoid_motion_server::motion
