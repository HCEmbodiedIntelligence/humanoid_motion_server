#include "humanoid_motion_server/motion/command_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <type_traits>
#include <utility>

namespace humanoid_motion_server::motion
{
namespace
{

template<typename RequestT>
const std::string & request_id(const RequestT & request)
{
  return request.request_id;
}

const std::string & request_id(const SessionRequest & request)
{
  return std::visit(
    [](const auto & value) -> const std::string & {return request_id(value);}, request);
}

const std::string & group_name(const SessionRequest & request)
{
  return std::visit(
    [](const auto & value) -> const std::string & {return value.group_name;}, request);
}

bool finite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](const double value) {return std::isfinite(value);});
}

bool finite_pose(const Pose & pose)
{
  if (!std::all_of(
      pose.position_m.begin(), pose.position_m.end(),
      [](const double value) {return std::isfinite(value);}) ||
    !std::all_of(
      pose.orientation_xyzw.begin(), pose.orientation_xyzw.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return false;
  }
  double norm_squared = 0.0;
  for (const auto value : pose.orientation_xyzw) {
    norm_squared += value * value;
  }
  return norm_squared > 1e-12;
}

MotionStatus validate_joint_target(
  const JointTarget & target, const JointGroupModel & group)
{
  if (target.joint_names.size() != group.joint_names.size() ||
    target.positions_rad.size() != target.joint_names.size() ||
    (!target.velocities_rad_s.empty() &&
    target.velocities_rad_s.size() != target.joint_names.size()) ||
    (!target.accelerations_rad_s2.empty() &&
    target.accelerations_rad_s2.size() != target.joint_names.size()))
  {
    return {StatusCode::INVALID_ARGUMENT, "joint target array lengths do not match the group"};
  }
  const std::set<std::string> target_names(
    target.joint_names.begin(), target.joint_names.end());
  const std::set<std::string> group_names(
    group.joint_names.begin(), group.joint_names.end());
  if (target_names != group_names || target_names.size() != target.joint_names.size() ||
    target_names.count("") != 0U)
  {
    return {StatusCode::INVALID_ARGUMENT,
      "joint target names must exactly match the configured group"};
  }
  if (!finite(target.positions_rad) || !finite(target.velocities_rad_s) ||
    !finite(target.accelerations_rad_s2))
  {
    return {StatusCode::INVALID_ARGUMENT, "joint target contains a non-finite value"};
  }
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < target.joint_names.size(); ++index) {
    positions.emplace(target.joint_names[index], target.positions_rad[index]);
  }
  for (std::size_t index = 0; index < group.joint_names.size(); ++index) {
    const auto position = positions.at(group.joint_names[index]);
    if (position < group.lower_position_rad[index] || position > group.upper_position_rad[index]) {
      return {
        StatusCode::LIMIT_VIOLATION,
        "joint target violates model limit: " + group.joint_names[index]};
    }
  }
  return MotionStatus::Ok();
}

MotionStatus validate_limits(
  const MotionLimits & limits, const std::size_t joint_count)
{
  const auto vector_valid = [joint_count](const std::vector<double> & values) {
      return (values.empty() || values.size() == joint_count) && finite(values) &&
             std::all_of(
        values.begin(), values.end(),
        [](const double value) {return value >= 0.0;});
    };
  if (!vector_valid(limits.joint_max_velocity_rad_s) ||
    !vector_valid(limits.joint_max_acceleration_rad_s2) ||
    !vector_valid(limits.joint_max_jerk_rad_s3))
  {
    return {StatusCode::INVALID_ARGUMENT, "joint motion limits have invalid length/value"};
  }
  const std::array<double, 6> cartesian{{
    limits.cartesian_max_linear_velocity_m_s,
    limits.cartesian_max_linear_acceleration_m_s2,
    limits.cartesian_max_linear_jerk_m_s3,
    limits.cartesian_max_angular_velocity_rad_s,
    limits.cartesian_max_angular_acceleration_rad_s2,
    limits.cartesian_max_angular_jerk_rad_s3}};
  if (!std::all_of(
      cartesian.begin(), cartesian.end(),
      [](const double value) {return std::isfinite(value) && value >= 0.0;}))
  {
    return {StatusCode::INVALID_ARGUMENT,
      "Cartesian motion limits must be finite and non-negative"};
  }
  return MotionStatus::Ok();
}

const MotionLimits & request_limits(const SessionRequest & request)
{
  return std::visit(
    [](const auto & value) -> const MotionLimits & {return value.limits;}, request);
}

}  // namespace

CommandPipeline::CommandPipeline(
  SdkMotionBackendPtr backend, std::vector<JointGroupModel> joint_groups,
  CommandPipelineConfig config, std::function<SteadyTime()> steady_now)
: backend_(std::move(backend)), config_(std::move(config)),
  steady_now_(std::move(steady_now)), monitor_(config_.goal_monitor)
{
  if (config_.control_frequency_hz > 0.0 && std::isfinite(config_.control_frequency_hz)) {
    period_sec_ = 1.0 / config_.control_frequency_hz;
    current_period_sec_ = period_sec_;
  }
  for (auto & group : joint_groups) {
    const std::set<std::string> unique(group.joint_names.begin(), group.joint_names.end());
    const bool valid = !group.name.empty() && !group.joint_names.empty() &&
      unique.size() == group.joint_names.size() && unique.count("") == 0U &&
      group.lower_position_rad.size() == group.joint_names.size() &&
      group.upper_position_rad.size() == group.joint_names.size() &&
      finite(group.lower_position_rad) && finite(group.upper_position_rad);
    if (!valid || groups_.count(group.name) != 0U) {
      configuration_status_ = {
        StatusCode::INVALID_ARGUMENT,
        "joint group models must have unique names/joints and complete finite limits"};
      continue;
    }
    for (std::size_t index = 0; index < group.joint_names.size(); ++index) {
      if (group.lower_position_rad[index] > group.upper_position_rad[index]) {
        configuration_status_ = {
          StatusCode::INVALID_ARGUMENT, "joint group lower limit exceeds upper limit"};
      }
    }
    groups_.emplace(group.name, std::move(group));
  }
}

MotionStatus CommandPipeline::validatePipelineConfig() const
{
  if (!configuration_status_.ok()) {
    return configuration_status_;
  }
  if (!backend_) {
    return {StatusCode::NOT_CONFIGURED, "SDK backend is null"};
  }
  if (config_.control_frequency_hz <= 0.0 || !std::isfinite(config_.control_frequency_hz) ||
    config_.feedback_max_age <= std::chrono::milliseconds::zero() ||
    config_.max_control_period <= std::chrono::milliseconds::zero() || !steady_now_ ||
    config_.servo_retry_interval <= std::chrono::milliseconds::zero() ||
    !std::isfinite(config_.feedback_limit_recovery_margin_rad) ||
    config_.feedback_limit_recovery_margin_rad < 0.0 ||
    !std::isfinite(config_.feedback_rebase_tolerance_rad) ||
    config_.feedback_rebase_tolerance_rad < 0.0)
  {
    return {StatusCode::INVALID_ARGUMENT, "pipeline rate/feedback age/recovery settings are invalid"};
  }
  if (groups_.empty()) {
    return {StatusCode::NOT_CONFIGURED, "no joint groups configured"};
  }
  const std::array<double, 4> goal_tolerances{{
    config_.goal_monitor.joint_position_tolerance_rad,
    config_.goal_monitor.joint_velocity_tolerance_rad_s,
    config_.goal_monitor.cartesian_position_tolerance_m,
    config_.goal_monitor.cartesian_orientation_tolerance_rad}};
  if (!std::all_of(
      goal_tolerances.begin(), goal_tolerances.end(),
      [](const double value) {return std::isfinite(value) && value >= 0.0;}) ||
    config_.goal_monitor.stable_duration < std::chrono::milliseconds::zero() ||
    config_.goal_monitor.move_timeout <= std::chrono::milliseconds::zero())
  {
    return {StatusCode::INVALID_ARGUMENT, "goal monitor configuration is invalid"};
  }
  return MotionStatus::Ok();
}

MotionStatus CommandPipeline::registerEndpoint(const EndpointPolicy & policy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto pipeline = validatePipelineConfig();
  if (!pipeline.ok()) {
    return pipeline;
  }
  if (groups_.count(policy.group_name) == 0U) {
    return {StatusCode::NOT_CONFIGURED, "endpoint references unknown group: " + policy.group_name};
  }
  const auto status = arbiter_.registerEndpoint(policy);
  if (status.ok()) {
    endpoints_.emplace(policy.endpoint_name, policy);
  }
  return status;
}

MotionStatus CommandPipeline::validateRequest(
  const EndpointPolicy & policy, const SessionRequest & request,
  const bool expect_servo, Session * session) const
{
  if (session == nullptr) {
    return {StatusCode::INTERNAL_ERROR, "null session output"};
  }
  const auto kind = motion_kind(request);
  if (is_servo(kind) != expect_servo || policy.kind != kind ||
    policy.group_name != group_name(request))
  {
    return {StatusCode::INVALID_ARGUMENT, "request kind/group does not match endpoint policy"};
  }
  const auto group = groups_.find(group_name(request));
  if (group == groups_.end()) {
    return {StatusCode::NOT_CONFIGURED, "unknown request group"};
  }
  if (!expect_servo && request_id(request).empty()) {
    return {StatusCode::INVALID_ARGUMENT, "Move request_id must be non-empty"};
  }
  const auto limits = validate_limits(request_limits(request), group->second.joint_names.size());
  if (!limits.ok()) {
    return limits;
  }

  MotionStatus request_status = MotionStatus::Ok();
  std::visit(
    [&](const auto & value) {
      using RequestT = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<RequestT, MoveJRequest>||
      std::is_same_v<RequestT, ServoJRequest>)
      {
        request_status = validate_joint_target(value.target, group->second);
      } else if constexpr (std::is_same_v<RequestT, MoveLRequest>||
      std::is_same_v<RequestT, ServoPRequest>)
      {
        if (value.base_link.empty() || value.link_name.empty() || !finite_pose(value.target)) {
          request_status = {
            StatusCode::INVALID_ARGUMENT,
            "Cartesian request requires frames and a finite non-zero quaternion pose"};
        }
      } else {
        if (value.base_link.empty() || value.link_name.empty() || value.waypoints.empty() ||
        !std::all_of(value.waypoints.begin(), value.waypoints.end(), finite_pose))
        {
          request_status = {
            StatusCode::INVALID_ARGUMENT,
            "MoveP requires frames and at least one finite waypoint"};
        }
      }
    }, request);
  if (!request_status.ok()) {
    return request_status;
  }

  session->endpoint_name = policy.endpoint_name;
  session->session_id = expect_servo ? "servo:" + policy.endpoint_name : request_id(request);
  session->group_name = group->first;
  session->joint_names = group->second.joint_names;
  session->request = request;
  return MotionStatus::Ok();
}

SubmissionResult CommandPipeline::submitMove(
  const std::string & endpoint_name, const SessionRequest & request,
  const SteadyTime now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto endpoint = endpoints_.find(endpoint_name);
  if (endpoint == endpoints_.end()) {
    return {{StatusCode::NOT_CONFIGURED, "unknown endpoint: " + endpoint_name}, {}};
  }
  Session session;
  const auto validation = validateRequest(endpoint->second, request, false, &session);
  if (!validation.ok()) {
    return {validation, {}};
  }
  if (sessions_.count(session.session_id) != 0U) {
    return {{StatusCode::INVALID_ARGUMENT, "duplicate session id: " + session.session_id}, {}};
  }
  session.submitted_at = now;
  const CommandClaim claim{
    session.session_id, endpoint_name, session.group_name,
    motion_kind(request), session.joint_names};
  auto arbitration = arbiter_.submitMove(claim, now);
  processPreemptions(arbitration.preempted_move_ids);
  for (const auto & expired : arbitration.expired_servo_ids) {
    terminate(
      expired, SessionState::ABORTED,
      {StatusCode::STALE_FEEDBACK, "Servo lease expired"});
  }
  if (!arbitration.status.ok() || !arbitration.admitted) {
    return {arbitration.status, session.session_id};
  }
  sessions_.emplace(session.session_id, std::move(session));
  revokeOutputs(claim.joint_names);
  return {MotionStatus::Ok(), claim.session_id};
}

SubmissionResult CommandPipeline::updateServo(
  const std::string & endpoint_name, const SessionRequest & request,
  const SteadyTime received_at, const SteadyTime now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto endpoint = endpoints_.find(endpoint_name);
  if (endpoint == endpoints_.end()) {
    return {{StatusCode::NOT_CONFIGURED, "unknown endpoint: " + endpoint_name}, {}};
  }
  Session updated;
  const auto validation = validateRequest(endpoint->second, request, true, &updated);
  if (!validation.ok()) {
    return {validation, {}};
  }
  if (received_at > now || now - received_at >= endpoint->second.servo_lease) {
    return {{StatusCode::REJECTED, "Servo target expired before arbitration"}, {}};
  }
  updated.submitted_at = received_at;
  auto existing = sessions_.find(updated.session_id);
  if (existing != sessions_.end() && received_at < existing->second.submitted_at) {
    return {{StatusCode::REJECTED, "out-of-order Servo target"}, {}};
  }
  if (existing != sessions_.end() &&
    now - existing->second.submitted_at >= endpoint->second.servo_lease)
  {
    // Reacquire an expired session from measured joints, never from an old RTC.
    terminate(updated.session_id, SessionState::ABORTED,
      {StatusCode::STALE_FEEDBACK, "Servo lease expired"});
    existing = sessions_.end();
  }
  const CommandClaim claim{
    updated.session_id, endpoint_name, updated.group_name,
    motion_kind(request), updated.joint_names};
  auto arbitration = arbiter_.updateServo(claim, received_at, now);
  if (!arbitration.status.ok()) {
    return {arbitration.status, updated.session_id};
  }
  if (existing != sessions_.end()) {
    existing->second.request = request;
    existing->second.submitted_at = received_at;
  } else {
    sessions_.emplace(updated.session_id, updated);
    servo_session_by_endpoint_[endpoint_name] = updated.session_id;
  }
  processPreemptions(arbitration.preempted_move_ids);
  for (const auto & expired : arbitration.expired_servo_ids) {
    if (expired != updated.session_id) {
      terminate(
        expired, SessionState::ABORTED,
        {StatusCode::STALE_FEEDBACK, "Servo lease expired"});
    }
  }
  if (arbitration.admitted) {
    revokeOutputs(claim.joint_names);
  }
  return {arbitration.status, updated.session_id};
}

bool CommandPipeline::validateCommandForPublication(
  const JointCommand & command, const SteadyTime now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return validatePublicationLocked(command, now);
}

void CommandPipeline::revokeOutputs(const std::vector<std::string> & joints)
{
  for (const auto & joint : joints) {
    publication_permits_.erase(joint);
  }
}

bool CommandPipeline::validatePublicationLocked(
  const JointCommand & command, const SteadyTime now)
{
  if (!command.publication_token || command.joint_names.empty()) {
    return false;
  }
  for (const auto & joint : command.joint_names) {
    const auto permit = publication_permits_.find(joint);
    if (permit == publication_permits_.end() ||
      permit->second != std::make_pair(command.session_id, command.publication_token))
    {
      return false;
    }
  }
  if (command.valid_until && now < *command.valid_until) {
    return true;
  }
  terminate(command.session_id, SessionState::ABORTED,
    {StatusCode::TIMEOUT, "command expired before publication"});
  return false;
}

bool CommandPipeline::publishCommand(
  const JointCommand & command, const SteadyTime now, const std::function<void()> & publish)
{
  const auto entered_at = steady_now_();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!validatePublicationLocked(command, now + (steady_now_() - entered_at))) {
    return false;
  }
  publish();
  revokeOutputs(command.joint_names);
  return true;
}

MotionStatus CommandPipeline::cancel(const std::string & session_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (sessions_.count(session_id) == 0U) {
    terminate(session_id, SessionState::CANCELED,
      {StatusCode::CANCELED, "session canceled"});
    // Cancellation and backend Stop are deliberately idempotent.
    return backend_ ? backend_->stopSession(session_id) : MotionStatus::Ok();
  }
  terminate(
    session_id, SessionState::CANCELED,
    {StatusCode::CANCELED, "session canceled"});
  return MotionStatus::Ok();
}

MotionStatus CommandPipeline::validateFeedback(
  const JointFeedback & feedback, const JointGroupModel & group,
  const SteadyTime now) const
{
  if (feedback.joint_names.size() != feedback.positions_rad.size() ||
    feedback.joint_names.size() != feedback.velocities_rad_s.size() ||
    !finite(feedback.positions_rad) || !finite(feedback.velocities_rad_s))
  {
    return {StatusCode::INVALID_ARGUMENT, "real feedback names/positions/velocities are invalid"};
  }
  const std::set<std::string> feedback_names(
    feedback.joint_names.begin(), feedback.joint_names.end());
  if (feedback_names.size() != feedback.joint_names.size() || feedback_names.count("") != 0U) {
    return {StatusCode::INVALID_ARGUMENT, "real feedback names must be non-empty and unique"};
  }
  if (now < feedback.received_at || now - feedback.received_at >= config_.feedback_max_age) {
    return {StatusCode::STALE_FEEDBACK, "real joint feedback is stale"};
  }
  if (!std::all_of(
      group.joint_names.begin(), group.joint_names.end(),
      [&feedback_names](const std::string & name) {
        return feedback_names.count(name) != 0U;
      }))
  {
    return {StatusCode::INVALID_ARGUMENT, "real feedback is missing commanded joints"};
  }
  for (std::size_t i = 0; i < group.joint_names.size(); ++i) {
    const auto source = std::find(
      feedback.joint_names.begin(), feedback.joint_names.end(), group.joint_names[i]);
    const double q = feedback.positions_rad[std::distance(feedback.joint_names.begin(), source)];
    if (q < group.lower_position_rad[i] - config_.feedback_limit_recovery_margin_rad ||
      q > group.upper_position_rad[i] + config_.feedback_limit_recovery_margin_rad)
    {
      std::ostringstream message;
      message << std::setprecision(9) << "measured joint outside limit recovery range: "
              << group.joint_names[i] << " measured=" << q << " limits=["
              << group.lower_position_rad[i] << "," << group.upper_position_rad[i]
              << "] recovery_margin=" << config_.feedback_limit_recovery_margin_rad;
      return {StatusCode::LIMIT_VIOLATION, message.str()};
    }
  }
  return MotionStatus::Ok();
}

MotionStatus CommandPipeline::validateCandidate(
  const JointCommand & command, const JointGroupModel & group) const
{
  if (command.passed_final_sdk_rtc) {
    return {StatusCode::INTERNAL_ERROR, "candidate was incorrectly marked as final RTC output"};
  }
  return validateFinalCommand(command, group);
}

MotionStatus CommandPipeline::validateFinalCommand(
  const JointCommand & command, const JointGroupModel & group,
  const std::vector<double> * previous_positions,
  const JointFeedback * feedback) const
{
  if (command.group_name != group.name ||
    command.joint_names != group.joint_names ||
    command.positions_rad.size() != group.joint_names.size() ||
    (!command.velocities_rad_s.empty() &&
    command.velocities_rad_s.size() != group.joint_names.size()) ||
    (!command.accelerations_rad_s2.empty() &&
    command.accelerations_rad_s2.size() != group.joint_names.size()) ||
    !finite(command.positions_rad) || !finite(command.velocities_rad_s) ||
    !finite(command.accelerations_rad_s2))
  {
    return {StatusCode::INVALID_ARGUMENT, "SDK command names/lengths/values are invalid"};
  }
  for (std::size_t index = 0; index < command.positions_rad.size(); ++index) {
    if (command.positions_rad[index] < group.lower_position_rad[index] ||
      command.positions_rad[index] > group.upper_position_rad[index])
    {
      // Candidates and requested targets still obey the model limits strictly.
      // A final RTC output may transiently remain outside only when BOTH the
      // measured joint and the previous output are already outside that same
      // boundary, and this output moves monotonically inward. Never clamp the
      // feedback or jump the command to the boundary, bypassing the final RTC.
      if (command.passed_final_sdk_rtc && previous_positions && feedback &&
        previous_positions->size() == command.positions_rad.size() &&
        feedback->positions_rad.size() == command.positions_rad.size())
      {
        const double position = command.positions_rad[index];
        const double previous = (*previous_positions)[index];
        const double measured = feedback->positions_rad[index];
        const double velocity = command.velocities_rad_s.empty() ? 0.0 :
          command.velocities_rad_s[index];
        const double lower = group.lower_position_rad[index];
        const double upper = group.upper_position_rad[index];
        if ((position < lower && previous < lower && measured < lower &&
          position >= previous && position >= measured && velocity >= 0.0) ||
          (position > upper && previous > upper && measured > upper &&
          position <= previous && position <= measured && velocity <= 0.0))
        {
          continue;
        }
      }
      std::ostringstream message;
      message << std::setprecision(9) << "SDK command violates model limit: "
              << group.joint_names[index]
              << " stage=" << (command.passed_final_sdk_rtc ? "final_rtc" : "candidate")
              << " position=" << command.positions_rad[index] << " limits=["
              << group.lower_position_rad[index] << "," << group.upper_position_rad[index] << "]";
      if (previous_positions && feedback) {
        message << " previous=" << previous_positions->at(index)
                << " measured=" << feedback->positions_rad.at(index);
      }
      return {StatusCode::LIMIT_VIOLATION, message.str()};
    }
  }
  return MotionStatus::Ok();
}

std::optional<JointTarget> CommandPipeline::limitRecoverySeed(
  const Session & session, const JointCommand & output,
  const JointFeedback & feedback) const
{
  // Called only for a structurally valid final output whose limit check failed.
  // A recovery state is not permission to retreat or create a new violation.
  if (!output.passed_final_sdk_rtc || !session.recovering_limits) {
    return std::nullopt;
  }
  auto seed = session.last_final_target;
  const auto & group = groups_.at(session.group_name);
  const auto & velocity_limits = request_limits(session.request).joint_max_velocity_rad_s;
  bool changed = false;
  for (std::size_t i = 0; i < group.joint_names.size(); ++i) {
    const double q = output.positions_rad[i];
    const double previous = seed.positions_rad[i];
    const double measured = feedback.positions_rad[i];
    const double velocity = output.velocities_rad_s.empty() ? 0.0 : output.velocities_rad_s[i];
    const double lower = group.lower_position_rad[i];
    const double upper = group.upper_position_rad[i];
    if (q >= lower && q <= upper) {
      continue;
    }
    const bool lower_recovery = q < lower && previous < lower && q >= previous && velocity >= 0.0;
    const bool upper_recovery = q > upper && previous > upper && q <= previous && velocity <= 0.0;
    if (!lower_recovery && !upper_recovery) {
      return std::nullopt;
    }
    const bool feedback_ahead = lower_recovery ? measured > q : measured < q;
    if (!feedback_ahead) {
      continue;
    }
    const double max_step = config_.feedback_rebase_tolerance_rad +
      (velocity_limits.empty() ? 0.0 : velocity_limits[i] * current_period_sec_);
    if (std::abs(measured - previous) > max_step || measured < lower -
      config_.feedback_limit_recovery_margin_rad || measured > upper +
      config_.feedback_limit_recovery_margin_rad ||
      (lower_recovery && measured > upper) || (upper_recovery && measured < lower))
    {
      return std::nullopt;
    }
    // Only this joint is synchronized to raw measured q/v. Other joints retain
    // their last emitted position, velocity and acceleration, not lagging feedback.
    seed.positions_rad[i] = measured;
    seed.velocities_rad_s[i] = feedback.velocities_rad_s[i];
    seed.accelerations_rad_s2[i] = 0.0;
    changed = true;
  }
  return changed ? std::optional<JointTarget>(std::move(seed)) : std::nullopt;
}

JointFeedback CommandPipeline::selectFeedback(
  const JointFeedback & feedback, const std::vector<std::string> & names) const
{
  std::map<std::string, std::size_t> source;
  for (std::size_t index = 0; index < feedback.joint_names.size(); ++index) {
    source.emplace(feedback.joint_names[index], index);
  }
  JointFeedback selected;
  selected.joint_names = names;
  selected.received_at = feedback.received_at;
  for (const auto & name : names) {
    const auto index = source.at(name);
    selected.positions_rad.push_back(feedback.positions_rad[index]);
    selected.velocities_rad_s.push_back(feedback.velocities_rad_s[index]);
  }
  return selected;
}

DynamicTarget CommandPipeline::dynamicTarget(const Session & session) const
{
  return std::visit(
    [](const auto & request) -> DynamicTarget {
      using RequestT = std::decay_t<decltype(request)>;
      if constexpr (std::is_same_v<RequestT, MoveJRequest>||
      std::is_same_v<RequestT, ServoJRequest>)
      {
        return request.target;
      } else if constexpr (std::is_same_v<RequestT, MovePRequest>) {
        return request.waypoints.back();
      } else {
        return request.target;
      }
    }, session.request);
}

JointTarget CommandPipeline::terminalJointGoal(const Session & session) const
{
  if (const auto * request = std::get_if<MoveJRequest>(&session.request)) {
    return request->target;
  }
  return {};
}

std::optional<Pose> CommandPipeline::terminalCartesianGoal(const Session & session) const
{
  if (const auto * request = std::get_if<MoveLRequest>(&session.request)) {
    return request->target;
  }
  if (const auto * request = std::get_if<MovePRequest>(&session.request)) {
    return request->waypoints.back();
  }
  return std::nullopt;
}

ForwardKinematicsResult CommandPipeline::feedbackPose(
  const Session & session, const JointFeedback & feedback)
{
  ForwardKinematicsRequest request;
  request.group_name = session.group_name;
  request.joints.joint_names = session.joint_names;
  const auto selected = selectFeedback(feedback, session.joint_names);
  request.joints.positions_rad = selected.positions_rad;
  std::visit(
    [&request](const auto & value) {
      using RequestT = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<RequestT, MoveLRequest>||
      std::is_same_v<RequestT, MovePRequest>)
      {
        request.base_link = value.base_link;
        request.link_name = value.link_name;
      }
    }, session.request);
  return backend_->forwardKinematics(request);
}

void CommandPipeline::processPreemptions(
  const std::vector<std::string> & session_ids)
{
  for (const auto & id : session_ids) {
    terminate(
      id, SessionState::PREEMPTED,
      {StatusCode::PREEMPTED, "Move was preempted and will not resume"});
  }
}

void CommandPipeline::terminate(
  const std::string & session_id, const SessionState state,
  const MotionStatus & status, const bool preserve_output)
{
  if (!preserve_output) {
    for (auto it = publication_permits_.begin(); it != publication_permits_.end(); ) {
      if (it->second.first == session_id) {
        it = publication_permits_.erase(it);
      } else {
        ++it;
      }
    }
  }
  const auto found = sessions_.find(session_id);
  if (found == sessions_.end()) {
    return;
  }
  backend_->stopSession(session_id);
  active_backend_sessions_.erase(session_id);
  arbiter_.release(session_id);
  monitor_.erase(session_id);
  const auto source = last_source_by_group_.find(found->second.group_name);
  if (source != last_source_by_group_.end() && source->second == session_id) {
    last_source_by_group_.erase(source);
  }
  const auto servo = servo_session_by_endpoint_.find(found->second.endpoint_name);
  if (servo != servo_session_by_endpoint_.end() && servo->second == session_id) {
    servo_session_by_endpoint_.erase(servo);
  }
  pending_events_.push_back({session_id, state, status});
  sessions_.erase(found);
}

std::optional<JointCommand> CommandPipeline::controlledStop(
  const Session & session, const JointFeedback & feedback,
  const MotionStatus &)
{
  try {
    const auto selected = selectFeedback(feedback, session.joint_names);
    const auto reset = backend_->resetFinalJointTarget(session.group_name, selected);
    if (!reset.ok()) {
      return std::nullopt;
    }
    JointCommand hold;
    hold.group_name = session.group_name;
    hold.joint_names = session.joint_names;
    hold.positions_rad = selected.positions_rad;
    hold.velocities_rad_s.assign(session.joint_names.size(), 0.0);
    hold.accelerations_rad_s2.assign(session.joint_names.size(), 0.0);
    auto shaped = backend_->updateFinalJointTarget(
      session.group_name, hold, current_period_sec_);
    if (!shaped.status.ok() || !shaped.candidate.passed_final_sdk_rtc) {
      return std::nullopt;
    }
    const auto valid = validateFinalCommand(shaped.candidate, groups_.at(session.group_name));
    return valid.ok() ? std::optional<JointCommand>(std::move(shaped.candidate)) : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

PipelineTick CommandPipeline::tick(
  const JointFeedback & feedback, const SteadyTime now)
{
  const auto entered_at = steady_now_();
  const auto current_time = [&]() {return now + (steady_now_() - entered_at);};
  std::lock_guard<std::mutex> lock(mutex_);
  const auto started_at = current_time();
  const double max_period_sec = std::max(
    period_sec_, std::chrono::duration<double>(config_.max_control_period).count());
  current_period_sec_ = period_sec_;
  bool scheduling_gap = false;
  if (last_tick_time_ && now > *last_tick_time_) {
    const auto elapsed = std::chrono::duration<double>(now - *last_tick_time_).count();
    if (elapsed > 0.0 && std::isfinite(elapsed)) {
      scheduling_gap = elapsed > max_period_sec;
      current_period_sec_ = scheduling_gap ? period_sec_ : elapsed;
    }
  }
  last_tick_time_ = now;
  PipelineTick result;
  result.events.swap(pending_events_);
  auto arbitration = arbiter_.evaluate(started_at);
  for (const auto & expired : arbitration.expired_servo_ids) {
    terminate(
      expired, SessionState::ABORTED,
      {StatusCode::STALE_FEEDBACK, "Servo lease expired"});
  }
  if (scheduling_gap) {
    const std::vector<std::string> active(
      active_backend_sessions_.begin(), active_backend_sessions_.end());
    for (const auto & id : active) {
      auto & session = sessions_.at(id);
      if (is_move(motion_kind(session.request))) {
        terminate(id, SessionState::ABORTED,
          {StatusCode::TIMEOUT, "control scheduling gap exceeded maximum period"});
      } else {
        backend_->stopSession(id);
        active_backend_sessions_.erase(id);
        last_source_by_group_.erase(session.group_name);
        session.backend_started = false;
      }
    }
  }
  const auto cycle_deadline = started_at + std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(max_period_sec));
  const auto deadline = [&](const Session & session) {
      auto until = std::min(feedback.received_at + config_.feedback_max_age, cycle_deadline);
      if (is_servo(motion_kind(session.request))) {
        until = std::min(until,
          session.submitted_at + endpoints_.at(session.endpoint_name).servo_lease);
      }
      return until;
    };
  const auto check_time = [&](const std::string & id) {
      const auto session = sessions_.find(id);
      if (session == sessions_.end()) {
        return false;
      }
      if (current_time() >= deadline(session->second)) {
        terminate(id, SessionState::ABORTED,
          {StatusCode::TIMEOUT, "control computation exceeded input/output deadline"});
        return false;
      }
      return true;
    };
  const auto append_command = [&](const Session & session, JointCommand command) {
      command.session_id = session.session_id;
      command.valid_until = deadline(session);
      command.publication_token = next_publication_token_++;
      for (const auto & joint : command.joint_names) {
        publication_permits_[joint] = {command.session_id, command.publication_token};
      }
      result.commands.push_back(std::move(command));
    };
  const auto fail_backend = [&](const std::string & id, const MotionStatus & status) {
      const auto & session = sessions_.at(id);
      if (is_servo(motion_kind(session.request))) {
        // Keep accepting/overwriting targets while the failed SDK session is
        // stopped. A retry uses the latest target and fresh measured joints.
        servo_retry_after_[session.endpoint_name] = current_time() + config_.servo_retry_interval;
      }
      terminate(id, SessionState::ABORTED, status, true);
    };
  result.events.insert(
    result.events.end(), pending_events_.begin(), pending_events_.end());
  pending_events_.clear();

  const std::set<std::string> winner_set(
    arbitration.winner_session_ids.begin(), arbitration.winner_session_ids.end());
  std::vector<std::string> deactivate;
  for (const auto & active : active_backend_sessions_) {
    if (winner_set.count(active) == 0U) {
      deactivate.push_back(active);
    }
  }
  for (const auto & id : deactivate) {
    backend_->stopSession(id);
    active_backend_sessions_.erase(id);
    const auto session = sessions_.find(id);
    if (session != sessions_.end()) {
      session->second.backend_started = false;
      const auto source = last_source_by_group_.find(session->second.group_name);
      if (source != last_source_by_group_.end() && source->second == id) {
        last_source_by_group_.erase(source);
      }
    }
  }

  for (const auto & id : arbitration.winner_session_ids) {
    auto found = sessions_.find(id);
    if (found == sessions_.end()) {
      continue;
    }
    const auto feedback_status = validateFeedback(
      feedback, groups_.at(found->second.group_name), current_time());
    if (!feedback_status.ok()) {
      // Invalid or stale measurements cannot seed a fresh hold command either.
      terminate(id, SessionState::ABORTED, feedback_status);
      continue;
    }
    if (!check_time(id)) {
      continue;
    }
    const auto selected_feedback = selectFeedback(feedback, found->second.joint_names);
    const auto retry = servo_retry_after_.find(found->second.endpoint_name);
    if (is_servo(motion_kind(found->second.request)) && !found->second.backend_started &&
      retry != servo_retry_after_.end() && current_time() < retry->second)
    {
      continue;
    }

    if (is_move(motion_kind(found->second.request)) &&
      now - found->second.submitted_at > config_.goal_monitor.move_timeout)
    {
      const MotionStatus timeout{StatusCode::TIMEOUT, "Move exceeded its steady-clock timeout"};
      const auto stop = controlledStop(found->second, selected_feedback, timeout);
      if (stop) {
        append_command(found->second, *stop);
      }
      terminate(id, SessionState::ABORTED, timeout, true);
      continue;
    }

    if (!found->second.backend_started) {
      const auto start = backend_->startSession(
        id, found->second.request, selected_feedback, current_period_sec_);
      if (!check_time(id)) {
        continue;
      }
      if (!start.ok()) {
        const auto stop = controlledStop(found->second, selected_feedback, start);
        if (stop) {
          append_command(found->second, *stop);
        }
        fail_backend(id, start);
        continue;
      }
      const auto reset = backend_->resetFinalJointTarget(
        found->second.group_name, selected_feedback);
      if (!check_time(id)) {
        continue;
      }
      if (!reset.ok()) {
        const auto stop = controlledStop(found->second, selected_feedback, reset);
        if (stop) {
          append_command(found->second, *stop);
        }
        fail_backend(id, reset);
        continue;
      }
      found->second.backend_started = true;
      found->second.last_final_target = {
        selected_feedback.joint_names, selected_feedback.positions_rad,
        selected_feedback.velocities_rad_s,
        std::vector<double>(selected_feedback.joint_names.size(), 0.0)};
      found->second.recovering_limits = false;
      const auto & model = groups_.at(found->second.group_name);
      for (std::size_t i = 0; i < model.joint_names.size(); ++i) {
        found->second.recovering_limits |=
          selected_feedback.positions_rad[i] < model.lower_position_rad[i] ||
          selected_feedback.positions_rad[i] > model.upper_position_rad[i];
      }
      active_backend_sessions_.insert(id);
      last_source_by_group_[found->second.group_name] = id;
      result.events.push_back({id, SessionState::RUNNING, {StatusCode::OK,
        found->second.recovering_limits ? "joint limit recovery started" : ""}});
      if (motion_kind(found->second.request) == MotionKind::MOVE_J) {
        const auto monitor_status = monitor_.begin(
          id, terminalJointGoal(found->second), std::nullopt,
          found->second.submitted_at);
        if (!monitor_status.ok()) {
          terminate(id, SessionState::ABORTED, monitor_status);
          continue;
        }
      }
    }

    const auto target = dynamicTarget(found->second);
    auto backend_tick = backend_->tickSession(id, &target, current_period_sec_);
    if (!check_time(id)) {
      continue;
    }
    if (!backend_tick.status.ok()) {
      const auto stop = controlledStop(found->second, selected_feedback, backend_tick.status);
      if (stop) {
        append_command(found->second, *stop);
      }
      fail_backend(id, backend_tick.status);
      continue;
    }
    const auto group = groups_.find(found->second.group_name);
    const auto candidate_status = validateCandidate(backend_tick.candidate, group->second);
    if (!candidate_status.ok()) {
      const auto stop = controlledStop(found->second, selected_feedback, candidate_status);
      if (stop) {
        append_command(found->second, *stop);
      }
      fail_backend(id, candidate_status);
      continue;
    }

    // This is the sole path that produces a normal publishable command.
    auto final_tick = backend_->updateFinalJointTarget(
      found->second.group_name, backend_tick.candidate, current_period_sec_);
    if (!check_time(id)) {
      continue;
    }
    if (!final_tick.status.ok()) {
      const auto stop = controlledStop(found->second, selected_feedback, final_tick.status);
      if (stop) {
        append_command(found->second, *stop);
      }
      fail_backend(id, final_tick.status);
      continue;
    }
    auto final_status = validateFinalCommand(
      final_tick.candidate, group->second, &found->second.last_final_target.positions_rad,
      &selected_feedback);
    if (final_status.code == StatusCode::LIMIT_VIOLATION) {
      const auto seed = limitRecoverySeed(found->second, final_tick.candidate, selected_feedback);
      if (seed) {
        final_status = backend_->rebaseFinalJointTarget(found->second.group_name, *seed);
        if (final_status.ok()) {
          // Exactly one retry. Never publish the rejected output or bypass RTC.
          final_tick = backend_->updateFinalJointTarget(
            found->second.group_name, backend_tick.candidate, current_period_sec_);
          final_status = final_tick.status;
          if (final_status.ok()) {
            final_status = validateFinalCommand(
              final_tick.candidate, group->second, &seed->positions_rad, &selected_feedback);
          }
        }
      }
    }
    if (!final_status.ok() || !final_tick.candidate.passed_final_sdk_rtc) {
      const MotionStatus failure = final_status.ok() ? MotionStatus{
        StatusCode::INTERNAL_ERROR, "backend output bypassed final SDK RTC"} : final_status;
      const auto stop = controlledStop(found->second, selected_feedback, failure);
      if (stop) {
        append_command(found->second, *stop);
      }
      fail_backend(id, failure);
      continue;
    }
    append_command(found->second, final_tick.candidate);
    servo_retry_after_.erase(found->second.endpoint_name);
    found->second.last_final_target = {
      final_tick.candidate.joint_names, final_tick.candidate.positions_rad,
      final_tick.candidate.velocities_rad_s, final_tick.candidate.accelerations_rad_s2};
    auto & last = found->second.last_final_target;
    if (last.velocities_rad_s.empty()) {
      last.velocities_rad_s.assign(last.joint_names.size(), 0.0);
    }
    if (last.accelerations_rad_s2.empty()) {
      last.accelerations_rad_s2.assign(last.joint_names.size(), 0.0);
    }
    if (found->second.recovering_limits &&
      validateFinalCommand(final_tick.candidate, group->second).ok())
    {
      found->second.recovering_limits = false;
      result.events.push_back({id, SessionState::RUNNING,
        {StatusCode::OK, "joint limit recovery completed"}});
    }

    if (is_servo(motion_kind(found->second.request))) {
      continue;
    }
    if (!monitor_.contains(id) && backend_tick.path_reached) {
      JointTarget joint_goal;
      joint_goal.joint_names = backend_tick.candidate.joint_names;
      joint_goal.positions_rad = backend_tick.candidate.positions_rad;
      const auto monitor_status = monitor_.begin(
        id, joint_goal, terminalCartesianGoal(found->second),
        found->second.submitted_at);
      if (!monitor_status.ok()) {
        terminate(id, SessionState::ABORTED, monitor_status);
        continue;
      }
    }
    if (!monitor_.contains(id)) {
      continue;
    }
    std::optional<Pose> fk_pose;
    if (terminalCartesianGoal(found->second)) {
      const auto fk_result = feedbackPose(found->second, feedback);
      if (!fk_result.status.ok()) {
        result.commands.pop_back();
        const auto fk_error = fk_result.status;
        const auto stop = controlledStop(found->second, selected_feedback, fk_error);
        if (stop) {
          append_command(found->second, *stop);
        }
        terminate(id, SessionState::ABORTED, fk_error, true);
        continue;
      }
      fk_pose = fk_result.pose;
    }
    const auto observation = monitor_.observe(id, feedback, fk_pose, now);
    if (observation.observation == GoalObservation::REACHED) {
      terminate(id, SessionState::SUCCEEDED, MotionStatus::Ok(), true);
    } else if (observation.observation == GoalObservation::TIMED_OUT ||
      observation.observation == GoalObservation::INVALID_FEEDBACK)
    {
      result.commands.pop_back();
      const auto stop = controlledStop(found->second, selected_feedback, observation.status);
      if (stop) {
        append_command(found->second, *stop);
      }
      terminate(id, SessionState::ABORTED, observation.status, true);
    }
  }

  // A later arm's solver can make an earlier arm's result obsolete. Recheck
  // the entire batch; the ROS adapter also checks immediately before publish.
  const auto finished_at = current_time();
  for (auto it = result.commands.begin(); it != result.commands.end(); ) {
    if (finished_at >= *it->valid_until) {
      terminate(it->session_id, SessionState::ABORTED,
        {StatusCode::TIMEOUT, "command expired while computing another session"});
      it = result.commands.erase(it);
    } else {
      ++it;
    }
  }
  result.events.insert(
    result.events.end(), pending_events_.begin(), pending_events_.end());
  pending_events_.clear();
  return result;
}

}  // namespace humanoid_motion_server::motion
