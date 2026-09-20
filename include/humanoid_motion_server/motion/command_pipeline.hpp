#ifndef HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_
#define HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "humanoid_motion_server/arbitration/control_arbiter.hpp"
#include "humanoid_motion_server/motion/goal_monitor.hpp"
#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"
#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion
{

struct CommandPipelineConfig
{
  double control_frequency_hz{100.0};
  std::chrono::milliseconds feedback_max_age{100};
  GoalMonitorConfig goal_monitor{};
  // Feedback outside this recovery envelope is a fault, never a hold target.
  double feedback_limit_recovery_margin_rad{0.1};
  // Measurement resolution allowance when bounding a recovery-state rebase.
  double feedback_rebase_tolerance_rad{0.001};
  // A long scheduling gap must not become one large RTC integration step.
  std::chrono::milliseconds max_control_period{100};
  std::chrono::milliseconds servo_retry_interval{50};
};

struct SubmissionResult
{
  MotionStatus status;
  std::string session_id;
};

/// The SDK scheduling/session layer. No ROS type crosses this API and every
/// returned JointCommand has passed the backend's independent final joint RTC.
class CommandPipeline
{
public:
  CommandPipeline(
    SdkMotionBackendPtr backend, std::vector<JointGroupModel> joint_groups,
    CommandPipelineConfig config = {},
    std::function<SteadyTime()> steady_now = SteadyClock::now);

  MotionStatus registerEndpoint(const EndpointPolicy & policy);

  SubmissionResult submitMove(
    const std::string & endpoint_name, const SessionRequest & request,
    SteadyTime now);
  SubmissionResult updateServo(
    const std::string & endpoint_name, const SessionRequest & request,
    SteadyTime now)
  {
    return updateServo(endpoint_name, request, now, now);
  }
  // Deferred inputs must supply BOTH their original freshness time and the
  // current processing time. Rejected inputs have no arbitration side effects.
  SubmissionResult updateServo(
    const std::string & endpoint_name, const SessionRequest & request,
    SteadyTime received_at, SteadyTime now);

  MotionStatus cancel(const std::string & session_id);
  PipelineTick tick(const JointFeedback & feedback, SteadyTime now);
  bool validateCommandForPublication(const JointCommand & command, SteadyTime now);
  // The sink must not call back into this pipeline. Validation and publication
  // share the cancellation/arbitration lock, so revocation cannot race the sink.
  bool publishCommand(const JointCommand & command, SteadyTime now,
    const std::function<void()> & publish);

private:
  struct Session
  {
    std::string endpoint_name;
    std::string session_id;
    std::string group_name;
    std::vector<std::string> joint_names;
    SessionRequest request;
    SteadyTime submitted_at{};
    bool backend_started{false};
    // Final RTC starts from measured joints, which can already lie outside a
    // model limit. Retain the last output to permit only inward recovery.
    JointTarget last_final_target;
    bool recovering_limits{false};
  };

  MotionStatus validatePipelineConfig() const;
  MotionStatus validateRequest(
    const EndpointPolicy & policy, const SessionRequest & request,
    bool expect_servo, Session * session) const;
  MotionStatus validateFeedback(
    const JointFeedback & feedback, const JointGroupModel & group,
    SteadyTime now) const;
  MotionStatus validateCandidate(
    const JointCommand & command, const JointGroupModel & group) const;
  MotionStatus validateFinalCommand(
    const JointCommand & command, const JointGroupModel & group,
    const std::vector<double> * previous_positions = nullptr,
    const JointFeedback * feedback = nullptr) const;
  std::optional<JointTarget> limitRecoverySeed(
    const Session & session, const JointCommand & output,
    const JointFeedback & feedback) const;
  JointFeedback selectFeedback(
    const JointFeedback & feedback, const std::vector<std::string> & names) const;
  DynamicTarget dynamicTarget(const Session & session) const;
  JointTarget terminalJointGoal(const Session & session) const;
  std::optional<Pose> terminalCartesianGoal(const Session & session) const;
  ForwardKinematicsResult feedbackPose(
    const Session & session, const JointFeedback & feedback);

  void processPreemptions(const std::vector<std::string> & session_ids);
  void terminate(
    const std::string & session_id, SessionState state,
    const MotionStatus & status, bool preserve_output = false);
  bool validatePublicationLocked(const JointCommand & command, SteadyTime now);
  void revokeOutputs(const std::vector<std::string> & joints);
  std::optional<JointCommand> controlledStop(
    const Session & session, const JointFeedback & feedback,
    const MotionStatus & cause);

  SdkMotionBackendPtr backend_;
  std::map<std::string, JointGroupModel> groups_;
  std::map<std::string, EndpointPolicy> endpoints_;
  CommandPipelineConfig config_;
  MotionStatus configuration_status_;
  double period_sec_{0.01};
  double current_period_sec_{0.01};
  std::optional<SteadyTime> last_tick_time_;
  std::function<SteadyTime()> steady_now_;
  ControlArbiter arbiter_;
  GoalMonitor monitor_;
  std::map<std::string, Session> sessions_;
  std::map<std::string, std::string> servo_session_by_endpoint_;
  std::map<std::string, SteadyTime> servo_retry_after_;
  std::map<std::string, std::string> last_source_by_group_;
  std::set<std::string> active_backend_sessions_;
  std::vector<SessionEvent> pending_events_;
  // At most one permit per configured joint; completed sessions do not accumulate.
  std::map<std::string, std::pair<std::string, std::uint64_t>> publication_permits_;
  std::uint64_t next_publication_token_{1};
  mutable std::mutex mutex_;
};

}  // namespace humanoid_motion_server::motion

#endif  // HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_
