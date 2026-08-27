#ifndef HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_
#define HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_

#include <chrono>
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
    CommandPipelineConfig config = {});

  MotionStatus registerEndpoint(const EndpointPolicy & policy);

  SubmissionResult submitMove(
    const std::string & endpoint_name, const SessionRequest & request,
    SteadyTime now);
  SubmissionResult updateServo(
    const std::string & endpoint_name, const SessionRequest & request,
    SteadyTime now);

  MotionStatus cancel(const std::string & session_id);
  PipelineTick tick(const JointFeedback & feedback, SteadyTime now);

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
  };

  MotionStatus validatePipelineConfig() const;
  MotionStatus validateRequest(
    const EndpointPolicy & policy, const SessionRequest & request,
    bool expect_servo, Session * session) const;
  MotionStatus validateFeedback(
    const JointFeedback & feedback, const std::vector<std::string> & required,
    SteadyTime now) const;
  MotionStatus validateCandidate(
    const JointCommand & command, const JointGroupModel & group) const;
  MotionStatus validateFinalCommand(
    const JointCommand & command, const JointGroupModel & group) const;
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
    const MotionStatus & status);
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
  ControlArbiter arbiter_;
  GoalMonitor monitor_;
  std::map<std::string, Session> sessions_;
  std::map<std::string, std::string> servo_session_by_endpoint_;
  std::map<std::string, std::string> last_source_by_group_;
  std::set<std::string> active_backend_sessions_;
  std::vector<SessionEvent> pending_events_;
  mutable std::mutex mutex_;
};

}  // namespace humanoid_motion_server::motion

#endif  // HUMANOID_MOTION_SERVER__COMMAND_PIPELINE_HPP_
