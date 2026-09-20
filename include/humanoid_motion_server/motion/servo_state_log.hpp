#ifndef HUMANOID_MOTION_SERVER__SERVO_STATE_LOG_HPP_
#define HUMANOID_MOTION_SERVER__SERVO_STATE_LOG_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion
{

// Observational only. Coalesce start+failure within one tick, so retrying the
// same failed ServoP at 100 Hz does not look like repeated recoveries.
class ServoStateLog
{
public:
  std::vector<std::string> update(const PipelineTick & tick, const SteadyTime now)
  {
    std::map<std::string, std::string> outcomes;
    for (const auto & event : tick.events) {
      if (event.session_id.rfind("servo:", 0) != 0 || event.state == SessionState::PENDING) {
        continue;
      }
      std::string outcome;
      switch (event.state) {
        case SessionState::RUNNING: outcome = "running"; break;
        case SessionState::ABORTED: outcome = "stopped"; break;
        case SessionState::CANCELED: outcome = "canceled"; break;
        case SessionState::PREEMPTED: outcome = "preempted"; break;
        case SessionState::SUCCEEDED: outcome = "succeeded"; break;
        default: continue;
      }
      if (!event.status.message.empty()) {
        outcome += ": " + event.status.message;
      }
      if (!event.status.ok()) {
        outcome += "; code=" + std::to_string(static_cast<int>(event.status.code));
        if (!event.status.sdk_api.empty()) {
          outcome += "; api=" + event.status.sdk_api;
        }
      }
      outcomes[event.session_id] = outcome;
    }
    for (const auto & [id, outcome] : outcomes) {
      auto & state = states_[id];
      if (state.current != outcome) {
        state.current = outcome;
        ++state.changes;
        if (state.path.size() < 8U) {
          state.path.push_back(outcome);
        } else {
          state.path.back() = outcome;
        }
      }
    }
    std::vector<std::string> messages;
    for (auto & [id, state] : states_) {
      if (!state.changes ||
        (state.have_log && now - state.last_log < std::chrono::seconds(1)))
      {
        continue;
      }
      auto message = "motion." + id + ": " + state.logged;
      for (const auto & step : state.path) {
        message += " -> " + step;
      }
      message += "; changes=" + std::to_string(state.changes);
      messages.push_back(message);
      state.logged = state.current;
      state.path.clear();
      state.changes = 0;
      state.last_log = now;
      state.have_log = true;
    }
    return messages;
  }

private:
  struct State
  {
    std::string current;
    std::string logged{"unknown"};
    std::vector<std::string> path;
    std::uint64_t changes{0};
    SteadyTime last_log{};
    bool have_log{false};
  };
  // Servo session IDs are derived from configured endpoints, not packet IDs.
  std::map<std::string, State> states_;
};

}  // namespace humanoid_motion_server::motion
#endif
