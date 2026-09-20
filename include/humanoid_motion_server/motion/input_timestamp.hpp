#ifndef HUMANOID_MOTION_SERVER__INPUT_TIMESTAMP_HPP_
#define HUMANOID_MOTION_SERVER__INPUT_TIMESTAMP_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion
{

// One tracker per input topic, independent of whether its latest value has
// already been consumed. Replayed packets cannot renew a lease.
class InputTimestamp
{
public:
  bool accept(
    const std::int64_t stamp_ns, const std::int64_t ros_now_ns,
    const SteadyTime received_at, const std::chrono::nanoseconds max_age,
    const std::chrono::nanoseconds future_tolerance, const bool require_stamp,
    SteadyTime & fresh_at, std::string & error)
  {
    if (ros_now_ns < 0 || stamp_ns < 0) {
      error = "negative input timestamp";
      return false;
    }
    if (ros_now_ns < last_ros_now_ns_) {
      // Permit a deliberate ROS/simulation clock reset without accepting
      // out-of-order packets during normal, monotonic ROS time.
      last_stamp_ns_ = 0;
    }
    last_ros_now_ns_ = ros_now_ns;
    if (stamp_ns == 0) {
      if (require_stamp) {
        error = "feedback requires a non-zero source timestamp";
        return false;
      }
      fresh_at = received_at;  // Legacy unstamped Servo: receipt ordering only.
      return true;
    }
    const auto age = std::chrono::nanoseconds(ros_now_ns - stamp_ns);
    if (age >= max_age || age < -future_tolerance) {
      error = "input source timestamp is stale or too far in the future";
      return false;
    }
    if (stamp_ns <= last_stamp_ns_) {
      error = "duplicate or out-of-order input source timestamp";
      return false;
    }
    last_stamp_ns_ = stamp_ns;
    fresh_at = received_at - std::max(age, std::chrono::nanoseconds::zero());
    return true;
  }

private:
  std::int64_t last_stamp_ns_{0};
  std::int64_t last_ros_now_ns_{0};
};

}  // namespace humanoid_motion_server::motion

#endif
