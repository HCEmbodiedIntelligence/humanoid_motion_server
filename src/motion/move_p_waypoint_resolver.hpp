#ifndef MOVE_P_WAYPOINT_RESOLVER_HPP_
#define MOVE_P_WAYPOINT_RESOLVER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "humanoid_motion_server/motion/types.hpp"

namespace humanoid_motion_server::motion::detail
{

struct MovePWaypointResolution
{
  MotionStatus status;
  std::vector<JointTarget> joint_waypoints;
};

inline humanoid_motion_server::kinematics::Pose toKinematicsPose(const Pose & input)
{
  humanoid_motion_server::kinematics::Pose output;
  output.position_m = {
    input.position_m[0], input.position_m[1], input.position_m[2]};
  output.orientation = {
    input.orientation_xyzw[0], input.orientation_xyzw[1],
    input.orientation_xyzw[2], input.orientation_xyzw[3]};
  return output;
}

inline MovePWaypointResolution resolveMovePWaypoints(
  const MovePRequest & request, JointTarget seed,
  const std::vector<std::string> & expected_joint_names,
  humanoid_motion_server::kinematics::IKinematics & kinematics)
{
  MovePWaypointResolution output;
  for (const auto & waypoint : request.waypoints) {
    humanoid_motion_server::kinematics::InverseKinematicsRequest ik_request;
    ik_request.kinematics.group_name = request.group_name;
    ik_request.kinematics.base_link = request.base_link;
    ik_request.kinematics.link_name = request.link_name;
    ik_request.target_pose = toKinematicsPose(waypoint);
    humanoid_motion_server::kinematics::JointState ik_seed;
    ik_seed.joint_names = seed.joint_names;
    ik_seed.positions_rad = seed.positions_rad;
    ik_request.seed = std::move(ik_seed);

    const auto ik_result = kinematics.inverseKinematics(ik_request);
    if (!ik_result.ok()) {
      output.status = {
        StatusCode::SDK_ERROR, ik_result.status.message,
        "humanoid_motion_server::kinematics::IKinematics::inverseKinematics",
        static_cast<std::int64_t>(ik_result.status.code)};
      return output;
    }

    std::map<std::string, double> positions;
    const auto & solution = ik_result.value->solution;
    if (solution.joint_names.size() != solution.positions_rad.size()) {
      output.status = {
        StatusCode::SDK_ERROR,
        "SDK kinematics adapter returned a joint name/value length mismatch",
        "humanoid_motion_server::kinematics::IKinematics::inverseKinematics", -1};
      return output;
    }
    for (std::size_t index = 0; index < solution.joint_names.size(); ++index) {
      if (!positions.emplace(
          solution.joint_names[index], solution.positions_rad[index]).second)
      {
        output.status = {
          StatusCode::SDK_ERROR,
          "SDK kinematics adapter returned duplicate joint names",
          "humanoid_motion_server::kinematics::IKinematics::inverseKinematics", -1};
        return output;
      }
    }

    JointTarget ordered;
    ordered.joint_names = expected_joint_names;
    for (const auto & name : expected_joint_names) {
      const auto found = positions.find(name);
      if (found == positions.end()) {
        output.status = {
          StatusCode::SDK_ERROR,
          "SDK kinematics adapter solution is missing joint: " + name,
          "humanoid_motion_server::kinematics::IKinematics::inverseKinematics", -1};
        return output;
      }
      ordered.positions_rad.push_back(found->second);
    }
    output.joint_waypoints.push_back(ordered);
    seed = std::move(ordered);
  }
  output.status = MotionStatus::Ok();
  return output;
}

}  // namespace humanoid_motion_server::motion::detail

#endif  // MOVE_P_WAYPOINT_RESOLVER_HPP_
