// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "humanoid_motion_server/kinematics/kinematics.hpp"
#include "humanoid_motion_server/motion/command_pipeline.hpp"
#include "humanoid_motion_server/motion/sdk_motion_backend.hpp"
#include "humanoid_motion_server/tf/tf_runtime.hpp"
#include "humanoid_motion_interfaces/action/move_j.hpp"
#include "humanoid_motion_interfaces/action/move_l.hpp"
#include "humanoid_motion_interfaces/action/move_p.hpp"
#include "humanoid_motion_interfaces/msg/motion_options.hpp"
#include "humanoid_motion_interfaces/msg/status.hpp"
#include "humanoid_motion_server/channel_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace humanoid_motion_server
{
namespace
{

using namespace std::chrono_literals;
using Core = humanoid_motion_server::motion::CommandPipeline;
using CoreStatus = humanoid_motion_server::motion::MotionStatus;
using CoreStatusCode = humanoid_motion_server::motion::StatusCode;
using SteadyClock = humanoid_motion_server::motion::SteadyClock;
using SteadyTime = humanoid_motion_server::motion::SteadyTime;
using Status = humanoid_motion_interfaces::msg::Status;

bool finite(const double value)
{
  return std::isfinite(value);
}

template<typename RangeT>
bool all_finite(const RangeT & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const auto value) {
      return finite(static_cast<double>(value));
    });
}

humanoid_motion_server::motion::MotionKind to_core_kind(const ChannelKind kind)
{
  switch (kind) {
    case ChannelKind::MOVE_J:
      return humanoid_motion_server::motion::MotionKind::MOVE_J;
    case ChannelKind::MOVE_L:
      return humanoid_motion_server::motion::MotionKind::MOVE_L;
    case ChannelKind::MOVE_P:
      return humanoid_motion_server::motion::MotionKind::MOVE_P;
    case ChannelKind::SERVO_J:
      return humanoid_motion_server::motion::MotionKind::SERVO_J;
    case ChannelKind::SERVO_P:
      return humanoid_motion_server::motion::MotionKind::SERVO_P;
  }
  throw std::runtime_error("unsupported channel kind");
}

std::uint16_t ros_status_code(const CoreStatusCode code)
{
  switch (code) {
    case CoreStatusCode::OK:
      return Status::OK;
    case CoreStatusCode::INVALID_ARGUMENT:
      return Status::INVALID_REQUEST;
    case CoreStatusCode::NOT_CONFIGURED:
      return Status::NOT_CONFIGURED;
    case CoreStatusCode::REJECTED:
      return Status::LOWER_PRIORITY;
    case CoreStatusCode::PREEMPTED:
      return Status::PREEMPTED;
    case CoreStatusCode::CANCELED:
      return Status::CANCELED;
    case CoreStatusCode::TIMEOUT:
      return Status::TIMEOUT;
    case CoreStatusCode::STALE_FEEDBACK:
      return Status::STATE_STALE;
    case CoreStatusCode::SDK_ERROR:
      return Status::SDK_ERROR;
    case CoreStatusCode::LIMIT_VIOLATION:
      return Status::LIMIT_VIOLATION;
    case CoreStatusCode::CONTROLLED_STOP:
      return Status::DRIVER_FAULT;
    case CoreStatusCode::INTERNAL_ERROR:
      return Status::INTERNAL_ERROR;
  }
  return Status::INTERNAL_ERROR;
}

void set_status(
  Status & output, const std::uint16_t code, const std::string & message)
{
  output.code = code;
  output.message = message;
}

void set_status(Status & output, const CoreStatus & input)
{
  std::string message = input.message;
  if (!input.sdk_api.empty()) {
    message += " [" + input.sdk_api;
    if (input.sdk_code != -1) {
      message += " code=" + std::to_string(input.sdk_code);
    }
    message += "]";
  }
  set_status(output, ros_status_code(input.code), message);
}

humanoid_motion_server::motion::Pose to_core_pose(const geometry_msgs::msg::Pose & pose)
{
  humanoid_motion_server::motion::Pose result;
  result.position_m = {{pose.position.x, pose.position.y, pose.position.z}};
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  result.orientation_xyzw = {{
    pose.orientation.x / norm, pose.orientation.y / norm,
    pose.orientation.z / norm, pose.orientation.w / norm}};
  return result;
}

geometry_msgs::msg::Pose to_ros_pose(const humanoid_motion_server::kinematics::Pose & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.position_m.x;
  result.position.y = pose.position_m.y;
  result.position.z = pose.position_m.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

/// The original SDK resolves model_path relative to its YAML file. Robot
/// profiles may live in a separate bringup package, so materialize a private
/// runtime copy that points at the already-resolved URDF launch parameter.
class RuntimeSdkConfig
{
public:
  RuntimeSdkConfig(const std::string & source_path, const std::string & model_path)
  {
    if (!std::filesystem::is_regular_file(source_path)) {
      throw std::runtime_error("SDK configuration does not exist: " + source_path);
    }
    const auto absolute_model = std::filesystem::absolute(model_path).lexically_normal();
    if (!std::filesystem::is_regular_file(absolute_model)) {
      throw std::runtime_error("SDK URDF model does not exist: " + absolute_model.string());
    }

    YAML::Node document = YAML::LoadFile(source_path);
    if (!document.IsMap()) {
      throw std::runtime_error("SDK configuration root must be a mapping");
    }
    document["model_path"] = absolute_model.string();
    YAML::Emitter emitter;
    emitter << document;
    if (!emitter.good()) {
      throw std::runtime_error("failed to serialize runtime SDK configuration");
    }

    auto path_template =
      (std::filesystem::temp_directory_path() / "humanoid_motion_server_sdk_XXXXXX").string();
    std::vector<char> path_buffer(path_template.begin(), path_template.end());
    path_buffer.push_back('\0');
    const int descriptor = mkstemp(path_buffer.data());
    if (descriptor < 0) {
      throw std::runtime_error(
              "failed to create runtime SDK configuration: " + std::string(std::strerror(errno)));
    }
    path_ = path_buffer.data();

    const std::string payload = emitter.c_str();
    std::size_t written = 0U;
    while (written < payload.size()) {
      const auto count = write(
        descriptor, payload.data() + written, payload.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        const auto error = std::string(std::strerror(errno));
        close(descriptor);
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
        path_.clear();
        throw std::runtime_error("failed to write runtime SDK configuration: " + error);
      }
      written += static_cast<std::size_t>(count);
    }
    if (close(descriptor) != 0) {
      const auto error = std::string(std::strerror(errno));
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
      path_.clear();
      throw std::runtime_error("failed to close runtime SDK configuration: " + error);
    }
  }

  ~RuntimeSdkConfig()
  {
    if (!path_.empty()) {
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
    }
  }

  RuntimeSdkConfig(const RuntimeSdkConfig &) = delete;
  RuntimeSdkConfig & operator=(const RuntimeSdkConfig &) = delete;

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

/// Serializes access to the one SDK kinematics instance shared by services and
/// motion runtime. It performs no kinematics itself.
class SerializedKinematics final : public humanoid_motion_server::kinematics::IKinematics
{
public:
  explicit SerializedKinematics(std::shared_ptr<humanoid_motion_server::kinematics::IKinematics> delegate)
  : delegate_(std::move(delegate))
  {
    if (!delegate_) {
      throw std::invalid_argument("kinematics delegate is null");
    }
  }

  humanoid_motion_server::kinematics::Status configure(
    const humanoid_motion_server::kinematics::Configuration & configuration) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->configure(configuration);
  }

  humanoid_motion_server::kinematics::Status load(const std::string & path) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->load(path);
  }

  humanoid_motion_server::kinematics::Result<humanoid_motion_server::kinematics::Pose> forwardKinematics(
    const humanoid_motion_server::kinematics::ForwardKinematicsRequest & request) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->forwardKinematics(request);
  }

  humanoid_motion_server::kinematics::Result<humanoid_motion_server::kinematics::InverseKinematicsResult> inverseKinematics(
    const humanoid_motion_server::kinematics::InverseKinematicsRequest & request) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->inverseKinematics(request);
  }

  humanoid_motion_server::kinematics::Status setToolTransform(
    const humanoid_motion_server::kinematics::ToolTransform & transform) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->setToolTransform(transform);
  }

  humanoid_motion_server::kinematics::Result<humanoid_motion_server::kinematics::ToolTransform> getToolTransform(
    const std::string & name) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->getToolTransform(name);
  }

  humanoid_motion_server::kinematics::Result<std::vector<humanoid_motion_server::kinematics::ToolTransform>>
  getToolTransforms() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->getToolTransforms();
  }

  humanoid_motion_server::kinematics::Result<std::vector<std::string>> jointGroup(
    const std::string & group_name) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->jointGroup(group_name);
  }

  humanoid_motion_server::kinematics::Status validateJointState(
    const std::string & group_name,
    const humanoid_motion_server::kinematics::JointState & state) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return delegate_->validateJointState(group_name, state);
  }

private:
  std::shared_ptr<humanoid_motion_server::kinematics::IKinematics> delegate_;
  mutable std::mutex mutex_;
};

}  // namespace

class HumanoidMotionControlNode : public rclcpp::Node
{
public:
  using MoveJ = humanoid_motion_interfaces::action::MoveJ;
  using MoveL = humanoid_motion_interfaces::action::MoveL;
  using MoveP = humanoid_motion_interfaces::action::MoveP;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;
  using GoalHandleMoveP = rclcpp_action::ServerGoalHandle<MoveP>;

  HumanoidMotionControlNode()
  : Node("humanoid_motion_control")
  {
    channel_config_file_ = declare_parameter<std::string>("channel_config_file", "");
    sdk_config_file_ = declare_parameter<std::string>("sdk_config_file", "");
    tool_config_file_ = declare_parameter<std::string>("tool_config_file", "");
    urdf_file_ = declare_parameter<std::string>("urdf_file", "");
    joint_state_endpoint_ = declare_parameter<std::string>(
      "joint_state_endpoint", "/hc_teleop/joint_states");
    joint_command_endpoint_ =
      declare_parameter<std::string>("joint_command_endpoint", "/hc_teleop/joint_cmd");
    control_frequency_hz_ = declare_parameter<double>("control_frequency_hz", 100.0);
    input_stamp_max_age_s_ = declare_parameter<double>("input_stamp_max_age_s", 0.5);
    input_stamp_future_tolerance_s_ =
      declare_parameter<double>("input_stamp_future_tolerance_s", 0.1);
    declare_parameter<bool>("test_pause_driver_feedback", false);
    feedback_max_age_ms_ = declare_parameter<int>("feedback_max_age_ms", 100);
    servo_lease_ms_ = declare_parameter<int>("servo_lease_ms", 100);
    default_move_timeout_s_ = declare_parameter<double>("default_move_timeout_s", 60.0);
    joint_position_tolerance_rad_ =
      declare_parameter<double>("move_j_position_tolerance_rad", 0.01);
    joint_velocity_tolerance_rad_s_ =
      declare_parameter<double>("stopped_velocity_tolerance_rad_s", 0.02);
    cartesian_position_tolerance_m_ =
      declare_parameter<double>("cartesian_position_tolerance_m", 0.005);
    cartesian_orientation_tolerance_rad_ =
      declare_parameter<double>("cartesian_orientation_tolerance_rad", 0.02);
    stable_duration_s_ = declare_parameter<double>("stable_duration_s", 0.1);
    joint_max_velocity_rad_s_ =
      declare_parameter<double>("joint_max_velocity_rad_s", 1.0);
    joint_max_acceleration_rad_s2_ =
      declare_parameter<double>("joint_max_acceleration_rad_s2", 2.0);
    joint_max_jerk_rad_s3_ =
      declare_parameter<double>("joint_max_jerk_rad_s3", 10.0);
    cartesian_max_linear_velocity_m_s_ =
      declare_parameter<double>("cartesian_max_linear_velocity_m_s", 0.3);
    cartesian_max_linear_acceleration_m_s2_ =
      declare_parameter<double>("cartesian_max_linear_acceleration_m_s2", 0.6);
    cartesian_max_linear_jerk_m_s3_ =
      declare_parameter<double>("cartesian_max_linear_jerk_m_s3", 3.0);
    cartesian_max_angular_velocity_rad_s_ =
      declare_parameter<double>("cartesian_max_angular_velocity_rad_s", 1.0);
    cartesian_max_angular_acceleration_rad_s2_ =
      declare_parameter<double>("cartesian_max_angular_acceleration_rad_s2", 2.0);
    cartesian_max_angular_jerk_rad_s3_ =
      declare_parameter<double>("cartesian_max_angular_jerk_rad_s3", 10.0);

    validate_parameters();
    load_groups();
    channels_ = load_channel_config(channel_config_file_);
    runtime_sdk_config_ = std::make_unique<RuntimeSdkConfig>(sdk_config_file_, urdf_file_);
    sdk_config_file_ = runtime_sdk_config_->path();
    configure_sdk();
    configure_pipeline();

    tf_runtime_ = std::make_unique<tf::TfRuntime>(*this, urdf_file_, tool_config_file_);
    create_fk_publishers();
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_endpoint_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (!get_parameter("test_pause_driver_feedback").as_bool() && handle_joint_state(*message)) {
          tf_runtime_->publishFeedback(*message);
          publish_fk_poses();
        }
      });
    joint_command_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_command_endpoint_, rclcpp::QoS(10).reliable());
    create_channel_endpoints();

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HumanoidMotionControlNode::control_tick, this));
    RCLCPP_INFO(
      get_logger(), "unified motion control ready: %zu channels at %.1f Hz",
      channels_.size(), control_frequency_hz_);
  }

private:
  struct FeedbackSnapshot
  {
    sensor_msgs::msg::JointState message;
    humanoid_motion_server::motion::JointFeedback core;
    bool valid{false};
  };

  struct FkPublisher
  {
    ChannelConfig channel;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  };

  void validate_parameters() const
  {
    if (channel_config_file_.empty() || sdk_config_file_.empty() || tool_config_file_.empty() ||
      urdf_file_.empty())
    {
      throw std::runtime_error(
              "channel_config_file, sdk_config_file, tool_config_file, and urdf_file are required");
    }
    const std::vector<double> positive_values{
      control_frequency_hz_,
      input_stamp_max_age_s_, input_stamp_future_tolerance_s_,
      default_move_timeout_s_, joint_position_tolerance_rad_,
      joint_velocity_tolerance_rad_s_, cartesian_position_tolerance_m_,
      cartesian_orientation_tolerance_rad_, joint_max_velocity_rad_s_,
      joint_max_acceleration_rad_s2_, joint_max_jerk_rad_s3_,
      cartesian_max_linear_velocity_m_s_, cartesian_max_linear_acceleration_m_s2_,
      cartesian_max_linear_jerk_m_s3_, cartesian_max_angular_velocity_rad_s_,
      cartesian_max_angular_acceleration_rad_s2_, cartesian_max_angular_jerk_rad_s3_};
    if (!all_finite(positive_values) ||
      std::any_of(
        positive_values.begin(), positive_values.end(),
        [](const double value) {return value <= 0.0;}) ||
      !finite(stable_duration_s_) || stable_duration_s_<0.0 ||
      control_frequency_hz_>1000.0 || feedback_max_age_ms_ <= 0 || servo_lease_ms_ <= 0)
    {
      throw std::runtime_error("runtime rates, limits, tolerances, or timeouts are invalid");
    }
  }

  void load_groups()
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "joint_group_names", std::vector<std::string>{});
    if (names.empty()) {
      throw std::runtime_error("joint_group_names must not be empty");
    }
    std::set<std::string> unique_groups;
    for (const auto & name : names) {
      if (name.empty() || !unique_groups.insert(name).second) {
        throw std::runtime_error("joint group names must be non-empty and unique");
      }
      humanoid_motion_server::motion::JointGroupModel group;
      group.name = name;
      group.joint_names = declare_parameter<std::vector<std::string>>(
        "groups." + name, std::vector<std::string>{});
      group.lower_position_rad = declare_parameter<std::vector<double>>(
        "group_lower_limits." + name, std::vector<double>{});
      group.upper_position_rad = declare_parameter<std::vector<double>>(
        "group_upper_limits." + name, std::vector<double>{});
      if (group.joint_names.empty() ||
        group.lower_position_rad.size() != group.joint_names.size() ||
        group.upper_position_rad.size() != group.joint_names.size() ||
        !all_finite(group.lower_position_rad) || !all_finite(group.upper_position_rad))
      {
        throw std::runtime_error("complete finite position limits are required for " + name);
      }
      std::set<std::string> unique_joints;
      for (std::size_t index = 0; index < group.joint_names.size(); ++index) {
        if (group.joint_names[index].empty() ||
          !unique_joints.insert(group.joint_names[index]).second ||
          group.lower_position_rad[index] > group.upper_position_rad[index])
        {
          throw std::runtime_error("invalid joint order or limits for " + name);
        }
      }
      group_index_.emplace(name, groups_.size());
      groups_.push_back(std::move(group));
    }
  }

  void configure_sdk()
  {
    auto raw = humanoid_motion_server::kinematics::SdkKinematics::create();
    kinematics_ = std::make_shared<SerializedKinematics>(std::move(raw));
    const auto load_status = kinematics_->load(sdk_config_file_);
    if (!load_status.ok()) {
      throw std::runtime_error("failed to load SDK kinematics adapter: " + load_status.message);
    }
    load_tools();

    humanoid_motion_server::motion::MotionContextFactoryOptions options;
    options.joint_groups = groups_;
    options.kinematics = kinematics_;
    auto context = humanoid_motion_server::motion::MotionContextFactory::createFromSdkYaml(
      sdk_config_file_, options);
    if (!context.status.ok() || !context.backend) {
      throw std::runtime_error("failed to create SDK motion backend: " + context.status.message);
    }
    backend_ = std::move(context.backend);
  }

  void load_tools()
  {
    YAML::Node document;
    try {
      document = YAML::LoadFile(tool_config_file_);
    } catch (const YAML::Exception & error) {
      throw std::runtime_error(std::string("failed to parse tool YAML: ") + error.what());
    }
    if (!document.IsMap() || document.size() != 1U || !document["tools"] ||
      !document["tools"].IsSequence())
    {
      throw std::runtime_error("tool YAML must contain only a tools sequence");
    }
    const std::set<std::string> allowed{
      "name", "parent_frame", "child_frame", "translation_m", "rotation_xyzw"};
    std::set<std::string> names;
    for (const auto & entry : document["tools"]) {
      if (!entry.IsMap() || entry.size() != allowed.size()) {
        throw std::runtime_error("every tool entry must use the closed five-key schema");
      }
      for (const auto & item : entry) {
        const auto key = item.first.as<std::string>();
        if (allowed.count(key) == 0U) {
          throw std::runtime_error("unknown tool key: " + key);
        }
      }
      try {
        const auto name = entry["name"].as<std::string>();
        const auto parent = entry["parent_frame"].as<std::string>();
        const auto child = entry["child_frame"].as<std::string>();
        const auto translation = entry["translation_m"].as<std::vector<double>>();
        const auto rotation = entry["rotation_xyzw"].as<std::vector<double>>();
        if (name.empty() || parent.empty() || child.empty() || name != child ||
          !names.insert(name).second || translation.size() != 3U || rotation.size() != 4U ||
          !all_finite(translation) || !all_finite(rotation))
        {
          throw std::runtime_error("invalid tool name, frames, translation, or quaternion");
        }
        tf2::Quaternion quaternion(rotation[0], rotation[1], rotation[2], rotation[3]);
        if (!finite(quaternion.length2()) || quaternion.length2() < 1.0e-12) {
          throw std::runtime_error("tool quaternion cannot be normalized");
        }
        quaternion.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        humanoid_motion_server::kinematics::ToolTransform tool;
        tool.name = name;
        tool.parent_link = parent;
        tool.translation_m = {translation[0], translation[1], translation[2]};
        tool.rpy_rad = {roll, pitch, yaw};
        const auto status = kinematics_->setToolTransform(tool);
        if (!status.ok()) {
          throw std::runtime_error("failed to configure tool " + name + ": " + status.message);
        }
      } catch (const YAML::Exception & error) {
        throw std::runtime_error(std::string("invalid tool field type: ") + error.what());
      }
    }
  }

  void configure_pipeline()
  {
    humanoid_motion_server::motion::CommandPipelineConfig config;
    config.control_frequency_hz = control_frequency_hz_;
    config.feedback_max_age = std::chrono::milliseconds(feedback_max_age_ms_);
    config.goal_monitor.joint_position_tolerance_rad = joint_position_tolerance_rad_;
    config.goal_monitor.joint_velocity_tolerance_rad_s = joint_velocity_tolerance_rad_s_;
    config.goal_monitor.cartesian_position_tolerance_m = cartesian_position_tolerance_m_;
    config.goal_monitor.cartesian_orientation_tolerance_rad =
      cartesian_orientation_tolerance_rad_;
    config.goal_monitor.stable_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(stable_duration_s_));
    // Per-goal ROS timeouts are enforced by the adapter. This large backstop
    // remains inside core for a lost action worker or other integration fault.
    config.goal_monitor.move_timeout = 24h;
    pipeline_ = std::make_unique<Core>(backend_, groups_, config);

    for (const auto & channel : channels_) {
      std::vector<std::string> allowed_groups;
      if (!channel.group.empty()) {
        if (group_index_.count(channel.group) == 0U) {
          throw std::runtime_error(
                  "channel '" + channel.name + "' references an unknown group");
        }
        allowed_groups.push_back(channel.group);
      } else {
        if (channel.kind == ChannelKind::SERVO_J || channel.kind == ChannelKind::SERVO_P) {
          throw std::runtime_error("Servo channel group is required");
        }
        for (const auto & group : groups_) {
          allowed_groups.push_back(group.name);
        }
      }
      for (const auto & group : allowed_groups) {
        const auto id = policy_id(channel, group);
        humanoid_motion_server::motion::EndpointPolicy policy;
        policy.endpoint_name = id;
        policy.kind = to_core_kind(channel.kind);
        policy.group_name = group;
        policy.priority = channel.priority;
        policy.servo_lease = std::chrono::milliseconds(servo_lease_ms_);
        const auto status = pipeline_->registerEndpoint(policy);
        if (!status.ok()) {
          throw std::runtime_error("failed to register channel policy: " + status.message);
        }
      }
    }
  }

  static std::string policy_id(const ChannelConfig & channel, const std::string & group)
  {
    return channel.name + "@" + group;
  }

  const humanoid_motion_server::motion::JointGroupModel * group(const std::string & name) const
  {
    const auto found = group_index_.find(name);
    return found == group_index_.end() ? nullptr : &groups_[found->second];
  }

  bool handle_joint_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.empty() || message.position.size() != message.name.size() ||
      message.velocity.size() != message.name.size() || !all_finite(message.position) ||
      !all_finite(message.velocity))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "discarded malformed real driver feedback");
      return false;
    }
    std::set<std::string> names;
    for (const auto & name : message.name) {
      if (name.empty() || !names.insert(name).second) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "discarded duplicate/empty driver feedback names");
        return false;
      }
    }
    FeedbackSnapshot snapshot;
    snapshot.message = message;
    snapshot.core.joint_names = message.name;
    snapshot.core.positions_rad = message.position;
    snapshot.core.velocities_rad_s = message.velocity;
    snapshot.core.received_at = SteadyClock::now();
    snapshot.valid = true;
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_ = std::move(snapshot);
    return true;
  }

  FeedbackSnapshot feedback_snapshot(const bool require_fresh) const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    auto result = feedback_;
    if (require_fresh && result.valid &&
      SteadyClock::now() - result.core.received_at >=
      std::chrono::milliseconds(feedback_max_age_ms_))
    {
      result.valid = false;
    }
    return result;
  }

  bool complete_joint_target(
    const std::string & group_name, const sensor_msgs::msg::JointState & input,
    humanoid_motion_server::motion::JointTarget & output, std::string & error) const
  {
    const auto * model = group(group_name);
    if (model == nullptr) {
      error = "group_name is empty or unknown";
      return false;
    }
    if (!validate_stamp(input.header.stamp, error)) {
      return false;
    }
    if (input.name.empty() || input.position.size() != input.name.size() ||
      !all_finite(input.position))
    {
      error = "joint target names/positions are empty, mismatched, or non-finite";
      return false;
    }
    std::map<std::string, double> requested;
    const std::set<std::string> allowed(model->joint_names.begin(), model->joint_names.end());
    for (std::size_t index = 0; index < input.name.size(); ++index) {
      if (input.name[index].empty() || allowed.count(input.name[index]) == 0U ||
        !requested.emplace(input.name[index], input.position[index]).second)
      {
        error = "joint target contains an empty, duplicate, or out-of-group name";
        return false;
      }
    }
    const auto state = feedback_snapshot(true);
    if (!state.valid) {
      error = "real JointState feedback is missing or stale";
      return false;
    }
    std::map<std::string, double> actual;
    for (std::size_t index = 0; index < state.core.joint_names.size(); ++index) {
      actual.emplace(state.core.joint_names[index], state.core.positions_rad[index]);
    }
    output.joint_names = model->joint_names;
    for (const auto & joint : model->joint_names) {
      const auto requested_value = requested.find(joint);
      if (requested_value != requested.end()) {
        output.positions_rad.push_back(requested_value->second);
      } else {
        const auto actual_value = actual.find(joint);
        if (actual_value == actual.end()) {
          error = "real feedback is missing joint '" + joint + "' needed to complete the group";
          return false;
        }
        output.positions_rad.push_back(actual_value->second);
      }
    }
    return true;
  }

  bool validate_stamp(
    const builtin_interfaces::msg::Time & stamp, std::string & error) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      return true;
    }
    try {
      const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
      if (!finite(age) || age > input_stamp_max_age_s_) {
        error = "input timestamp is stale or invalid";
        return false;
      }
      if (age < -input_stamp_future_tolerance_s_) {
        error = "input timestamp is too far in the future";
        return false;
      }
    } catch (const std::exception & exception) {
      error = "invalid input timestamp: " + std::string(exception.what());
      return false;
    }
    return true;
  }

  bool validate_options(
    const humanoid_motion_interfaces::msg::MotionOptions & options,
    std::string & error) const
  {
    const std::vector<double> scales{
      options.velocity_scale, options.acceleration_scale, options.jerk_scale};
    if (!all_finite(scales) || !finite(options.timeout_sec) || options.timeout_sec < 0.0) {
      error = "motion options contain a non-finite or negative timeout";
      return false;
    }
    for (const double scale : scales) {
      if (scale < 0.0 || scale > 1.0) {
        error = "motion option scales must be zero/default or in (0, 1]";
        return false;
      }
    }
    return true;
  }

  bool transform_target_pose(
    const geometry_msgs::msg::PoseStamped & input, const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & output, std::string & error) const
  {
    if (!validate_stamp(input.header.stamp, error)) {
      return false;
    }
    return tf_runtime_->transformPose(input, target_frame, output, error);
  }

  humanoid_motion_server::motion::MotionLimits motion_limits(
    const humanoid_motion_interfaces::msg::MotionOptions & options,
    const std::size_t joint_count) const
  {
    const auto scale = [](const double value) {return value == 0.0 ? 1.0 : value;};
    humanoid_motion_server::motion::MotionLimits limits;
    limits.joint_max_velocity_rad_s.assign(
      joint_count, joint_max_velocity_rad_s_ * scale(options.velocity_scale));
    limits.joint_max_acceleration_rad_s2.assign(
      joint_count, joint_max_acceleration_rad_s2_ * scale(options.acceleration_scale));
    limits.joint_max_jerk_rad_s3.assign(
      joint_count, joint_max_jerk_rad_s3_ * scale(options.jerk_scale));
    limits.cartesian_max_linear_velocity_m_s =
      cartesian_max_linear_velocity_m_s_ * scale(options.velocity_scale);
    limits.cartesian_max_linear_acceleration_m_s2 =
      cartesian_max_linear_acceleration_m_s2_ * scale(options.acceleration_scale);
    limits.cartesian_max_linear_jerk_m_s3 =
      cartesian_max_linear_jerk_m_s3_ * scale(options.jerk_scale);
    limits.cartesian_max_angular_velocity_rad_s =
      cartesian_max_angular_velocity_rad_s_ * scale(options.velocity_scale);
    limits.cartesian_max_angular_acceleration_rad_s2 =
      cartesian_max_angular_acceleration_rad_s2_ * scale(options.acceleration_scale);
    limits.cartesian_max_angular_jerk_rad_s3 =
      cartesian_max_angular_jerk_rad_s3_ * scale(options.jerk_scale);
    return limits;
  }

  template<typename GoalHandleT>
  static std::string session_id(
    const ChannelConfig & channel, const std::shared_ptr<GoalHandleT> & handle)
  {
    std::ostringstream stream;
    stream << channel.name << ':' << std::hex << std::setfill('0');
    for (const auto byte : handle->get_goal_id()) {
      stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
  }

  static double effective_timeout(
    const humanoid_motion_interfaces::msg::MotionOptions & options,
    const double default_timeout)
  {
    return options.timeout_sec == 0.0 ? default_timeout : options.timeout_sec;
  }

  void create_channel_endpoints()
  {
    for (const auto & channel : channels_) {
      const auto & endpoint = channel.endpoint;
      switch (channel.kind) {
        case ChannelKind::MOVE_J:
          move_j_servers_.push_back(
            rclcpp_action::create_server<MoveJ>(
              this, endpoint,
              [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveJ::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
              },
              [](const std::shared_ptr<GoalHandleMoveJ>) {
                return rclcpp_action::CancelResponse::ACCEPT;
              },
              [this, channel](const std::shared_ptr<GoalHandleMoveJ> handle) {
                std::thread([this, channel, handle]() {execute_move_j(channel, handle);}).detach();
              }));
          break;
        case ChannelKind::MOVE_L:
          move_l_servers_.push_back(
            rclcpp_action::create_server<MoveL>(
              this, endpoint,
              [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveL::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
              },
              [](const std::shared_ptr<GoalHandleMoveL>) {
                return rclcpp_action::CancelResponse::ACCEPT;
              },
              [this, channel](const std::shared_ptr<GoalHandleMoveL> handle) {
                std::thread([this, channel, handle]() {execute_move_l(channel, handle);}).detach();
              }));
          break;
        case ChannelKind::MOVE_P:
          move_p_servers_.push_back(
            rclcpp_action::create_server<MoveP>(
              this, endpoint,
              [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveP::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
              },
              [](const std::shared_ptr<GoalHandleMoveP>) {
                return rclcpp_action::CancelResponse::ACCEPT;
              },
              [this, channel](const std::shared_ptr<GoalHandleMoveP> handle) {
                std::thread([this, channel, handle]() {execute_move_p(channel, handle);}).detach();
              }));
          break;
        case ChannelKind::SERVO_J:
          servo_j_subscriptions_.push_back(
            create_subscription<sensor_msgs::msg::JointState>(
              endpoint, rclcpp::SensorDataQoS(),
              [this, channel](const sensor_msgs::msg::JointState::SharedPtr message) {
                handle_servo_j(channel, *message);
              }));
          break;
        case ChannelKind::SERVO_P:
          servo_p_subscriptions_.push_back(
            create_subscription<geometry_msgs::msg::PoseStamped>(
              endpoint, rclcpp::SensorDataQoS(),
              [this, channel](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
                handle_servo_p(channel, *message);
              }));
          break;
      }
    }
  }

  void create_fk_publishers()
  {
    for (const auto & channel : channels_) {
      if (channel.fk_pose_topic.empty()) {
        continue;
      }
      fk_publishers_.push_back(
        FkPublisher{
          channel,
          create_publisher<geometry_msgs::msg::PoseStamped>(
            channel.fk_pose_topic, rclcpp::SensorDataQoS())});
      RCLCPP_INFO(
        get_logger(), "publishing measured FK for %s on %s (%s -> %s)",
        channel.group.c_str(), channel.fk_pose_topic.c_str(),
        channel.base_frame.c_str(), channel.tip_frame.c_str());
    }
  }

  void publish_fk_poses()
  {
    if (fk_publishers_.empty()) {
      return;
    }
    const auto state = feedback_snapshot(false);
    if (!state.valid) {
      return;
    }
    for (const auto & output : fk_publishers_) {
      geometry_msgs::msg::PoseStamped pose;
      if (feedback_pose(
          state, output.channel.group, output.channel.base_frame,
          output.channel.tip_frame, pose))
      {
        output.publisher->publish(pose);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cannot publish measured FK for channel '%s'",
          output.channel.name.c_str());
      }
    }
  }

  void execute_move_j(
    const ChannelConfig & channel, const std::shared_ptr<GoalHandleMoveJ> & handle)
  {
    const auto & goal = *handle->get_goal();
    std::string error;
    if ((!channel.group.empty() && goal.group_name != channel.group) ||
      !validate_options(goal.options, error))
    {
      abort_invalid<MoveJ>(
        handle, error.empty() ? "group_name is not permitted by this channel" : error);
      return;
    }
    humanoid_motion_server::motion::JointTarget target;
    if (!complete_joint_target(goal.group_name, goal.target, target, error)) {
      abort_invalid<MoveJ>(handle, error);
      return;
    }
    humanoid_motion_server::motion::MoveJRequest request;
    request.request_id = session_id(channel, handle);
    request.group_name = goal.group_name;
    request.target = std::move(target);
    request.limits = motion_limits(goal.options, request.target.joint_names.size());
    execute_move<MoveJ>(
      channel, handle, request, goal.options, channel.base_frame, channel.tip_frame);
  }

  void execute_move_l(
    const ChannelConfig & channel, const std::shared_ptr<GoalHandleMoveL> & handle)
  {
    const auto & goal = *handle->get_goal();
    std::string error;
    if (group(goal.group_name) == nullptr || goal.tip_frame.empty() ||
      (!channel.group.empty() && goal.group_name != channel.group) ||
      (!channel.tip_frame.empty() && goal.tip_frame != channel.tip_frame) ||
      !validate_options(goal.options, error))
    {
      abort_invalid<MoveL>(
        handle, error.empty() ? "group_name or tip_frame is not permitted by this channel" : error);
      return;
    }
    geometry_msgs::msg::PoseStamped target;
    if (!transform_target_pose(goal.target_pose, channel.base_frame, target, error)) {
      abort_invalid<MoveL>(handle, error);
      return;
    }
    humanoid_motion_server::motion::MoveLRequest request;
    request.request_id = session_id(channel, handle);
    request.group_name = goal.group_name;
    request.base_link = channel.base_frame;
    request.link_name = goal.tip_frame;
    request.target = to_core_pose(target.pose);
    request.limits = motion_limits(goal.options, group(goal.group_name)->joint_names.size());
    execute_move<MoveL>(
      channel, handle, request, goal.options, request.base_link, request.link_name);
  }

  void execute_move_p(
    const ChannelConfig & channel, const std::shared_ptr<GoalHandleMoveP> & handle)
  {
    const auto & goal = *handle->get_goal();
    std::string error;
    if (group(goal.group_name) == nullptr || goal.tip_frame.empty() ||
      (!channel.group.empty() && goal.group_name != channel.group) ||
      (!channel.tip_frame.empty() && goal.tip_frame != channel.tip_frame) ||
      !validate_options(goal.options, error))
    {
      abort_invalid<MoveP>(
        handle, error.empty() ? "group_name or tip_frame is not permitted by this channel" : error);
      return;
    }
    geometry_msgs::msg::PoseStamped target;
    if (!transform_target_pose(goal.target_pose, channel.base_frame, target, error)) {
      abort_invalid<MoveP>(handle, error);
      return;
    }
    humanoid_motion_server::motion::MovePRequest request;
    request.request_id = session_id(channel, handle);
    request.group_name = goal.group_name;
    request.base_link = channel.base_frame;
    request.link_name = goal.tip_frame;
    request.waypoints.push_back(to_core_pose(target.pose));
    request.limits = motion_limits(goal.options, group(goal.group_name)->joint_names.size());
    execute_move<MoveP>(
      channel, handle, request, goal.options, request.base_link, request.link_name);
  }

  template<typename ActionT, typename GoalHandleT, typename RequestT>
  void execute_move(
    const ChannelConfig & channel, const std::shared_ptr<GoalHandleT> & handle,
    const RequestT & request,
    const humanoid_motion_interfaces::msg::MotionOptions & options,
    const std::string & base_frame, const std::string & tip_frame)
  {
    const auto state = feedback_snapshot(true);
    if (!state.valid) {
      abort_status<ActionT>(
        handle, Status::STATE_STALE, "real JointState feedback is missing or stale");
      return;
    }
    const auto submitted = pipeline_->submitMove(
      policy_id(channel, request.group_name), request, SteadyClock::now());
    if (!submitted.status.ok()) {
      abort_core<ActionT>(handle, submitted.status);
      return;
    }

    const auto started = SteadyClock::now();
    auto next_feedback = started;
    bool cancel_requested = false;
    while (rclcpp::ok()) {
      if (handle->is_canceling() && !cancel_requested) {
        (void)pipeline_->cancel(submitted.session_id);
        cancel_requested = true;
      }
      const double elapsed =
        std::chrono::duration<double>(SteadyClock::now() - started).count();
      if (!cancel_requested && elapsed > effective_timeout(options, default_move_timeout_s_)) {
        (void)pipeline_->cancel(submitted.session_id);
        discard_terminal_event(submitted.session_id);
        abort_status<ActionT>(handle, Status::TIMEOUT, "Move exceeded its requested timeout");
        return;
      }

      if (const auto event = take_terminal_event(submitted.session_id); event) {
        finish_move<ActionT>(handle, *event, request.group_name, base_frame, tip_frame);
        return;
      }
      if (SteadyClock::now() >= next_feedback) {
        publish_feedback<ActionT>(handle, request.group_name, base_frame, tip_frame);
        next_feedback = SteadyClock::now() + 50ms;
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  template<typename ActionT, typename GoalHandleT>
  void publish_feedback(
    const std::shared_ptr<GoalHandleT> & handle, const std::string & group_name,
    const std::string & base_frame, const std::string & tip_frame)
  {
    const auto state = feedback_snapshot(true);
    if (!state.valid) {
      return;
    }
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    feedback->progress = 0.0F;
    feedback->actual_joint_state = state.message;
    if constexpr (!std::is_same_v<ActionT, MoveJ>) {
      geometry_msgs::msg::PoseStamped pose;
      if (feedback_pose(state, group_name, base_frame, tip_frame, pose)) {
        feedback->actual_pose = pose;
      }
    }
    handle->publish_feedback(feedback);
  }

  template<typename ActionT, typename GoalHandleT>
  void finish_move(
    const std::shared_ptr<GoalHandleT> & handle,
    const humanoid_motion_server::motion::SessionEvent & event, const std::string & group_name,
    const std::string & base_frame, const std::string & tip_frame)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    set_status(result->status, event.status);
    const auto state = feedback_snapshot(false);
    if (state.valid) {
      result->final_joint_state = state.message;
    }
    if constexpr (!std::is_same_v<ActionT, MoveJ>) {
      geometry_msgs::msg::PoseStamped pose;
      if (state.valid && feedback_pose(state, group_name, base_frame, tip_frame, pose)) {
        result->final_pose = pose;
      }
    }

    switch (event.state) {
      case humanoid_motion_server::motion::SessionState::SUCCEEDED:
        set_status(
          result->status, Status::OK,
          "motion GoalMonitor reached the goal from real feedback");
        handle->succeed(result);
        break;
      case humanoid_motion_server::motion::SessionState::CANCELED:
        set_status(result->status, Status::CANCELED, event.status.message);
        handle->canceled(result);
        break;
      case humanoid_motion_server::motion::SessionState::PREEMPTED:
        set_status(result->status, Status::PREEMPTED, event.status.message);
        handle->abort(result);
        break;
      case humanoid_motion_server::motion::SessionState::ABORTED:
        handle->abort(result);
        break;
      case humanoid_motion_server::motion::SessionState::PENDING:
      case humanoid_motion_server::motion::SessionState::RUNNING:
        set_status(result->status, Status::INTERNAL_ERROR, "non-terminal core event escaped");
        handle->abort(result);
        break;
    }
  }

  template<typename ActionT, typename GoalHandleT>
  void abort_invalid(const std::shared_ptr<GoalHandleT> & handle, const std::string & message)
  {
    abort_status<ActionT>(handle, Status::INVALID_REQUEST, message);
  }

  template<typename ActionT, typename GoalHandleT>
  void abort_core(const std::shared_ptr<GoalHandleT> & handle, const CoreStatus & status)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    set_status(result->status, status);
    const auto state = feedback_snapshot(false);
    if (state.valid) {
      result->final_joint_state = state.message;
    }
    handle->abort(result);
  }

  template<typename ActionT, typename GoalHandleT>
  void abort_status(
    const std::shared_ptr<GoalHandleT> & handle, const std::uint16_t code,
    const std::string & message)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    set_status(result->status, code, message);
    const auto state = feedback_snapshot(false);
    if (state.valid) {
      result->final_joint_state = state.message;
    }
    handle->abort(result);
  }

  void handle_servo_j(
    const ChannelConfig & channel, const sensor_msgs::msg::JointState & message)
  {
    humanoid_motion_server::motion::JointTarget target;
    std::string error;
    if (!complete_joint_target(channel.group, message, target, error)) {
      RCLCPP_WARN(get_logger(), "discarding ServoJ: %s", error.c_str());
      return;
    }
    humanoid_motion_server::motion::ServoJRequest request;
    request.request_id = "servo:" + channel.name;
    request.group_name = channel.group;
    request.target = std::move(target);
    const humanoid_motion_interfaces::msg::MotionOptions options;
    request.limits = motion_limits(options, request.target.joint_names.size());
    const auto result = pipeline_->updateServo(
      policy_id(channel, channel.group), request, SteadyClock::now());
    if (!result.status.ok()) {
      RCLCPP_WARN(get_logger(), "motion runtime rejected ServoJ: %s", result.status.message.c_str());
    }
  }

  void handle_servo_p(
    const ChannelConfig & channel, const geometry_msgs::msg::PoseStamped & message)
  {
    const auto * model = group(channel.group);
    if (model == nullptr) {
      return;
    }
    geometry_msgs::msg::PoseStamped target;
    std::string error;
    if (!transform_target_pose(message, channel.base_frame, target, error)) {
      RCLCPP_WARN(get_logger(), "discarding ServoP: %s", error.c_str());
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TRACE motion IN ServoP channel=%s endpoint=%s target_xyz=(%.4f, %.4f, %.4f)",
      channel.name.c_str(), channel.endpoint.c_str(),
      target.pose.position.x, target.pose.position.y, target.pose.position.z);
    humanoid_motion_server::motion::ServoPRequest request;
    request.request_id = "servo:" + channel.name;
    request.group_name = channel.group;
    request.base_link = channel.base_frame;
    request.link_name = channel.tip_frame;
    request.target = to_core_pose(target.pose);
    const humanoid_motion_interfaces::msg::MotionOptions options;
    request.limits = motion_limits(options, model->joint_names.size());
    const auto result = pipeline_->updateServo(
      policy_id(channel, channel.group), request, SteadyClock::now());
    if (!result.status.ok()) {
      RCLCPP_WARN(get_logger(), "motion runtime rejected ServoP: %s", result.status.message.c_str());
    }
  }

  void control_tick()
  {
    const auto state = feedback_snapshot(false);
    humanoid_motion_server::motion::JointFeedback feedback;
    if (state.valid) {
      feedback = state.core;
    }
    const auto tick = pipeline_->tick(feedback, SteadyClock::now());
    if (!tick.commands.empty()) {
      const auto & first = tick.commands.front();
      const double first_position =
        first.positions_rad.empty() ? 0.0 : first.positions_rad.front();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TRACE motion OUT /hc_teleop/joint_cmd commands=%zu joints=%zu first_position=%.4f",
        tick.commands.size(), first.joint_names.size(), first_position);
    }
    for (const auto & command : tick.commands) {
      if (!command.passed_final_sdk_rtc) {
        RCLCPP_FATAL(get_logger(), "motion runtime returned a command without final SDK RTC");
        continue;
      }

      sensor_msgs::msg::JointState output;
      output.header.stamp = now();
      output.name = command.joint_names;
      output.position = command.positions_rad;
      output.velocity = command.velocities_rad_s;
      // This topic is the only motion-server-to-driver command boundary.
      joint_command_publisher_->publish(output);
    }
    {
      std::lock_guard<std::mutex> lock(event_mutex_);
      for (const auto & event : tick.events) {
        if (event.state != humanoid_motion_server::motion::SessionState::PENDING &&
          event.state != humanoid_motion_server::motion::SessionState::RUNNING)
        {
          terminal_events_[event.session_id] = event;
        }
      }
    }
    event_condition_.notify_all();
  }

  std::optional<humanoid_motion_server::motion::SessionEvent> take_terminal_event(
    const std::string & session)
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    const auto found = terminal_events_.find(session);
    if (found == terminal_events_.end()) {
      return std::nullopt;
    }
    auto event = found->second;
    terminal_events_.erase(found);
    return event;
  }

  void discard_terminal_event(const std::string & session)
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    terminal_events_.erase(session);
  }

  bool feedback_pose(
    const FeedbackSnapshot & state, const std::string & group_name,
    const std::string & base_frame, const std::string & tip_frame,
    geometry_msgs::msg::PoseStamped & output) const
  {
    const auto * model = group(group_name);
    if (model == nullptr) {
      return false;
    }
    std::map<std::string, double> positions;
    for (std::size_t index = 0; index < state.core.joint_names.size(); ++index) {
      positions.emplace(state.core.joint_names[index], state.core.positions_rad[index]);
    }
    humanoid_motion_server::kinematics::ForwardKinematicsRequest request;
    request.kinematics.group_name = group_name;
    request.kinematics.base_link = base_frame;
    request.kinematics.link_name = tip_frame;
    request.joint_state.joint_names = model->joint_names;
    for (const auto & joint : model->joint_names) {
      const auto found = positions.find(joint);
      if (found == positions.end()) {
        return false;
      }
      request.joint_state.positions_rad.push_back(found->second);
    }
    const auto result = kinematics_->forwardKinematics(request);
    if (!result.ok()) {
      return false;
    }
    output.header.stamp = now();
    output.header.frame_id = base_frame;
    output.pose = to_ros_pose(*result.value);
    return true;
  }

  std::string channel_config_file_;
  std::string sdk_config_file_;
  std::string tool_config_file_;
  std::string urdf_file_;
  std::unique_ptr<RuntimeSdkConfig> runtime_sdk_config_;
  std::string joint_state_endpoint_;
  std::string joint_command_endpoint_;
  double control_frequency_hz_{100.0};
  double input_stamp_max_age_s_{0.5};
  double input_stamp_future_tolerance_s_{0.1};
  int feedback_max_age_ms_{100};
  int servo_lease_ms_{100};
  double default_move_timeout_s_{60.0};
  double joint_position_tolerance_rad_{0.01};
  double joint_velocity_tolerance_rad_s_{0.02};
  double cartesian_position_tolerance_m_{0.005};
  double cartesian_orientation_tolerance_rad_{0.02};
  double stable_duration_s_{0.1};
  double joint_max_velocity_rad_s_{1.0};
  double joint_max_acceleration_rad_s2_{2.0};
  double joint_max_jerk_rad_s3_{10.0};
  double cartesian_max_linear_velocity_m_s_{0.3};
  double cartesian_max_linear_acceleration_m_s2_{0.6};
  double cartesian_max_linear_jerk_m_s3_{3.0};
  double cartesian_max_angular_velocity_rad_s_{1.0};
  double cartesian_max_angular_acceleration_rad_s2_{2.0};
  double cartesian_max_angular_jerk_rad_s3_{10.0};

  std::vector<ChannelConfig> channels_;
  std::vector<humanoid_motion_server::motion::JointGroupModel> groups_;
  std::map<std::string, std::size_t> group_index_;
  humanoid_motion_server::motion::HumanoidKinematicsPtr kinematics_;
  humanoid_motion_server::motion::SdkMotionBackendPtr backend_;
  std::unique_ptr<Core> pipeline_;
  std::unique_ptr<tf::TfRuntime> tf_runtime_;

  mutable std::mutex feedback_mutex_;
  FeedbackSnapshot feedback_;
  std::mutex event_mutex_;
  std::condition_variable event_condition_;
  std::map<std::string, humanoid_motion_server::motion::SessionEvent> terminal_events_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::vector<rclcpp_action::Server<MoveJ>::SharedPtr> move_j_servers_;
  std::vector<rclcpp_action::Server<MoveL>::SharedPtr> move_l_servers_;
  std::vector<rclcpp_action::Server<MoveP>::SharedPtr> move_p_servers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  servo_j_subscriptions_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
  servo_p_subscriptions_;
  std::vector<FkPublisher> fk_publishers_;
};

}  // namespace humanoid_motion_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<humanoid_motion_server::HumanoidMotionControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("humanoid_motion_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
