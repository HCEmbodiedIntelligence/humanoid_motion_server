#pragma once

#include <string>
#include <vector>

#include "robo_manip/tasks/common/basic_task.hpp"
#include "rkd.hpp"
#include "rtc.hpp"

namespace robo_manip::tasks {

class DualArmCoop : public BasicTask {
 public:
  /// 双臂协同 IK 模式。
  enum class IkMode {
    /// 左右臂分别用各自 group 做 Placo IK，再把两个解拼接输出。
    kIndependentArmGroups,
    /// 使用用户配置的 group 做 Placo IK，适合腰+双臂等全身协同。
    kWholeBodyGroup,
  };

  /// 双臂协同请求。
  ///
  /// object_path 表示双臂共同搬运/操作的刚体路径；left_grasp_in_object 和
  /// right_grasp_in_object 表示左右末端相对该刚体坐标系的固定抓取位姿。
  /// 以上 Pose 均使用 mm 位置和 Pose::euler_zyx_deg 姿态（ZYX 欧拉角，deg）；
  /// Pose::orientation 仅作为内部四元数缓存。
  struct CoopRequest {
    std::string request_id;
    /// 全身协同时使用的关节组，例如 waist_dual_arm 或 whole_body。
    std::string group_name{"whole_body"};
    /// 独立双臂 IK 时左/右臂各自使用的关节组。
    std::string left_group_name{"left_arm"};
    std::string right_group_name{"right_arm"};
    /// 左/右臂 IK 链起点。真实双臂机器人通常左右臂 base 不同。
    std::string left_base_link{"base_link"};
    std::string right_base_link{"base_link"};
    std::string left_link_name;
    std::string right_link_name;
    /// 物体几何路径。base_link 是物体位姿表达坐标系；link_name 是物体 frame 名称。
    /// 实时模式使用 points.front() 作为初始物体位姿。
    motion_control::types::CartesianTrajectory object_path{};
    motion_control::types::Pose left_grasp_in_object{};
    motion_control::types::Pose right_grasp_in_object{};
    motion_control::types::MotionLimits limits{};
    motion_control::types::PlanningParameters parameters{};
  };

  /// 外部左右 TCP 起终点请求。
  ///
  /// 该接口用于外部轴/上层规划器已经给出左右手 TCP 起点和目标点的场景。
  /// 左/右 TCP 点分别在 left_base_link/right_base_link 下表达。任务内部
  /// 会用相同采样数量生成左右两条端点路径，并在各自 base 下分别做 IK；
  /// 每个采样 index 的左右关节解会拼成同一个 joint path 点，最后调用
  /// RMP/TOPP-RA 做关节空间统一时间参数化。
  struct EndpointObjectRequest {
    /// x/y/z 单位 mm；rx/ry/rz 单位 deg。
    ///
    /// RPY 按 ZYX 欧拉角组合：R = Rz(rz) * Ry(ry) * Rx(rx)。
    struct CartesianRpyPose {
      double x{0.0};
      double y{0.0};
      double z{0.0};
      double rx{0.0};
      double ry{0.0};
      double rz{0.0};
    };

    std::string request_id;
    /// 全身协同时使用的关节组，例如 waist_dual_arm 或 whole_body。
    std::string group_name{"whole_body"};
    /// 独立双臂 IK 时左/右臂各自使用的关节组。
    std::string left_group_name{"left_arm"};
    std::string right_group_name{"right_arm"};
    /// 左/右臂 IK 链起点。left/right TCP 起终点分别用各自 base 表达。
    std::string left_base_link{"base_link"};
    std::string right_base_link{"base_link"};
    std::string left_link_name;
    std::string right_link_name;
    /// 左/右 TCP 起点和目标点，分别用 left_base_link/right_base_link 表达。
    CartesianRpyPose left_start_in_base{};
    CartesianRpyPose left_goal_in_base{};
    CartesianRpyPose right_start_in_base{};
    CartesianRpyPose right_goal_in_base{};
    motion_control::types::MotionLimits limits{};
    motion_control::types::PlanningParameters parameters{};
  };

  /// 实时接口输入的物体目标位姿：位置单位 mm，姿态使用
  /// Pose::euler_zyx_deg（ZYX 欧拉角，deg）。
  using CartesianPos = motion_control::types::Pose;

  /// 单个关节组的命令。positions/velocities/efforts 使用该 group 在 RKD 中注册的顺序。
  struct JointGroupCommand {
    std::string group_name;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
  };

  /// 双臂协同输出的关节命令。
  ///
  /// 独立双臂模式输出 left/right 两个 group；全身协同模式输出 group_name 一个 group。
  struct JointPos {
    std::vector<JointGroupCommand> groups;
  };

  struct OptParams {
    double dt_sec{0.02};
    IkMode ik_mode{IkMode::kIndependentArmGroups};
    motion_control::IkOptions left_ik_options{};
    motion_control::IkOptions right_ik_options{};
  };

  motion_control::types::PlanningResult plan(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const motion_control::types::PlanningRequest& request) override;

  /// 离线双臂协同路径规划：先根据物体路径和固定抓取位姿生成整体关节几何路径，
  /// 再调用 RMP/TOPP-RA 对该关节路径统一时间参数化。
  motion_control::types::PlanningResult planCoopPath(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const CoopRequest& request,
      const OptParams& opt);

  motion_control::types::PlanningResult planCoopPath(
      const core::MotionContext& context,
      const CoopRequest& request,
      const OptParams& opt);

  /// 根据外部左右 TCP 起终点生成同步关节路径。
  ///
  /// 左/右 TCP 路径使用同一个采样数量，但位姿始终分别在各自 base 下表达。
  /// 每个采样点分别做 IK 后合并成整体 joint path，再调用 RMP/TOPP-RA
  /// 做关节空间时间参数化，从而保证两臂轨迹点按同一时间律同步执行。
  /// PlaCo 首次求解失败时，会使用 soft FrameTask、低姿态权重、正则与
  /// 可操作度任务，从初始状态重新计算左右臂整段路径一次；重试结果不检查
  /// 位置/姿态容差。
  motion_control::types::PlanningResult planEndpointObjectPath(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const EndpointObjectRequest& request,
      const OptParams& opt);

  motion_control::types::PlanningResult planEndpointObjectPath(
      const core::MotionContext& context,
      const EndpointObjectRequest& request,
      const OptParams& opt);

  bool StartRealtimeDualArmCoop(
      const core::MotionContext& context,
      const motion_control::types::RobotState& state,
      const CoopRequest& request,
      const OptParams& opt);

  bool TickRealtimeDualArmCoop(const CartesianPos& object_target,
                               JointPos& command_joint,
                               bool& reached,
                               const OptParams& opt);

  void StopRealtimeDualArmCoop();

 private:
  core::MotionContext realtime_context_{};
  motion_control::types::RobotState realtime_state_{};
  CoopRequest realtime_request_{};
  bool realtime_active_{false};
};

}  // namespace robo_manip::tasks
