#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core_module.hpp"
#include "types/geometry.hpp"
#include "types/robot_state.hpp"

namespace motion_control {

/// WBC 求解状态。
enum class WbcStatus {
  kSuccess,
  kModelNotLoaded,
  kInvalidInput,
  kSolverFailed,
};

/// WBC 任务优先级。
enum class WbcTaskPriority {
  kSoft,
  kHard,
};

/// WBC 求解器配置。
struct WbcConfig {
  std::string model_path;
  std::string active_group{"whole_body"};
  /// active_group 对应的显式关节列表。为空时，configure() 会把模型中所有
  /// 单自由度关节注册为 active_group。
  std::vector<std::string> active_joint_names;
  double control_period_sec{0.002};
  bool enable_joint_limits{true};
  bool enable_velocity_limits{true};
  bool mask_floating_base{true};
  /// 所有关节的默认 delta-q 正则权重。
  double regularization_weight{1e-6};
  /// 指定关节的 delta-q 正则权重，覆盖 regularization_weight。
  std::map<std::string, double> joint_regularization_weights;
  int solve_iterations{1};
};

/// 末端位姿任务。
struct WbcFrameTask {
  std::string name;
  /// 被控制的机器人 frame/link 名称。
  std::string link_name;
  /// target_pose 的参考坐标系。默认 "world" 表示 PlaCo 世界坐标系；
  /// 也可以填 "base_link" 或其他模型 frame，此时内部会转换为 world 目标。
  std::string reference_frame{"world"};
  /// 目标位姿，位置单位为 mm，姿态使用 Pose::euler_zyx_deg（ZYX 欧拉角，deg）；
  /// 参考系由 reference_frame 指定。
  types::Pose target_pose{};
  double position_weight{1.0};
  double orientation_weight{1.0};
  WbcTaskPriority priority{WbcTaskPriority::kSoft};
};

/// 两个 frame 的相对位置任务，target_a_to_b 单位为 mm。
struct WbcRelativePositionTask {
  std::string name;
  std::string frame_a;
  std::string frame_b;
  types::Vector3 target_a_to_b{};
  double weight{1.0};
  WbcTaskPriority priority{WbcTaskPriority::kSoft};
};

/// 关节姿态正则/目标任务，关节位置单位为 deg。
struct WbcJointPostureTask {
  std::string name;
  std::vector<std::string> joint_names;
  std::vector<double> target_positions;
  double weight{1e-4};
};

/// 单周期 WBC 任务请求。
struct WbcRequest {
  /// 末端位姿任务列表。调用 configure() 后，后续 updateTaskTargets()
  /// 需要保持列表数量、顺序、link_name 和 reference_frame 不变，只更新目标位姿/权重等参数。
  std::vector<WbcFrameTask> frame_tasks;
  /// 相对位置任务列表。frame_a/frame_b 决定 PlaCo 内部 task 结构，
  /// 实时循环中应只更新 target_a_to_b 和权重。
  std::vector<WbcRelativePositionTask> relative_position_tasks;
  /// 关节姿态任务列表。joint_names 决定 task 结构，实时循环中应只更新
  /// target_positions 和权重。
  std::vector<WbcJointPostureTask> posture_tasks;
};

/// 全身控制器输出的速度级命令。
struct WbcCommand {
  WbcStatus status{WbcStatus::kSolverFailed};
  std::string message;
  /// 移动底盘速度命令；linear 为 mm/s，angular 为 rad/s。
  types::Twist base_velocity{};
  /// 与 joint_positions/joint_velocity 一一对应的关节名。
  std::vector<std::string> joint_names;
  /// PlaCo 本周期求解后的预测/目标关节位置，单位 deg。
  std::vector<double> joint_positions;
  /// 关节速度命令，单位 deg/s。
  std::vector<double> joint_velocity;
};

/// 全身控制模块，用于把任务请求分配到底盘和机械臂自由度。
class Wbc : public CoreModule {
 public:
  /// 构造空的 WBC 控制器。构造后尚未加载模型和任务，需要先调用 configure()。
  Wbc();

  /// 释放 PlaCo 模型、求解器和已缓存任务。
  ~Wbc() override;

  /// WBC 内部持有 PlaCo solver 和 task 指针，不支持拷贝。
  Wbc(const Wbc&) = delete;

  /// WBC 内部持有 PlaCo solver 和 task 指针，不支持拷贝赋值。
  Wbc& operator=(const Wbc&) = delete;

  /// 支持移动构造，用于把控制器实例转移到容器或上层模块中。
  Wbc(Wbc&&) noexcept;

  /// 支持移动赋值；移动后源对象不再拥有 PlaCo 资源。
  Wbc& operator=(Wbc&&) noexcept;

  /// 返回模块名称，便于日志、诊断和统一模块管理。
  std::string name() const override { return "WBC"; }

  /// 加载机器人模型、初始化 PlaCo solver，并创建 WBC 任务结构。
  ///
  /// 该接口会清空旧模型、旧 solver 和旧任务缓存，然后按 config.model_path
  /// 创建 RobotWrapper/KinematicsSolver，再按 request 创建 PlaCo task。
  /// config.active_joint_names 非空时会注册为 config.active_group；为空时会把
  /// 模型中所有单自由度关节作为 config.active_group。
  ///
  /// \return 配置成功返回 true；模型路径为空、周期非法、模型加载失败或任务非法
  ///         时返回 false。
  virtual bool configure(const WbcConfig& config, const WbcRequest& request);

  /// 返回当前 WBC 配置。
  ///
  /// 注意：返回的是内部配置引用，调用方不要长期保存该引用跨越下一次 configure()。
  virtual const WbcConfig& config() const;

  /// 注册一个可参与 WBC 优化的关节组。
  ///
  /// joint_names 的顺序决定 solveVelocityLevel() 输出 joint_names、
  /// joint_positions 和 joint_velocity 的顺序。输入 RobotState 中 active_group
  /// 对应的 positions 也必须按同样顺序排列。
  virtual void registerJointGroup(std::string group_name,
                                  std::vector<std::string> joint_names);

  /// 查询已注册关节组。未找到时返回空 vector。
  virtual std::vector<std::string> jointGroup(const std::string& group_name) const;

  /// 只更新已缓存任务的目标和权重，不重建 PlaCo task。
  ///
  /// request 的任务数量、顺序、link/frame/joint_names 必须与 configure()
  /// 时一致。这样可以避免实时循环中反复分配/释放 task。
  ///
  /// \return 更新成功返回 true；未配置任务或任务结构变化时返回 false。
  virtual bool updateTaskTargets(const WbcRequest& request);

  /// 清空已缓存任务。清空后 solveVelocityLevel(state) 会返回 kInvalidInput。
  virtual void clearTasks();

  /// 是否已经通过 configure() 成功初始化任务结构。
  virtual bool tasksConfigured() const;

  /// 使用已缓存的任务求解速度级全身控制命令。
  ///
  /// state 必须包含 config().active_group 对应的 JointState，且 positions 长度
  /// 必须与该关节组一致。输出 joint_positions 为 PlaCo solve(true) 后的预测位置
  /// deg，joint_velocity 为 (joint_positions - 输入 positions) / control_period_sec。
  ///
  /// 该接口不会创建或删除 task，适合实时控制周期调用。
  virtual WbcCommand solveVelocityLevel(const types::RobotState& state);

  /// 便捷接口：任务结构不变时先更新目标再求解。
  ///
  /// 该接口不会在任务结构变化时自动重建任务；如需改变 link/frame/joint_names，
  /// 请重新调用 configure(config, request)。
  virtual WbcCommand solveVelocityLevel(const types::RobotState& state,
                                        const WbcRequest& request);

 private:
  /// 内部创建 PlaCo task 缓存。对外统一通过 configure(config, request) 调用。
  bool configureTaskCache(const WbcRequest& request);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // 命名空间 motion_control
