#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "core_module.hpp"
#include "types/geometry.hpp"
#include "types/ik.hpp"
#include "types/robot_state.hpp"

namespace KDL {
class Tree;
}

namespace placo::kinematics {
struct ArmAngleTask;
struct FrameTask;
struct JointsTask;
class KinematicsSolver;
}

namespace placo::model {
class RobotWrapper;
}

namespace urdf {
class Model;
}

namespace motion_control {

enum class IkStatus {
  kSuccess,
  kModelNotLoaded,
  kSolverNotAvailable,
  kSolverFailed,
};

struct IkResult {
  IkStatus status{IkStatus::kSolverFailed};
  int solver_code{-1};
  std::string message;
  std::vector<std::string> joint_names;
  /// 参考关节角，单位 deg。
  std::vector<double> reference_positions;
  /// 求解得到的关节角，单位 deg。
  std::vector<double> positions;
  /// 目标位姿与解算结果 FK 位姿的位置误差，单位 mm。
  double position_error{0.0};
  /// 目标位姿与解算结果 FK 位姿的姿态误差，单位 rad。
  double orientation_error{0.0};

  bool success() const { return status == IkStatus::kSuccess; }
};

struct JacobianResult {
  bool success{false};
  std::string message;
  std::vector<std::string> joint_names;
  std::string base_frame;
  std::string link_frame;
  std::string reference_frame;
  /// 6 x N frame 雅可比矩阵，表示 link_frame 相对 base_frame 的 twist，
  /// 并使用 reference_frame 坐标轴表达。
  ///
  /// 线速度三行单位为 mm/rad，角速度三行单位为 rad/rad。
  std::vector<std::vector<double>> matrix;
};

struct ToolTransform {
  /// 工具/TCP 名称。FK/IK 的 link_name 命中该名称时，会按工具 TCP 语义处理。
  std::string name;
  /// 工具挂载的父 link/frame，必须是 URDF/Pinocchio 模型中存在的 frame。
  std::string parent_link;
  /// 工具 TCP 原点在 link/frame 坐标系下的位置，单位 mm。
  double x{0.0};
  double y{0.0};
  double z{0.0};
  /// 工具 TCP 坐标系相对 link/frame 坐标系的 ZYX 欧拉角，单位 rad。
  ///
  /// 旋转矩阵按 R = Rz(rz) * Ry(ry) * Rx(rx) 组合；也就是先绕局部 X
  /// 轴滚转 rx，再绕新的 Y 轴俯仰 ry，最后绕新的 Z 轴偏航 rz。
  double rx{0.0};
  double ry{0.0};
  double rz{0.0};
};

class Rkd : public CoreModule {
 public:
  Rkd();
  ~Rkd() override;

  Rkd(const Rkd&) = delete;
  Rkd& operator=(const Rkd&) = delete;
  Rkd(Rkd&&) noexcept;
  Rkd& operator=(Rkd&&) noexcept;

  std::string name() const override { return "RKD"; }

  /// 从 URDF 文件加载机器人模型。
  ///
  /// 加载后的模型会被当前 Rkd 实例的运动学/动力学查询复用。
  /// 重复调用该接口会替换之前已经加载的模型。
  ///
  /// \param model_path URDF 文件的绝对路径或相对路径。
  /// \return 加载成功返回 true，失败返回 false。
  virtual bool loadModel(const std::string& model_path);

  /// 注册常用关节组，例如 left_arm、right_arm、whole_body。
  virtual void registerJointGroup(const std::string& group_name,
                                  std::vector<std::string> joint_names);

  /// 查询已经注册的关节组。未找到时返回空向量。
  virtual std::vector<std::string> jointGroup(const std::string& group_name) const;

  /// 为指定 link/frame 配置工具 TCP 偏置。
  ///
  /// tool_transform.name 是工具/TCP 名称；tool_transform.parent_link 是该工具
  /// 挂载的父 link/frame，必须存在于机器人模型中。
  ///
  /// x/y/z/rx/ry/rz 表示 T_parent_link_tool：x/y/z 是工具 TCP 原点在
  /// parent_link 坐标系下的位置，单位 mm；rx/ry/rz 是工具 TCP 坐标系相对
  /// parent_link 坐标系的 ZYX 欧拉角，单位 rad，旋转矩阵按
  /// R = Rz(rz) * Ry(ry) * Rx(rx) 组合。
  ///
  /// 后续 FK/IK 中如果 link_name 命中 tool_transform.name，则对外语义变为
  /// 工具 TCP 位姿；没有命中配置的 link_name 保持原有行为。
  virtual void setToolTransform(const ToolTransform& tool_transform);

  /// 查询已经配置的工具 TCP 偏置。
  virtual bool toolTransform(const std::string& tool_name,
                             ToolTransform* tool_transform) const;

  /// 查询所有已经配置的工具 TCP 偏置。
  virtual std::vector<ToolTransform> toolTransforms() const;

  /// 按工具/TCP 名称清除工具配置。
  virtual void clearToolTransform(const std::string& tool_name);

  /// 清除所有工具 TCP 偏置。
  virtual void clearToolTransforms();

  /// 根据当前机器人状态计算指定 link/frame 在参考坐标系下的位姿。
  ///
  /// 输入关节角单位为 deg，输出位置单位为 mm。
  /// 如果任一 state.joint_groups[*].positions 的长度等于模型的 nq，则按完整配置向量读取。
  /// 否则，会根据 state.joint_groups 中各 JointState 的 group_name 解析关节组，
  /// 填充单自由度关节。
  ///
  /// \param state 当前机器人关节状态。
  /// \param link_name 目标 Pinocchio frame/link 名称。
  /// \param reference_frame 目标位姿的参考坐标系，默认 base_link。传空字符串时返回
  ///        模型世界位姿。
  /// \return 目标 frame 在 reference_frame 下的位姿。若模型未加载或找不到 frame，
  ///         则返回默认位姿。
  virtual types::Pose forwardKinematics(const types::RobotState& state,
                                        const std::string& link_name,
                                        const std::string& reference_frame = "base_link") const;

  /// 当前模型是否已经初始化可用的逆运动学求解器。
  virtual bool inverseKinematicsAvailable() const;

  /// 根据指定 IK 链起点和目标末端求逆运动学。
  ///
  /// target.position 单位为 mm，reference_state.joint_groups 中关节角单位为 deg。
  /// options.solver 默认使用 TRAC-IK。
  /// reference_state.joint_groups 会作为 IK 初值；冗余解偏好通过
  /// options.placo.redundancy_preference_positions 指定。
  ///
  /// target 使用 ik_base_link 作为参考坐标系；link_name 表示目标末端 frame/link。
  ///
  /// \param reference_state 参考关节状态，即 IK 初值。
  /// \param ik_base_link IK 链起点 frame/link 名称，也是 target 的参考坐标系。
  /// \param link_name 目标末端 link/frame 名称。
  /// \param target 目标末端位姿。
  /// \param options IK 后端和对应参数。不传时使用 TRAC-IK。
  /// \return 包含求解状态、求解器返回码、参考关节角、解和 FK 校验误差。
  ///         其中关节角为 deg，位置误差为 mm，姿态误差为 rad。
  virtual IkResult inverseKinematics(const types::RobotState& reference_state,
                                     const std::string& ik_base_link,
                                     const std::string& link_name,
                                     const types::Pose& target,
                                     const IkOptions& options = {}) const;

  /// 计算 link_frame 相对 base_frame 的雅可比矩阵，并使用 reference_frame 表达。
  /// link_frame/base_frame/reference_frame 可传模型 frame 名；若命中已配置工具名，
  /// 则按该工具 TCP frame 处理。
  ///
  /// 输入关节角单位为 deg。返回矩阵为 6 x N，前三行为线速度雅可比，单位
  /// mm/rad；后三行为角速度雅可比，单位 rad/rad。
  virtual JacobianResult frameJacobian(const types::RobotState& state,
                                       const std::string& base_frame,
                                       const std::string& link_name,
                                       const std::string& reference_frame,
                                       const std::vector<std::string>& joint_names) const;

  /// 兼容旧调用：计算 link_name 相对 base_link，并使用 base_link 坐标轴表达。
  virtual JacobianResult frameJacobian(const types::RobotState& state,
                                       const std::string& link_name,
                                       const std::vector<std::string>& joint_names) const;

 private:
  struct Impl;
  bool modelLoaded() const;
  std::string loadedModelPath() const;
  std::string defaultBaseLink() const;
  const KDL::Tree* tracIkTree() const;
  const urdf::Model* tracIkUrdfModel() const;
  placo::model::RobotWrapper* placoRobot() const;
  placo::kinematics::KinematicsSolver* placoSolver() const;
  placo::kinematics::FrameTask* placoFrameTask(
      const std::string& link_name,
      const Eigen::Affine3d& target_world) const;
  placo::kinematics::JointsTask* placoJointTask(bool enabled) const;
  placo::kinematics::ArmAngleTask* placoArmAngleTask(
      bool enabled,
      const std::string& shoulder_frame,
      const std::string& elbow_frame,
      const std::string& wrist_frame,
      const Eigen::Vector3d& elbow_hint_world) const;
  bool configuredToolTransform(const std::string& tool_name,
                               ToolTransform* tool_transform) const;
  IkResult solvePlacoInverseKinematics(
      const types::RobotState& reference_state,
      const std::string& base_link,
      const std::string& link_name,
      const types::Pose& target,
      const PlacoIkOptions& options) const;
  IkResult solveTracIkInverseKinematics(
      const types::RobotState& reference_state,
      const std::string& base_link,
      const std::string& link_name,
      const types::Pose& target,
      const TracIkOptions& options) const;

  std::unique_ptr<Impl> impl_;
};

}  // 命名空间 motion_control
