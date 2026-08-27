#pragma once

#include <string>

#include "robo_manip/execution/execution.hpp"

namespace robo_manip::execution::vendor {

struct TianjiRobotDriverConfig {
  /// 控制器 IP，例如 192.168.58.2。
  std::string ip;
  /// robo_manip 侧左/右臂 group 名，用于映射到 SDK 的 A/B 臂。
  std::string left_group_name;
  std::string right_group_name;
  /// SDK 日志开关：0 关闭，1 打开。
  int log_switch{0};
  /// SDK 位置模式速度/加速度百分比，安全起见建议实机初期保持较小值。
  int velocity_ratio{0};
  int acceleration_ratio{0};
};

/// 天机/Marvin 控制 SDK 到 robo_manip 执行层的适配器。
///
/// SDK 是进程级单连接模型，因此通常只在系统初始化阶段创建一个实例，
/// 然后通过 MotionContext 共享给规划和执行流程。
class TianjiRobotDriver : public RobotDriver {
 public:
  explicit TianjiRobotDriver(TianjiRobotDriverConfig config);
  ~TianjiRobotDriver() override;

  bool connect() override;
  void disconnect() override;
  bool isConnected() const override;
  bool readState(motion_control::types::RobotState* state) const override;
  bool powerOn(const std::string& group_name) override;
  bool powerOff(const std::string& group_name) override;
  bool sendJointCommand(
      const std::string& group_name,
      const motion_control::types::JointTrajectory& command) override;
  void stop() override;

  /// 将指定 group 对应的天机 A/B 臂切到关节位置模式，并设置速度/加速度百分比。
  bool setJointMode(const std::string& group_name,
                    int velocity_ratio,
                    int acceleration_ratio) override;

 private:
  char armForGroup(const std::string& group_name) const;

  TianjiRobotDriverConfig config_{};
  bool connected_{false};
};

}  // namespace robo_manip::execution::vendor
