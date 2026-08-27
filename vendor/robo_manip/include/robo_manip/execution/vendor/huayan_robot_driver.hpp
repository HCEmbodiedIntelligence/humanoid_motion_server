#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "robo_manip/execution/execution.hpp"

namespace robo_manip::execution::vendor {

struct HuayanRobotDriverConfig {
  /// 控制器 IP，例如 192.168.1.10。
  std::string ip;
  /// 华沿 CPS 连接端口，SDK 默认 10003。
  int port{10003};
  /// 控制箱和机器人 ID，单臂第一版通常都使用 0。
  unsigned int box_id{0};
  unsigned int robot_id{0};
  /// robo_manip 侧单臂 group 名。
  std::string group_name;
  /// 全局速度倍率，SDK 要求 0.01..1.00；实机初期建议低速。
  double override_ratio{0.1};
  /// ServoJ 更新周期和前瞻时间。
  double servo_time_sec{0.02};
  double lookahead_time_sec{0.1};
};

/// 华沿 HR_Pro 6 轴单臂控制 SDK 到 robo_manip 执行层的适配器。
class HuayanRobotDriver : public RobotDriver {
 public:
  explicit HuayanRobotDriver(HuayanRobotDriverConfig config);
  ~HuayanRobotDriver() override;

  bool connect() override;
  void disconnect() override;
  bool isConnected() const override;
  bool readState(motion_control::types::RobotState* state) const override;
  bool isMoving(const std::string& group_name,
                bool* is_moving) const override;
  bool powerOn(const std::string& group_name) override;
  bool powerOff(const std::string& group_name) override;
  bool setJointMode(const std::string& group_name,
                    int velocity_ratio,
                    int acceleration_ratio) override;
  bool sendJointCommand(
      const std::string& group_name,
      const motion_control::types::JointTrajectory& command) override;
  void stop() override;
  bool startJointStream(const std::string& group_name,
                        double command_period_sec) override;
  bool sendJointStreamPoint(
      const std::string& group_name,
      const motion_control::types::JointTrajectoryPoint& point) override;
  void stopJointStream(const std::string& group_name, bool halt) override;

  /// 设置末端数字输出，对应华沿 SDK `HRIF_SetEndDO()`。
  /// @param bit End DO 位号。
  /// @param value 输出值，true 为 1，false 为 0。
  bool setEndDO(int bit, bool value);
  /// 读取末端数字输入，对应华沿 SDK `HRIF_ReadEndDI()`。
  /// @param bit End DI 位号。
  /// @param value 输出参数，返回读取到的 DI 值。
  bool readEndDI(int bit, int* value) const;
  /// 设置控制箱模拟输出，对应华沿 SDK `HRIF_SetBoxAOVal()`。
  /// @param bit Box AO 通道号。
  /// @param value AO 输出值，单位和范围由华沿 AO mode 决定。
  /// @param mode AO 模式，透传给华沿 SDK。
  bool setBoxAOVal(int bit, double value, int mode);
  /// 读取控制箱模拟输出，对应华沿 SDK `HRIF_ReadBoxAO()`。
  /// @param bit Box AO 通道号。
  /// @param mode 输出参数，返回当前 AO 模式。
  /// @param value 输出参数，返回当前 AO 输出值。
  bool readBoxAO(int bit, int* mode, double* value) const;

 private:
  HuayanRobotDriverConfig config_{};
  bool connected_{false};
  bool joint_stream_active_{false};
  std::chrono::steady_clock::time_point joint_stream_start_time_{};
  std::size_t joint_stream_point_count_{0};
};

}  // namespace robo_manip::execution::vendor
