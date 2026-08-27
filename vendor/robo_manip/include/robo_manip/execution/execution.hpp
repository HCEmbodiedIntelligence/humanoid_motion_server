#pragma once

#include <memory>
#include <string>

#include "types/robot_state.hpp"
#include "types/task.hpp"
#include "types/trajectory.hpp"

namespace robo_manip::execution {

enum class ExecutionStatus {
  kSucceeded,
  kFailed,
  kCanceled,
  kTimeout,
  kNotConfigured,
  kInvalidTrajectory,
};

struct ExecutionResult {
  bool success{false};
  ExecutionStatus status{ExecutionStatus::kFailed};
  std::string message;
  double duration_sec{0.0};
};

struct ExecuteOptions {
  std::string group_name;
  double command_period_sec{0.02};
  double timeout_sec{0.0};
  bool stop_on_failure{true};
};

class RobotDriver {
 public:
  virtual ~RobotDriver() = default;

  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;
  virtual bool readState(motion_control::types::RobotState* state) const = 0;
  /// 查询指定关节组对应的机器人当前是否仍在运动。
  /// 返回值表示查询是否成功，运动状态通过 is_moving 输出。
  virtual bool isMoving(const std::string& group_name,
                        bool* is_moving) const;
  virtual bool powerOn(const std::string& group_name) = 0;
  virtual bool powerOff(const std::string& group_name) = 0;
  virtual bool setJointMode(const std::string& group_name,
                            int velocity_ratio,
                            int acceleration_ratio) = 0;
  virtual bool sendJointCommand(
      const std::string& group_name,
      const motion_control::types::JointTrajectory& command) = 0;
  virtual void stop() = 0;

  /// 启动关节流式命令。连续点动等实时任务应先调用该接口，再逐周期
  /// sendJointStreamPoint()，最后 stopJointStream()。
  virtual bool startJointStream(const std::string& group_name,
                                double command_period_sec);
  /// 下发一个控制周期的关节位置/速度命令。
  virtual bool sendJointStreamPoint(
      const std::string& group_name,
      const motion_control::types::JointTrajectoryPoint& point);
  /// 停止关节流式命令。halt=true 时底层执行安全停止。
  virtual void stopJointStream(const std::string& group_name, bool halt);
};

class MockRobotDriver : public RobotDriver {
 public:
  MockRobotDriver() = default;
  explicit MockRobotDriver(motion_control::types::RobotState initial_state);

  void setState(motion_control::types::RobotState state);

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

 private:
  bool connected_{false};
  motion_control::types::RobotState state_{};
};

class TrajectoryExecutor {
 public:
  explicit TrajectoryExecutor(std::shared_ptr<RobotDriver> driver);

  ExecutionResult execute(const motion_control::types::PlanningResult& plan,
                          const ExecuteOptions& options = {});
  ExecutionResult execute(const motion_control::types::JointTrajectory& trajectory,
                          const ExecuteOptions& options = {});

 private:
  std::shared_ptr<RobotDriver> driver_;
};

}  // namespace robo_manip::execution
