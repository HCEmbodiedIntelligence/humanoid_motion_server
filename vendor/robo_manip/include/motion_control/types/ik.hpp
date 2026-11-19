#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace motion_control {

struct PlacoIkOptions {
  /// 最大 QP 迭代次数。每次迭代会应用一次 placo 求出的 delta-q。
  int max_iterations{100};
  /// 位置收敛阈值，单位 mm。
  double position_tolerance{1.0};
  /// 姿态收敛阈值，单位 rad。
  double orientation_tolerance{1e-3};
  /// 是否使用位置/姿态容差判断收敛。关闭时接受首次 QP 迭代结果。
  bool enable_tolerance_check{true};
  /// FrameTask 优先级，可取 "soft"、"hard" 或 "scaled"。
  std::string frame_task_priority{"hard"};
  double frame_position_weight{1.0};
  double frame_orientation_weight{1.0};
  /// PlaCo 求解器周期，单位 s。正值会写入 solver.dt。
  double solver_dt_sec{0.0};
  /// 是否增加 delta-q 正则任务。
  bool enable_regularization_task{false};
  double regularization_task_weight{1e-4};
  /// 是否增加末端可操作度任务（position + orientation）。
  bool enable_manipulability_task{false};
  double manipulability_task_weight{1e-3};
  double manipulability_task_lambda{1.0};
  /// 是否增加 JointTask，把解拉向 redundancy_preference_positions，帮助选择冗余解。
  bool enable_joint_task{false};
  /// JointTask 权重，仅在 enable_joint_task=true 时生效。
  double joint_task_weight{1e-3};
  /// JointTask 的冗余解偏好关节角，单位 deg；顺序与本次 IK 输出 joint_names 一致。
  std::vector<double> redundancy_preference_positions;
  /// 是否在 IK 收敛成功后校验模型关节位置限位。
  bool enable_joint_limits{true};
  /// placo 默认使用 floating base；固定底座机械臂通常应保持 mask。
  bool mask_floating_base{true};
  /// 指定求解和返回的关节。为空时优先使用 joint_group，再使用 reference_state
  /// 中 group_name 对应的 RKD 关节组。
  std::vector<std::string> joint_names;
  std::string joint_group;
  /// 是否增加 ArmAngleTask，引导七轴冗余解让肘部靠近期望偏置方向。
  bool enable_arm_angle_task{false};
  /// ArmAngleTask 权重，仅在 enable_arm_angle_task=true 时生效。
  double arm_angle_task_weight{1e-7};
  /// 臂角几何定义用到的肩、肘、腕 frame。为空时按末端名推断常见左/右臂或测试模型。
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;
  /// 期望肘部偏置方向由某个 frame 下的局部轴给出；内部会转成世界坐标并投影到垂直肩腕轴的平面。
  /// frame 为 "world" 时 arm_angle_elbow_hint_axis 直接按世界坐标解释。
  std::string arm_angle_elbow_hint_frame{"world"};
  Eigen::Vector3d arm_angle_elbow_hint_axis = Eigen::Vector3d::UnitZ();
};

enum class TracIkSolveType {
  kSpeed,
  kDistance,
  kManip1,
  kManip2,
};

struct TracIkOptions {
  /// 每次 TRAC-IK 求解允许的最长时间，单位 s。
  double timeout{0.005};
  /// TRAC-IK 内部收敛精度，单位 m/rad。
  double epsilon{1e-5};
  TracIkSolveType solve_type{TracIkSolveType::kSpeed};
  /// 输出结果的外层校验阈值，位置单位 mm，姿态单位 rad。
  double position_tolerance{1.0};
  double orientation_tolerance{1e-3};
  /// 指定求解和返回的关节。为空时优先使用 joint_group，再使用 reference_state
  /// 中 group_name 对应的 RKD 关节组，最后使用 KDL chain 中的可动关节。
  std::vector<std::string> joint_names;
  std::string joint_group;
  /// 是否优先使用 preference_positions 作为 seed，并用 Distance 模式求解。
  /// 若该模式失败，会自动回退到 reference_state seed + Speed 模式。
  bool enable_preference_positions{false};
  /// 偏好关节角，单位 deg；顺序与本次 TRAC-IK 输出 joint_names 一致。
  std::vector<double> preference_positions;
  /// 是否在 IK 成功后校验 TRAC-IK/KDL 返回的关节限位。
  bool enable_joint_limits{true};
};

enum class IkSolverType {
  kTracIk,
  kPlaco,
  /// 新增：速度级 QP 伺服逆解。每调用一次只走一小步（实时伺服语义），
  /// 详见 motion_control/include/qp_servo_ik.hpp 与 src/rkd/rkd_qp_servo.cpp。
  kQpServo,
};

/// 速度级 QP 伺服逆解参数。
///
/// 与 PlacoIkOptions / TracIkOptions 定位一致：只描述「这一次求解怎么做」，
/// 不描述机器人模型。默认值与工程 A `HC_ROBOT/tools/ik_bridge.cpp` 一致，
/// 保证「搬过来跑」与「原来跑」行为相同，便于对照调试。
///
/// 调参优先级（影响从大到小）：
///   1. joint_v_max_deg —— 唯一硬限速，务必与 rtc.max_velocity 一致；
///   2. gain            —— 响应快慢的主旋钮；
///   3. damping         —— 奇异位形附近稳不稳；
///   4. position/orientation weight —— 不可达时「保位姿还是保位置」；
///   5. dt_sec          —— 必须等于真实周期，否则限速折算错误。
struct QpServoIkOptions {
  // -------------------------------------------------------------------------
  // 速度级 QP：每拍能走多远、走多急
  // -------------------------------------------------------------------------

  /// 每步消除的误差比例 g ∈ (0, 1]。
  ///
  /// 闭环上是离散一阶低通，时间常数 τ ≈ dt_sec / g
  /// （g=0.5、dt=10 ms → τ≈20 ms，约 6~7 拍消除 99%）。
  ///   · 调大 → 跟手更紧；过大会过冲/抖动，尤其接近奇异时；
  ///   · 调小 → 平顺、超调少；但跟踪滞后变大，遥操作手感变"肉"。
  /// 推荐 0.3 ~ 0.8，遥操作常用 0.5。
  double gain{0.5};

  /// 控制周期 Δt，单位 s。
  ///
  /// 速度上限是按 v_max·Δt 折算成位移盒的，所以这个值必须等于**真实**周期。
  /// 填大 → 每拍允许位移偏大（会超速）；填小 → 人为限速（跟不上）。
  /// MoveLine 的实时路径每拍会用 OptParams.dt_sec 覆盖它，正常无需手工维护。
  double dt_sec{0.01};

  /// 关节速度上限，单位 deg/s。
  ///
  /// 求解器内部**唯一**的硬限速来源。建议与 RtcConfig::max_velocity 相同：
  ///   · 这里更小 → 求解器先饱和，最终关节 RTC 永远用不满，表现为整体变慢；
  ///   · 这里更大 → 最终关节 RTC 才是实际限速层，两层滤波叠加 → 响应更滞后。
  /// 默认 90（与工程 A 的 yaml 一致）。
  double joint_v_max_deg{90.0};

  /// 最小二乘阻尼 λ（实际加到对角线上的是 λ²）。
  ///
  /// 奇异位形附近 J 的最小奇异值很小时，伪逆会给出极大步长；阻尼把
  /// s ≈ (JᵀJ + λ²I)⁻¹Jᵀe，等价于截断「转很多度只换来一点点位移」的方向。
  ///   · 调大 → 更稳不甩，但正常位形也变迟钝，且留下稳态残差；
  ///   · 调小 → 精度好，奇异处可能突变。
  /// 推荐 1e-3 ~ 1e-2，默认 2e-3。
  double damping{0.002};

  // -------------------------------------------------------------------------
  // 误差归一化：位置与姿态各算多少分
  // -------------------------------------------------------------------------
  //
  // 进 QP 前残差被缩放为 ẽ = [ (√w_p/σ_p)·e_p ; (√w_o/σ_o)·e_o ]，
  // 真正起作用的是 w/σ² 这个比值。要改强弱建议只调 w，
  // 因为 σ 同时改变数值尺度（对 λ 的相对影响也会变）。

  /// 位置任务相对权重。
  double position_weight{1.0};

  /// 姿态任务相对权重。
  ///
  /// 目标不可达时的折中方向：调大它 → 宁可位置差一点也要保姿态；
  /// 调小它 → 宁可姿态歪一点也要贴近位置。遥操作一般保持 1:1。
  double orientation_weight{1.0};

  /// 位置误差归一化尺度 σ_p，单位 mm。默认 50 —— 末端差 5 cm 记 1 分。
  double norm_position_mm{50.0};

  /// 姿态误差归一化尺度 σ_o，单位 rad。默认 0.2 rad ≈ 11.5°。
  double norm_orientation_rad{0.2};

  // -------------------------------------------------------------------------
  // 收敛判据：只影响「何时提前退出」与上报，不影响单步步长
  // -------------------------------------------------------------------------

  /// 位置收敛阈值，单位 mm。
  /// iterations==1 时仅用于填 IkResult 的 converged 与 message 文案。
  double position_tolerance{0.5};

  /// 姿态收敛阈值，单位 rad。
  double orientation_tolerance{1e-3};

  // -------------------------------------------------------------------------
  // 关节限位
  // -------------------------------------------------------------------------

  /// 关节位置下限/上限，单位 deg，与关节组同序。
  ///
  /// 留空时的回退顺序：模型限位（Rkd::jointGroupLimits 读 placo 的 URDF 限位）
  /// → ±180° 兜底。**真机不要依赖 ±180° 兜底**：限位错了会把手臂往硬限位上推，
  /// 虽然还有最终关节 RTC 与上层限位校验，但那已经变成「被拒 → 受控停止」。
  std::vector<double> joint_lower_deg;
  std::vector<double> joint_upper_deg;

  // -------------------------------------------------------------------------
  // 求解范围
  // -------------------------------------------------------------------------

  /// 关节组名。留空时由分发层用 placo / trac_ik 的 joint_group 补齐，
  /// 再退到 reference_state 的第一个 group（MoveLine 场景下就是请求的组）。
  std::string joint_group;

  /// 显式关节顺序。留空则用 jointGroup(joint_group) 的注册顺序。
  /// 若指定，必须与 seed 顺序、雅可比列顺序一致。
  std::vector<std::string> joint_names;

  // -------------------------------------------------------------------------
  // 臂角（7 轴冗余）
  // -------------------------------------------------------------------------
  //
  // ψ 是自运动流形上的坐标：ψ=0 表示肘部朝 hint 方向，ψ≠0 表示肘部偏转。
  // 选哪个 ψ 决定肘部往哪边拐，直接影响可操作度与是否撞本体。

  /// 是否启用臂角任务。三个 frame 名任一为空则整条链路关闭（即使这里为 true）。
  bool enable_arm_angle_task{false};

  /// 肩 / 肘 / 腕三个 frame 名。必须在模型里存在；
  /// 典型取关节 1/4/7 所在的 link（如 openarmx_left_link1 / link4 / link7）。
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;

  /// off | best_effort | forced
  ///   off         —— 不参与求解（仍会算并上报 ψ，便于只观察）；
  ///   best_effort —— 只走零空间，只用位姿步之后的剩余预算（35%），位姿优先；
  ///   forced      —— 与位姿争用同一份预算（100%），位姿可让步。
  /// 遥操作用 best_effort；"先摆肘部再动末端"的规划用 forced。
  std::string arm_angle_mode{"off"};

  /// 臂角每步收敛比例，0~1。调大更快，但占用零空间预算更多。
  double arm_angle_gain{0.5};

  /// 期望臂角 ψ_d，单位 deg（0 表示肘部朝 hint 方向）。
  double arm_angle_target_deg{0.0};

  /// 臂角收敛容差，单位 rad。默认 5e-3 ≈ 0.29°。
  double arm_angle_tolerance{5e-3};

  /// 期望肘部方向，表达在 **ik_base_link（臂基座）坐标系**，内部会归一化。
  /// 注意旧库 PlacoIkOptions 的 hint 有 frame 约定（默认 world），
  /// 这里固定按基座系解释，跨机型移植时不要直接照抄 world 系数值。
  Eigen::Vector3d arm_angle_elbow_hint_axis = Eigen::Vector3d::UnitZ();

  // -------------------------------------------------------------------------
  // 迭代
  // -------------------------------------------------------------------------

  /// 单次调用内最多走几步。
  ///   1    → 实时伺服（遥操作 / ServoP / MoveL 每拍一次）。**默认**。
  ///   50   → 快速近似收敛，看看最近可达点在哪。
  ///   200  → 实测到残差地板（≈2.6 ms @真实 SDK），适合路点/离线。
  /// 规划类调用必须是 >1，否则只会得到「走了一步」的关节轨迹。
  ///
  /// ⚠ **实时场景请保持 1**：每个 solveOnce 都按 joint_v_max_deg·dt_sec
  /// 单独做盒约束，一次调用走 N 步，累计位移最多 N·v_max·Δt，
  /// 等于把每拍速度上限放大 N 倍。要更快响应请调 gain，不要加步数。
  int iterations{1};

  /// 盒约束 QP（坐标下降）最大扫描次数。
  /// 内部在 max|Δx| < 1e-12 时提前退出，给大一般不多花时间。
  /// 一般无需修改；某轴长期贴边且收敛慢时可试 100~200。
  int coordinate_descent_sweeps{60};
};


struct IkOptions {
  IkSolverType solver{IkSolverType::kTracIk};
  TracIkOptions trac_ik;
  PlacoIkOptions placo;
  QpServoIkOptions qp_servo;
};

}  // namespace motion_control
