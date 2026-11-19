// qp_servo_ik.hpp
//
// 速度级 QP 伺服逆解（qp_servo）—— 纯 Eigen 实现，不依赖 SDK/ROS。
//
// 与 ik_bridge.cpp 中 SolveQpServoSingle() 的算法一致，差异仅在于：
//   · 运动学通过抽象接口注入（SDK 侧用 Rkd，基准用解析模型）；
//   · 臂角的三个 frame 由参数指定，不写死 *_link_1/4/7；
//   · 自由度不再限定 n>=7（n==7 走精确零向量，n>7 走 SVD）；
//   · 增加 iterations：1 = 每拍一步（实时遥操），N>1 = 迭代到容差（路点/离线）。
//
// 单位约定（与 motion_control 一致）：
//   关节角 deg，位置 mm，雅可比 0~2 行 mm/rad、3~5 行 rad/rad。
//
// ===========================================================================
// 它在解什么
// ===========================================================================
// 与"把目标位姿一次解完"的绝对位姿 IK 不同，这里每调用一次只解**一步关节位移** s：
//
//   min  ½‖W^{½}(J·s − g·e)‖² + ½λ²‖s‖²
//   s.t. max(q_lo − q, −v_max·Δt) ≤ s ≤ min(q_hi − q, +v_max·Δt)      (逐关节盒约束)
//
//   q_{k+1} = q_k + s
//
// 其中 e = [p_d − p(q);  log(R_d R(q)^T)] 为位姿残差，g 是每步消除比例，
// W 由 position/orientation 权重与归一化尺度构成。盒约束同时编码了
// **关节位置限位**与**关节速度上限**，因此输出天然不会超速、不会撞限位，
// 不需要事后裁剪。
//
// 单步内部顺序（见 solveOnce）：
//   ① 限位/seed 归一  →  ② 算残差  →  ③ 算雅可比  →  ④ 组成加权最小二乘
//   → ⑤ 坐标下降解盒约束 QP  →  ⑥ 回溯线搜索单调性守卫  →  ⑦ 臂角零空间修正
//   → ⑧ 应用并用 FK 复核残差
//
// 每步固定 **6 次 FK + 1 次雅可比**（不启用臂角时）。这是可预测的常数开销：
// 实测约 5 µs（解析模型）/ 约 13 µs（真实 SDK，含 Rkd 的 FK/雅可比）。
//
// ===========================================================================
// 两种用法（由 Options::iterations 决定）
// ===========================================================================
//   iterations = 1   → 实时伺服：返回"本拍该走到哪"，由调用方每拍调一次。
//                      不做收敛，耗时与目标是否可达**无关**。遥操作走这个。
//   iterations = N>1 → 求近似收敛解：反复走步直到满足容差或用尽预算。
//                      目标不可达时残差会停在一个地板（最近可达点），
//                      实测约 200 步到底（≈2.6 ms @真实 SDK）。
//
// ===========================================================================
// 调参速查（按影响从大到小）
// ===========================================================================
//   joint_v_max_deg_s  决定"每拍最多走多少"，是唯一的硬限速，务必与
//                      rtc.max_velocity 一致，否则两层限速会互相掩盖。
//   gain               决定"每拍消除多少误差"，是响应快慢的主旋钮。
//   damping            决定奇异位形附近是否会被巨大步长带飞。
//   orientation/position weight  不可达时"保姿态还是保位置"的折中。
//   dt_sec             **必须等于真实控制周期**，否则 v_max·Δt 算错。
//
// 已知近似：线搜索与收敛判据只看 6 维位姿代价，不含曲率项；臂角修正走
// 零空间，理论上不改变位姿，但实现里用精确零向量（n=7）或 SVD（n>6），
// 在接近奇异时零向量数值条件变差，可能带来微小位姿扰动。

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace qp_servo {

/// 位姿：位置 mm + 3x3 旋转矩阵（不用四元数，避免反复归一化带来的抖动）。
///
/// 表达坐标系固定为「臂基座系」（SDK 里的 ik_base_link），
/// 与 `Rkd::forwardKinematics(state, link, reference_frame)` 的 reference_frame 一致。
struct Pose {
  Eigen::Vector3d position_mm{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

/// 运动学注入点。
///
/// 求解器本身不认识"机器人"二字，只通过这 5 个方法问模型问题。这样做的好处：
///   · SDK 侧用 Rkd 实现，确保正解/雅可比与 MoveLine、RTC 用的是**同一个模型**
///     （否则路径与正解不一致，会出现"参考位姿和真实末端系统性偏差"）；
///   · 基准测试用解析模型实现，不需要 pinocchio/placo 就能测时；
///   · 单元测试可以塞进任何假模型。
///
/// 单位约定（必须严格遵守，否则限速会差 1000 倍）：
///   q 一律 deg；位置一律 mm；雅可比线性行 mm/rad、角行 rad/rad。
///
/// 调用次数（每个单步）：tipPose 6 次、tipJacobian 1 次；
/// 启用臂角时再加 3 次 linkPose + 2 次 linkJacobian（另有纯算术的差分）。
/// 实现方应保证这些调用是**廉价且无分配**的（Rkd 适配层已做状态缓冲复用）。
class Kinematics {
 public:
  virtual ~Kinematics() = default;

  /// 被控自由度数 n。必须与 seed 长度、雅可比列数一致。
  virtual int dof() const = 0;

  /// 基座系下的末端位姿，位置单位 mm。用于算残差与回溯线搜索。
  virtual Pose tipPose(const std::vector<double>& q_deg) const = 0;

  /// 基座系下的末端雅可比 6 x n：0~2 行 mm/rad，3~5 行 rad/rad。
  ///
  /// 返回 false 或形状不对 → 本次求解按硬失败处理（不会静默返回旧解）。
  virtual bool tipJacobian(const std::vector<double>& q_deg,
                           Eigen::MatrixXd* jacobian) const = 0;

  /// 任意 frame（link 名）的位姿。**只有臂角需要**，默认不支持。
  ///
  /// 肩/肘/腕三个 frame 由 Options 指定；适配层应只对这三个名字返回 true，
  /// 其余返回 false（避免调用方误传导致每拍多做无用 FK）。
  virtual bool linkPose(const std::string& /*link*/,
                        const std::vector<double>& /*q_deg*/,
                        Pose* /*out*/) const {
    return false;
  }

  /// 任意 frame 的 6 x n 雅可比，单位同上。用于把臂角的梯度链到关节空间。
  virtual bool linkJacobian(const std::string& /*link*/,
                            const std::vector<double>& /*q_deg*/,
                            Eigen::MatrixXd* /*jacobian*/) const {
    return false;
  }
};

/// 臂角策略。
///
/// 7 轴冗余臂在给定末端位姿后仍有一个自运动自由度（解集构成一维流形），
/// 臂角 ψ 就是这条流形上的坐标：
///     ψ = 0   → 肘部朝 hint 方向
///     ψ ≠ 0   → 肘部相对参考平面偏转 ψ
/// 选哪个 ψ 决定了"肘部往哪边拐"，直接影响可操作度与是否撞到本体。
///
/// 三种策略：
///   kOff        —— 不参与求解。仍会计算并上报 ψ，便于只观察不干预。
///   kBestEffort —— 只在**零空间**里修正 ψ，且只使用位姿步之后剩余的速度预算
///                  （best_effort 占 35%）。位姿永远优先，适合遥操作。
///   kForced     —— 允许与位姿争用同一份预算（100%），位姿可为臂角让步。
///                  适合"先摆好肘部再动末端"的规划场景。
enum class ArmAngleMode { kOff, kBestEffort, kForced };

/// 求解器全部可调参数。
///
/// 默认值与工程 A `HC_ROBOT/tools/ik_bridge.cpp` 的默认值一致，
/// 保证"搬过来跑"和"原来跑"行为相同，便于对照。
struct Options {
  // -------------------------------------------------------------------------
  // 速度级 QP：决定"每拍能走多远、走多急"
  // -------------------------------------------------------------------------

  /// 每步消除的误差比例 g，归一化到 (0, 1]。
  ///
  /// 物理含义：`s ≈ g·J⁺e`，即这一拍想消掉残差的 g 倍。
  /// 闭环上是离散一阶低通，时间常数 τ ≈ Δt / g（g=0.5、Δt=10 ms → τ≈20 ms）。
  ///   · 调大 → 跟手更紧，但过大会过冲/抖动（尤其接近奇异时）；
  ///   · 调小 → 平顺、超调少，但跟踪滞后变大。
  /// 推荐 0.3 ~ 0.8；遥操作常用 0.5。
  double gain{0.5};

  /// 控制周期 Δt，单位 s。
  ///
  /// **必须等于真实控制周期**，因为速度上限是按 v_max·Δt 折算成位移盒的。
  /// 填大会让每拍允许的位移偏大（超速），填小会人为限速（跟不上）。
  /// MoveLine 的实时路径会在每次 tick 用 OptParams.dt_sec 覆盖它，无需手工维护。
  double dt_sec{0.01};

  /// 关节速度上限，单位 deg/s。
  ///
  /// 这是求解器内部**唯一**的硬限速来源（盒约束的 ±v_max·Δt 边界）。
  /// 建议与 `RtcConfig::max_velocity` 取值一致：
  ///   · 若这里更小 → 求解器先饱和，最终关节 RTC 永远用不满，表现为"慢"；
  ///   · 若这里更大 → 最终关节 RTC 才是实际限速层，求解器给的步长被二次削减，
  ///     两层滤波叠加会让响应更滞后。
  /// 默认 90，与工程 A 的 yaml 一致。
  double joint_v_max_deg_s{90.0};

  /// 最小二乘阻尼 λ（加到 A = JᵀJ 的对角线上，实际加的是 λ²）。
  ///
  /// 作用：奇异位形附近 J 的最小奇异值很小时，伪逆会给出极大步长；
  /// 阻尼把它压回 `s ≈ (JᵀJ + λ²I)⁻¹Jᵀe`，等价于截断那些"要转很多度
  /// 才换来一点点末端位移"的方向。
  ///   · 调大 → 更稳、不甩，但正常位形下也变迟钝，且留下稳态残差；
  ///   · 调小 → 精度好，奇异处可能突变。
  /// 推荐 1e-3 ~ 1e-2；默认 2e-3。
  double damping{0.002};

  // -------------------------------------------------------------------------
  // 误差归一化：决定"位置和姿态各算多少分"
  // -------------------------------------------------------------------------
  //
  // 进 QP 前残差被缩放为  ẽ = [ (√w_p / σ_p)·e_p ; (√w_o / σ_o)·e_o ]
  // 所以真正起作用的是 w/σ² 这个比值：σ 是"多少误差算一份代价"的尺度。
  // 想让某个任务更强势，调 w 或调小 σ 等效（但 σ 同时影响数值尺度，建议只调 w）。

  /// 位置任务相对权重（参与开方，实际强度 ∝ w_p/σ_p²）。
  double position_weight{1.0};

  /// 姿态任务相对权重。
  ///
  /// 目标不可达时的折中方向：调大它 → 宁可位置差一点也要保姿态；
  /// 调小它 → 宁可姿态歪一点也要贴近位置。遥操作一般保持 1:1。
  double orientation_weight{1.0};

  /// 位置误差归一化尺度 σ_p，单位 mm。
  ///
  /// "多少 mm 的残差算一份代价"。默认 50 mm —— 末端差 5 cm 记 1 分。
  /// 调小相当于放大位置任务；一般只在换机器人尺度时才改。
  double norm_position_mm{50.0};

  /// 姿态误差归一化尺度 σ_o，单位 rad。默认 0.2 rad ≈ 11.5°。
  double norm_orientation_rad{0.2};

  // -------------------------------------------------------------------------
  // 收敛判据：只影响"何时提前退出"与上报，不影响单步步长
  // -------------------------------------------------------------------------

  /// 位置收敛阈值，单位 mm（iterations=1 时仅用于填 Result 的 converged 标志）。
  double position_tolerance_mm{0.5};

  /// 姿态收敛阈值，单位 rad。
  double orientation_tolerance_rad{1e-3};

  // -------------------------------------------------------------------------
  // 关节限位
  // -------------------------------------------------------------------------

  /// 关节位置下限/上限，单位 deg，与 q 同序。
  ///
  /// 留空时走这个顺序：显式指定 → 从模型读（Rkd::jointGroupLimits，placo 限位）
  /// → 兜底 ±180°。**真机上不要依赖兜底**：限位错了会把手臂往硬限位上推，
  /// 虽然还有最终关节 RTC 和上层限位校验兜底，但那已经是"被拒 → 受控停止"了。
  std::vector<double> joint_lower_deg;
  std::vector<double> joint_upper_deg;

  // -------------------------------------------------------------------------
  // 臂角（7 轴冗余）
  // -------------------------------------------------------------------------

  /// 策略开关。注意：即使 kOff，只要三个 frame 都给全，仍会计算并上报 ψ。
  ArmAngleMode arm_angle_mode{ArmAngleMode::kOff};

  /// 肩 / 肘 / 腕三个 frame 名。任一为空则整条臂角链路关闭。
  ///
  /// 语义与旧库 ArmAngleTask 一致；典型取关节 1/4/7 所在的 link
  /// （如 openarmx_left_link1 / link4 / link7）。必须在模型里存在。
  std::string arm_angle_shoulder_frame;
  std::string arm_angle_elbow_frame;
  std::string arm_angle_wrist_frame;

  /// 期望肘部方向，表达在 **ik_base_link（臂基座）坐标系**，内部会归一化。
  ///
  /// ψ 的零点定义就是"肘部投影方向与这个向量一致"。
  /// 注意旧库 PlacoIkOptions 的 hint 有多种 frame 约定（默认 world），
  /// 这里固定按基座系解释，跨机器人移植时不要直接照抄 world 系数值。
  Eigen::Vector3d arm_angle_elbow_hint_axis{Eigen::Vector3d::UnitZ()};

  /// 期望臂角目标 ψ_d，单位 rad。0 表示"肘部朝 hint 方向"。
  double arm_angle_target_rad{0.0};

  /// 臂角每步收敛比例，0~1。调大更快但可能占用过多零空间预算。
  double arm_angle_gain{0.5};

  /// 臂角收敛容差，单位 rad。默认 5e-3 ≈ 0.29°。
  double arm_angle_tolerance_rad{5e-3};

  // -------------------------------------------------------------------------
  // 迭代控制
  // -------------------------------------------------------------------------

  /// 单次调用内最多走几步。
  ///
  ///   1    → 实时伺服（遥操作、ServoP/MoveL 每拍一次）。**默认**。
  ///   50   → 快速近似收敛，适合"看一下最近可达点在哪"。
  ///   200  → 实测到残差地板（≈2.6 ms @真实 SDK），适合路点/离线。
  /// 注意：规划类调用（MoveP / planLine）必须是 >1 的收敛语义，
  /// 那里若用 1 会得到"只走一步"的关节轨迹。
  ///
  /// ⚠ **实时场景请保持 1**。每个 solveOnce 都按 v_max·Δt 单独做盒约束，
  /// 一次调用内走 N 步，累计位移最大可达 N·v_max·Δt —— 也就是把每拍速度上限
  /// 放大了 N 倍（实测 iterations=2/3 时 80 mm/s 跟踪残差从 0.80 mm 降到
  /// 0.27 mm，正是靠突破这个上限换来的）。要更快响应请调 gain，不要加步数。
  int iterations{1};

  /// 盒约束 QP（坐标下降）的最大扫描次数。
  ///
  /// 每轮扫描 n 次一维精确极小；凸问题下目标函数单调不增。
  /// 内部在 max|Δx| < 1e-12 时提前退出，所以给大一点通常不额外花时间。
  /// 一般无需修改；若发现某轴长期贴边且收敛慢，可试着调到 100~200。
  int coordinate_descent_sweeps{60};

  /// 三个 frame 都给全且策略非 kOff 时才真正启用。
  bool armAngleEnabled() const {
    return arm_angle_mode != ArmAngleMode::kOff &&
           !arm_angle_shoulder_frame.empty() &&
           !arm_angle_elbow_frame.empty() &&
           !arm_angle_wrist_frame.empty();
  }
  /// true 表示臂角可以与位姿争用速度预算（kForced）。
  bool armAngleGuided() const { return arm_angle_mode == ArmAngleMode::kForced; }
};

/// 求解结果。
///
/// 注意 `ok` 的含义是"这次调用正常完成了"，**不是**"已到达目标"。
/// 实时伺服语义下（iterations=1）绝大多数调用都是 ok=true 但 converged=false，
/// 这是设计如此：它只负责走一步。
struct Result {
  /// 求解流程是否正常完成。false 表示输入不合法 / 雅可比不可用 /
  /// 步长非有限等硬失败，此时 q_deg 不可用。
  bool ok{false};

  /// 人类可读的状态，取值固定为以下几类（便于上层做字符串判断或日志分级）：
  ///   "target reached"
  ///       已满足位置与姿态容差（iterations=1 时较少见）。
  ///   "servo step (not converged in one call)"
  ///       正常单步：走了一步但还没到。实时伺服下应视为成功。
  ///   "no descent direction (joint limits or unreachable)"
  ///       回溯线搜索发现四个候选步长都不降低位姿代价，已整步归零。
  ///       说明被关节限位卡死，或目标在当前构型下无下降方向。
  ///   "zero step (blocked by joint limits)"
  ///       QP 解本身就是零步。
  ///   其余为硬失败信息（如 "tipJacobian unavailable"）。
  std::string message;

  /// 解，单位 deg，与输入 seed 同序。iterations=1 时是"本拍应到的位置"。
  std::vector<double> q_deg;

  /// 用结果构型做 FK 复核得到的**实际**残差（不是线性化预测值）。
  /// 单位 mm。不可达时会收敛到一个地板（最近可达点），不会无限下降。
  double position_error_mm{0.0};
  /// 实际姿态残差，单位 rad（旋转向量模长）。
  double orientation_error_rad{0.0};

  /// 是否已满足 position_tolerance_mm / orientation_tolerance_rad。
  bool converged_position{false};
  bool converged_orientation{false};

  /// 臂角的有效性（手臂完全伸直或 hint 与肩腕轴平行时 ψ 奇异 → false）。
  bool arm_angle_valid{false};
  /// 结果构型下的臂角，单位 rad。
  double arm_angle_rad{0.0};

  // ---- 诊断计数：用于估算真实 SDK 下的总耗时 ----
  // 真实单拍耗时 ≈ 求解器自身 + fk_calls×T_fk + jacobian_calls×T_jac，
  // 其中 Rkd 的 T_fk/T_jac 实测约 1.51 µs / 2.34 µs。不启用臂角时固定 6/1。
  int iterations_run{0};
  int fk_calls{0};
  int jacobian_calls{0};
};

namespace detail {

inline Eigen::Vector3d rotationError(const Eigen::Matrix3d& target,
                                     const Eigen::Matrix3d& current) {
  const Eigen::AngleAxisd delta(target * current.transpose());
  return delta.axis() * delta.angle();
}

/// 6x7 矩阵的精确零向量（六个列向量的广义叉积）。
inline Eigen::VectorXd nullVector6x7(const Eigen::MatrixXd& matrix) {
  Eigen::VectorXd null_vector(7);
  for (int column = 0; column < 7; ++column) {
    Eigen::Matrix<double, 6, 6> minor;
    int target = 0;
    for (int j = 0; j < 7; ++j) {
      if (j == column) {
        continue;
      }
      minor.col(target++) = matrix.col(j);
    }
    null_vector(column) = ((column % 2) ? -1.0 : 1.0) * minor.determinant();
  }
  return null_vector;
}

/// 一般 n>6 情况：取最小右奇异向量作为零空间方向。
inline Eigen::VectorXd nullVectorGeneral(const Eigen::MatrixXd& matrix) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix,
                                              Eigen::ComputeFullV);
  return svd.matrixV().col(matrix.cols() - 1);
}

/// psi：由肩/肘/腕位置与参考方向决定。ok=false 表示几何退化（手臂伸直等）。
inline double armAngleFromPositions(const Eigen::Vector3d& shoulder,
                                    const Eigen::Vector3d& elbow,
                                    const Eigen::Vector3d& wrist,
                                    const Eigen::Vector3d& hint, bool* ok) {
  *ok = false;
  const Eigen::Vector3d sw = wrist - shoulder;
  const double length = sw.norm();
  if (length < 1e-6) {
    return 0.0;
  }
  const Eigen::Vector3d u = sw / length;

  Eigen::Vector3d h = hint - u * hint.dot(u);
  if (h.norm() < 1e-9) {
    return 0.0;  // hint 与肩腕轴平行：无参考平面
  }
  h.normalize();

  Eigen::Vector3d r = elbow - shoulder;
  r -= u * r.dot(u);
  if (r.norm() < 1e-6) {
    return 0.0;  // 手臂完全伸直：psi 奇异
  }
  r.normalize();

  *ok = true;
  return std::atan2(u.dot(r.cross(h)), r.dot(h));
}

struct ArmAngleState {
  bool valid{false};
  double angle{0.0};
  Eigen::VectorXd gradient;
};

/// 臂角及其对关节角的梯度（位置中心差分 + 肘/腕雅可比链式）。
inline ArmAngleState computeArmAngleNamed(
    const Kinematics& model, const std::vector<double>& q_deg,
    const std::string& elbow_frame, const std::string& wrist_frame,
    const Eigen::Vector3d& shoulder, const Eigen::Vector3d& hint, int n,
    int* fk_calls, int* jacobian_calls) {
  ArmAngleState state;
  state.gradient = Eigen::VectorXd::Zero(n);

  Pose elbow_pose;
  Pose wrist_pose;
  *fk_calls += 2;
  if (!model.linkPose(elbow_frame, q_deg, &elbow_pose) ||
      !model.linkPose(wrist_frame, q_deg, &wrist_pose)) {
    return state;
  }
  const Eigen::Vector3d elbow = elbow_pose.position_mm;
  const Eigen::Vector3d wrist = wrist_pose.position_mm;

  bool ok = false;
  const double psi =
      armAngleFromPositions(shoulder, elbow, wrist, hint, &ok);
  if (!ok) {
    return state;
  }

  // dpsi/dE、dpsi/dW 用位置中心差分（纯算术，无额外运动学）。
  const double delta = 1e-3;  // mm
  Eigen::RowVector3d dpsi_dE = Eigen::RowVector3d::Zero();
  Eigen::RowVector3d dpsi_dW = Eigen::RowVector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d step = Eigen::Vector3d::Zero();
    step[axis] = delta;
    bool ok_p = false;
    bool ok_m = false;
    const double e_p =
        armAngleFromPositions(shoulder, elbow + step, wrist, hint, &ok_p);
    const double e_m =
        armAngleFromPositions(shoulder, elbow - step, wrist, hint, &ok_m);
    if (ok_p && ok_m) {
      dpsi_dE[axis] = (e_p - e_m) / (2.0 * delta);
    }
    const double w_p =
        armAngleFromPositions(shoulder, elbow, wrist + step, hint, &ok_p);
    const double w_m =
        armAngleFromPositions(shoulder, elbow, wrist - step, hint, &ok_m);
    if (ok_p && ok_m) {
      dpsi_dW[axis] = (w_p - w_m) / (2.0 * delta);
    }
  }

  Eigen::MatrixXd jacobian_elbow;
  Eigen::MatrixXd jacobian_wrist;
  *jacobian_calls += 2;
  if (!model.linkJacobian(elbow_frame, q_deg, &jacobian_elbow) ||
      !model.linkJacobian(wrist_frame, q_deg, &jacobian_wrist) ||
      jacobian_elbow.rows() != 6 || jacobian_wrist.rows() != 6) {
    return state;
  }

  state.gradient =
      (dpsi_dE * jacobian_elbow.topRows(3) +
       dpsi_dW * jacobian_wrist.topRows(3))
          .transpose();
  state.angle = psi;
  state.valid = true;
  return state;
}

}  // namespace detail

namespace detail {

struct StepOutcome {
  bool ok{false};
  std::string message;
  std::vector<double> q_deg;
  double position_error_mm{0.0};
  double orientation_error_rad{0.0};
  bool converged_position{false};
  bool converged_orientation{false};
  bool arm_angle_valid{false};
  double arm_angle_rad{0.0};
  int fk_calls{0};
  int jacobian_calls{0};
};

/// 单步求解（与 ik_bridge SolveQpServoSingle 一致）。
///
/// 实现顺序（读代码时按这 8 段对照即可）：
///   ① 限位归一      把 seed 夹进 [lower, upper]，并把限位转成 rad 供后续使用
///   ② 臂角状态      仅在启用时计算 ψ 及其对 q 的梯度（含 3 次 linkPose）
///   ③ 残差          位置残差 e_p 与姿态残差 e_o = log(R_d·R(q)ᵀ)∨
///   ④ 收敛短路      位姿+臂角都达标就直接返回（不浪费一次雅可比）
///   ⑤ 加权最小二乘  把残差按 (√w/σ) 缩放，组成 A = JᵀJ + λ²I、b = Jᵀẽ
///   ⑥ 盒约束 QP     无约束解 → 坐标下降逐维精确极小 → 与"夹取解"比代价取优
///   ⑦ 单调性守卫    用真实 FK 回溯 α ∈ {1, 1/2, 1/4, 1/8}，不接受让代价变大的步
///   ⑧ 臂角修正      沿零空间方向叠加 ψ 修正，之后再用 FK 复核残差
///
/// 两个关键设计点（改动前务必看懂）：
///   · 第 ⑤ 步**不除以 Δt**。除以 Δt 会把每步都顶到 v_max 上，表现为持续满速
///     并振荡（实测行程暴涨）。gain 本身就是"每步消除多少误差"的比例。
///   · 第 ⑥ 步用**坐标下降**而不是"无约束解沿射线等比缩放到盒内"。后者只要
///     有一个关节顶在限位上且还想往外走，缩放系数就整体归零，整步变 0，
///     手臂彻底冻住（实测出界后滚手腕，姿态误差停在 29° 不再下降）。
inline StepOutcome solveOnce(const Kinematics& model,
                             const std::vector<double>& seed_deg,
                             const Pose& target, const Options& options) {
  StepOutcome out;
  const int n = model.dof();
  if (n <= 0 || static_cast<int>(seed_deg.size()) != n) {
    out.message = "seed size does not match model dof";
    return out;
  }

  // ---- 限位 ----
  std::vector<double> lower(n, -1.0e9), upper(n, 1.0e9);
  for (int i = 0; i < n; ++i) {
    if (static_cast<int>(options.joint_lower_deg.size()) > i) {
      lower[i] = options.joint_lower_deg[i];
    }
    if (static_cast<int>(options.joint_upper_deg.size()) > i) {
      upper[i] = options.joint_upper_deg[i];
    }
  }
  out.q_deg = seed_deg;
  for (int i = 0; i < n; ++i) {
    out.q_deg[i] = std::min(std::max(out.q_deg[i], lower[i]), upper[i]);
  }

  const Eigen::Vector3d target_position = target.position_mm;
  const Eigen::Matrix3d target_rotation = target.rotation;
  const double dt = std::max(1e-4, options.dt_sec);
  const double gain = std::max(1e-6, std::min(1.0, options.gain));
  const double v_max =
      std::max(1e-6, options.joint_v_max_deg_s) * M_PI / 180.0;
  const double lambda = std::max(1e-9, options.damping);
  const double sp = std::sqrt(std::max(1e-9, options.position_weight)) /
                    std::max(1e-9, options.norm_position_mm);
  const double so = std::sqrt(std::max(1e-9, options.orientation_weight)) /
                    std::max(1e-9, options.norm_orientation_rad);
  constexpr double kRadToDeg = 180.0 / M_PI;

  // ---- 臂角状态 ----
  detail::ArmAngleState angle_state;
  Eigen::Vector3d arm_hint = Eigen::Vector3d::Zero();
  Eigen::Vector3d arm_shoulder = Eigen::Vector3d::Zero();
  bool have_arm_angle = false;
  if (options.armAngleEnabled()) {
    arm_hint = options.arm_angle_elbow_hint_axis;
    if (arm_hint.allFinite() && arm_hint.norm() > 1e-9) {
      arm_hint.normalize();
      Pose shoulder_pose;
      ++out.fk_calls;
      if (model.linkPose(options.arm_angle_shoulder_frame, out.q_deg,
                         &shoulder_pose)) {
        arm_shoulder = shoulder_pose.position_mm;
        angle_state = detail::computeArmAngleNamed(
            model, out.q_deg, options.arm_angle_elbow_frame,
            options.arm_angle_wrist_frame, arm_shoulder, arm_hint, n,
            &out.fk_calls, &out.jacobian_calls);
        out.arm_angle_valid = angle_state.valid;
        out.arm_angle_rad = angle_state.angle;
        have_arm_angle = angle_state.valid &&
                         angle_state.gradient.size() == n;
      }
    }
  }

  // ---- 当前误差 ----
  ++out.fk_calls;
  const Pose pose = model.tipPose(out.q_deg);
  const Eigen::Vector3d e_p = target_position - pose.position_mm;
  const Eigen::Vector3d e_o = detail::rotationError(target_rotation, pose.rotation);

  out.ok = true;
  out.position_error_mm = e_p.norm();
  out.orientation_error_rad = e_o.norm();
  out.converged_position =
      out.position_error_mm <= options.position_tolerance_mm;
  out.converged_orientation =
      out.orientation_error_rad <= options.orientation_tolerance_rad;

  const double arm_angle_error =
      angle_state.valid ? angle_state.angle - options.arm_angle_target_rad : 0.0;
  const bool arm_angle_done =
      !(have_arm_angle && options.armAngleEnabled()) || !angle_state.valid ||
      std::abs(arm_angle_error) <= options.arm_angle_tolerance_rad;
  if (out.converged_position && out.converged_orientation && arm_angle_done) {
    if (out.message.empty()) {
      out.message = "target reached";
    }
    return out;
  }

  // ---- 雅可比 ----
  Eigen::MatrixXd jacobian;
  ++out.jacobian_calls;
  if (!model.tipJacobian(out.q_deg, &jacobian) || jacobian.rows() != 6 ||
      jacobian.cols() != n) {
    out.message = "tipJacobian unavailable";
    out.ok = false;
    return out;
  }

  Eigen::MatrixXd Jw(6, n);
  Jw.topRows(3) = jacobian.topRows(3) * sp;
  Jw.bottomRows(3) = jacobian.bottomRows(3) * so;
  // Jw·s ≈ gain·ew（s 为本步关节位移）。不再除以 dt：gain 即「每步消除多少误差」。
  Eigen::VectorXd desired(6);
  desired.head(3) = e_p * sp * gain;
  desired.tail(3) = e_o * so * gain;

  // ---- 每关节位移区间：位置限位 ∩ 速度上限 ----
  Eigen::VectorXd step_lower(n), step_upper(n);
  for (int i = 0; i < n; ++i) {
    const double by_pos_lo = (lower[i] - out.q_deg[i]) * M_PI / 180.0;
    const double by_pos_hi = (upper[i] - out.q_deg[i]) * M_PI / 180.0;
    const double by_v = v_max * dt;
    step_lower[i] = std::max(by_pos_lo, -by_v);
    step_upper[i] = std::min(by_pos_hi, by_v);
    if (step_lower[i] > step_upper[i]) {  // 已顶在限位
      step_lower[i] = 0.0;
      step_upper[i] = 0.0;
    }
  }

  // ---- 盒约束加权最小二乘 ----
  Eigen::MatrixXd A = Jw.transpose() * Jw;
  A.diagonal().array() += lambda * lambda;
  const Eigen::VectorXd rhs = Jw.transpose() * desired;

  const Eigen::VectorXd x_unc = A.ldlt().solve(rhs);
  if (!x_unc.allFinite()) {
    out.message = "non-finite joint step";
    out.ok = false;
    return out;
  }

  // 逐坐标精确极小化（坐标下降），凸 QP ⇒ 全局最优；且目标函数单调不增。
  Eigen::VectorXd x_qp = x_unc;
  for (int i = 0; i < n; ++i) {
    x_qp[i] = std::min(std::max(x_qp[i], step_lower[i]), step_upper[i]);
  }
  {
    const Eigen::VectorXd diag = A.diagonal();
    for (int sweep = 0; sweep < options.coordinate_descent_sweeps; ++sweep) {
      double max_change = 0.0;
      for (int i = 0; i < n; ++i) {
        if (diag[i] <= 0.0) {
          continue;
        }
        const double partial = A.row(i).dot(x_qp) - diag[i] * x_qp[i];
        double cand = (rhs[i] - partial) / diag[i];
        cand = std::min(std::max(cand, step_lower[i]), step_upper[i]);
        max_change = std::max(max_change, std::abs(cand - x_qp[i]));
        x_qp[i] = cand;
      }
      if (max_change < 1e-12) {
        break;
      }
    }
  }
  const auto quadratic_cost = [&](const Eigen::VectorXd& x) {
    return 0.5 * x.dot(A * x) - rhs.dot(x);
  };
  Eigen::VectorXd step = x_qp;
  {
    Eigen::VectorXd x_clip = x_unc;
    for (int i = 0; i < n; ++i) {
      x_clip[i] = std::min(std::max(x_clip[i], step_lower[i]), step_upper[i]);
    }
    if (quadratic_cost(x_clip) < quadratic_cost(step)) {
      step = x_clip;
    }
  }

  // ---- 单调性守卫：只接受真正降低归一化位姿误差的步长 ----
  const auto weighted_cost = [&](const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& o) {
    return (p * sp).squaredNorm() + (o * so).squaredNorm();
  };
  const double cost_hold = weighted_cost(e_p, e_o);
  double best_cost = cost_hold;
  double best_alpha = 0.0;
  const std::vector<double> seed_q = out.q_deg;  // 此时仍是 seed
  for (const double alpha : {1.0, 0.5, 0.25, 0.125}) {
    std::vector<double> trial = seed_q;
    for (int i = 0; i < n; ++i) {
      const double next = seed_q[i] + alpha * step[i] * kRadToDeg;
      trial[i] = std::min(std::max(next, lower[i]), upper[i]);
    }
    ++out.fk_calls;
    const Pose fk = model.tipPose(trial);
    const double cost = weighted_cost(
        target_position - fk.position_mm,
        detail::rotationError(target_rotation, fk.rotation));
    if (cost < best_cost - 1e-12) {
      best_cost = cost;
      best_alpha = alpha;
    }
  }
  step *= best_alpha;  // 被拒绝时整步归零

  // ---- 臂角：沿零空间修正，占用位姿步之后剩余的速度预算 ----
  if (have_arm_angle && options.armAngleEnabled()) {
    Eigen::VectorXd null_vector;
    if (n == 7) {
      Eigen::MatrixXd task(6, n);
      task.topRows(3) = jacobian.topRows(3);
      task.bottomRows(3) = jacobian.bottomRows(3);
      null_vector = detail::nullVector6x7(task);
    } else if (n > 6) {
      Eigen::MatrixXd task(6, n);
      task.topRows(3) = jacobian.topRows(3);
      task.bottomRows(3) = jacobian.bottomRows(3);
      null_vector = detail::nullVectorGeneral(task);
    }
    const double null_norm = null_vector.size() == n ? null_vector.norm() : 0.0;
    if (null_norm > 1e-12) {
      null_vector /= null_norm;
      const double coupling = null_vector.dot(angle_state.gradient);
      if (std::abs(coupling) > 1e-9) {
        Eigen::VectorXd correction =
            null_vector *
            (-options.arm_angle_gain * arm_angle_error / coupling);
        const bool guided = options.armAngleGuided();
        const double budget = (guided ? 1.0 : 0.35) * v_max * dt;
        const double peak = correction.cwiseAbs().maxCoeff();
        if (peak > budget && peak > 1e-12) {
          correction *= budget / peak;
        }
        if (correction.allFinite()) {
          for (int i = 0; i < n; ++i) {
            const double lo =
                guided ? step_lower[i] : step_lower[i] - step[i];
            const double hi =
                guided ? step_upper[i] : step_upper[i] - step[i];
            step[i] += std::min(std::max(correction[i], lo), hi);
            step[i] = std::min(std::max(step[i], step_lower[i]), step_upper[i]);
          }
        }
      }
    }
  }

  if (step.norm() < 1e-9 && out.message.empty()) {
    out.message = "zero step (blocked by joint limits)";
  }

  // ---- 应用并复核 ----
  out.q_deg = seed_q;
  for (int i = 0; i < n; ++i) {
    const double next = seed_q[i] + step[i] * kRadToDeg;
    out.q_deg[i] = std::min(std::max(next, lower[i]), upper[i]);
  }

  ++out.fk_calls;
  const Pose after = model.tipPose(out.q_deg);
  out.position_error_mm = (target_position - after.position_mm).norm();
  out.orientation_error_rad =
      detail::rotationError(target_rotation, after.rotation).norm();
  out.converged_position =
      out.position_error_mm <= options.position_tolerance_mm;
  out.converged_orientation =
      out.orientation_error_rad <= options.orientation_tolerance_rad;
  if (out.message.empty() && !out.converged_position) {
    out.message = (best_alpha == 0.0)
                      ? "no descent direction (joint limits or unreachable)"
                      : "servo step (not converged in one call)";
  }
  if (have_arm_angle) {
    detail::ArmAngleState final_angle = detail::computeArmAngleNamed(
        model, out.q_deg, options.arm_angle_elbow_frame,
        options.arm_angle_wrist_frame, arm_shoulder, arm_hint, n, &out.fk_calls,
        &out.jacobian_calls);
    out.arm_angle_valid = final_angle.valid;
    if (final_angle.valid) {
      out.arm_angle_rad = final_angle.angle;
    }
  }
  return out;
}

}  // namespace detail

/// 求解入口。
///
/// 两种语义由 options.iterations 决定，**不是**两个不同的算法：
///
/// 【实时伺服】iterations == 1
///   只调用一次 solveOnce，返回"本拍该走到哪"。调用方每拍调一次，
///   跨拍自然收敛（离散一阶 lag，τ≈Δt/g）。特点：
///     · 耗时为常数、与目标是否可达无关（实测约 13 µs @真实 SDK）；
///     · 目标不可达时不会失败，而是给出"朝目标方向、且不超速不撞限位"的一步。
///   这是遥操作（ServoP/MoveL）该用的模式。
///
/// 【收敛求解】iterations == N > 1
///   反复走步直到位姿满足容差或用尽预算，返回最终近似解。
///     · 可达目标：一般 10~20 步收敛到毫米级（受 gain 与容差影响）；
///     · 不可达目标：残差单调下降后停在"最近可达点"的地板，实测 200 步到底。
///   这是路点/离线/离线标定该用的模式；不要在规划路径里用 1。
///
/// 两个模式共用同一条求解路径与同一套限位约束，所以从这里切模式不会改变
/// "每一步是否合法"的判定，只改变走几步。
inline Result solve(const Kinematics& model,
                    const std::vector<double>& seed_deg, const Pose& target,
                    const Options& options) {
  Result result;
  std::vector<double> q = seed_deg;
  detail::StepOutcome step;
  const int budget = std::max(1, options.iterations);
  for (int k = 0; k < budget; ++k) {
    step = detail::solveOnce(model, q, target, options);
    ++result.iterations_run;
    result.fk_calls += step.fk_calls;
    result.jacobian_calls += step.jacobian_calls;
    if (!step.ok) {
      result.ok = false;
      result.message = step.message;
      result.q_deg = step.q_deg;
      return result;
    }
    q = step.q_deg;
    if (step.converged_position && step.converged_orientation) {
      break;
    }
  }
  result.ok = true;
  result.message = step.message;
  result.q_deg = std::move(q);
  result.position_error_mm = step.position_error_mm;
  result.orientation_error_rad = step.orientation_error_rad;
  result.converged_position = step.converged_position;
  result.converged_orientation = step.converged_orientation;
  result.arm_angle_valid = step.arm_angle_valid;
  result.arm_angle_rad = step.arm_angle_rad;
  return result;
}

}  // namespace qp_servo
