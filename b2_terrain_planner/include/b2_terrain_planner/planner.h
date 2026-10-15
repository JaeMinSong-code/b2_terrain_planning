/**
 * @file planner.h
 * @brief 평지·지형 trot의 SQP 입력·출력 및 시간 구간 계획 계약.
 * @version 0.0.21
 * @update 2026-10-15: 제약 해제 실험과 상세 탐색 추적 전용 계약을 제거함.
 * @update 2026-10-14: 폐기한 추가 접촉력 검증기의 결과·시간 계약을 제거함.
 */
#ifndef B2_TERRAIN_PLANNER_PLANNER_H_
#define B2_TERRAIN_PLANNER_PLANNER_H_

#include "b2_terrain_planner/terrain.h"

#include <array>
#include <optional>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace b2_terrain_planner {
class RobotModel;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Feet = Eigen::Matrix<double, 4, 3, Eigen::RowMajor>;

struct Robot {
  double mass = 74.0;
  Eigen::Matrix3d inertia = Eigen::Vector3d(1.5, 4.0, 4.0).asDiagonal();
  Feet hips;
  Feet nominal_offsets;
  Feet reference_legs = Feet::Zero();
  Eigen::Vector4d reach_min = Eigen::Vector4d::Zero();
  Eigen::Vector4d reach_max = Eigen::Vector4d::Zero();
  bool use_reference_legs = false;
  Eigen::Vector3d com_offset = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 12, 1> reference_joints = Eigen::Matrix<double, 12, 1>::Zero();
  std::shared_ptr<const RobotModel> model;
  double height = 0.55;
  double friction = 0.5;
  double leg_min = 0.25;
  double leg_max = 0.75;
  double period = 0.8;
  double duty = 0.6;
  Robot();
};

struct Input {
  Vector6 position = Vector6::Zero();  // CoM XYZ, yaw/pitch/roll (ZYX).
  Vector6 velocity = Vector6::Zero();  // XYZ derivative and Euler derivative.
  Feet feet = Feet::Zero();
  Eigen::Vector3d command = Eigen::Vector3d::Zero();  // Body vx/vy, yaw rate.
  double gait_time = 0.0;
  bool has_nominal = false;
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> nominal =
      Eigen::Matrix<double, 4, 2, Eigen::RowMajor>::Zero();
  std::array<bool, 4> fixed{false, false, false, false};
  std::string frame = "odom";
  std::optional<HeightMap> terrain;  // 없으면 기존 평지 계산 경로를 사용한다.
};

struct Phase {
  double start = 0.0;
  double duration = 0.0;
  std::array<bool, 4> contacts{};
  std::array<bool, 4> landed{};
};

enum class QpReuseMode { kNone, kWorkspace, kWarmStart };
std::string QpReuseModeName(QpReuseMode mode);

// 기본값은 기존 cold solve를 유지한다. 재사용 수명은 한 Solve 안의 SQP 반복이다.
struct Settings {
  QpReuseMode qp_reuse = QpReuseMode::kNone;
  bool full_evaluation = false;
  bool qp_working_set = false;
  bool structured_equalities = false;
  bool block_assembly = false;
  bool polynomial_tail = false;
  bool projection_l2 = false;
  bool chord_projection = false;
  double equality_rank_tolerance = 1e-6;
  double projection_rank_tolerance = 1e-9;
  double projection_residual_tolerance = 1e-9;
  bool lapack_svd = false;  // 동일한 절단 기준의 LAPACK 비교 경로.
  int max_iterations = 60;
  int realtime_iterations = 0;  // 0: 기존 수렴. 양수: feasible 개선 후 다음 호출에서 계속.
  double orientation_limit = 0.35;  // pitch/roll 절댓값 상한 [rad], 0 < limit < pi/2.
  double pitch_weight = 30.0;  // 수평 pitch 선호 비용. 각도 제한과 별개다.
  double leg_vertical_weight = 100.0;  // 기준 다리 벡터의 월드 Z 오차 추가 비용.
  double vz_weight = 0.0;  // 월드 수직속도 제곱 비용(0=비활성). 지형-인지 기준 도입 전까지 꺼 둔다.
  double foothold_backward_weight = 1.4;  // nominal 뒤쪽 거리 제곱의 추가 비용, 0이면 비활성.
  double qp_regularization = 0.01;
  double qp_eps_abs = 1e-9;
  double qp_eps_rel = 1e-9;
  int qp_max_iterations = 100000;
  int sample_intervals = 12;
  double feasibility_tolerance = 1e-5;
  double step_tolerance = 1e-5;
  TerrainSettings terrain;
  double time_limit = 20.0;  // Offline baseline; not a real-time guarantee.
};

// 잔차는 변환된 OSQP 문제의 값이며 비선형/물리 제약 오차가 아니다.
// optional 값은 해당 단계가 실행되지 않았으면 미측정이다.
struct QpDiagnostic {
  int working_set_pass = 0;
  int working_set_total_rows = 0;
  std::optional<double> working_set_omitted_violation;
  std::optional<double> working_set_full_violation;
  bool workspace_reused = false;
  std::string workspace_reason;
  std::optional<int> update_return;
  std::optional<double> update_seconds;
  std::optional<int> warm_start_return;
  std::optional<double> warm_start_seconds;
  bool warm_start_used = false;
  std::string warm_start_reason;
  int reused_duals = 0;
  int variables = 0;
  int rows = 0;
  std::optional<double> warm_start_roundtrip_error;
  std::optional<double> warm_start_projection_error;
  int sqp_iteration = 0;  // 1-based.
  std::string kind;
  std::optional<int> setup_return;
  std::optional<int> solve_return;
  std::optional<int> status_code;
  std::string status;
  std::optional<int> iterations;
  std::optional<double> primal_residual;
  std::optional<double> dual_residual;
  std::optional<int> polish_status;
  std::optional<int> rho_updates;
  std::optional<double> setup_seconds;
  std::optional<double> solve_seconds;
  bool accepted = false;  // SQP 방향 후보로 채택; line search 채택과 별개다.
  std::string acceptance_reason;
};

struct ConstraintDiagnostic {
  std::string name;
  int rows = 0;
  double max_violation = 0.0;  // 제약 그룹별 비선형 잔차의 최대 위반.
  int phase = -1;
  int sample = -1;
  int row = -1;
};

struct Profile {
  double initialization_seconds = 0.0;
  double ik_seconds = 0.0;
  // 포함 시간: 최종 평가·IK를 포함한다.
  double final_validation_seconds = 0.0;
  // 아래 10개 시간은 서로 겹치지 않는다. 모든 시간 단위는 초다.
  double full_evaluation_seconds = 0.0;
  double values_evaluation_seconds = 0.0;
  double constraint_jacobian_seconds = 0.0;
  double svd_seconds = 0.0;
  double qp_prepare_seconds = 0.0;
  double qp_setup_seconds = 0.0;
  double qp_solve_seconds = 0.0;
  double qp_cleanup_seconds = 0.0;
  double qp_update_seconds = 0.0;
  double qp_warm_start_seconds = 0.0;
  // 포함 시간: 하위 평가/SVD/QP 시간을 더해서 합산하면 안 된다.
  double projection_seconds = 0.0;
  double line_search_seconds = 0.0;
  int full_evaluations = 0;
  int values_evaluations = 0;
  int constraint_jacobian_evaluations = 0;
  int svd_calls = 0;
  int qp_calls = 0;
  int qp_setups = 0;
  int qp_updates = 0;
  int qp_warm_starts = 0;
  long long qp_iterations = 0;
  std::vector<int> qp_iterations_per_solve;
  std::vector<QpDiagnostic> qp_diagnostics;
};

struct TerrainValidation {
  bool passed = false;
  double max_contact_error = 0.0;
  double min_foot_distance = 0.0;  // 측정값이며 성공 판정에 사용하지 않는다.
  std::string reason;
};

// 동일한 모델 스냅샷·정적 지형의 재계획에 사용한다. 실패한 계획은 재사용하지 않는다.
struct SeedContext {
  double gait_time = 0.0;
  double period = 0.0;
  double duty = 0.0;
  std::string frame;
  std::string robot_signature;
  bool terrain = false;
  Feet initial_feet = Feet::Zero();
  std::string terrain_signature;
};

struct WarmStartDiagnostic {
  bool requested = false;
  bool used = false;
  std::string reason = "not_requested";
  double elapsed = 0.0;
  int shifted_phases = 0;
  int extended_phases = 0;
  int polynomial_extended_phases = 0;
  int reused_footholds = 0;
  int reach_reselected_footholds = 0;
  int reach_candidate_fallbacks = 0;
};

struct TerrainAttempt {
  int leg = -1;  // -1은 기본 초기화, 나머지는 다른 지지면으로 바꾼 다리.
  Eigen::Vector3d seed = Eigen::Vector3d::Zero();
  std::string status;
  double elapsed = 0;
  bool success = false;
};

struct Result {
  SeedContext seed_context;
  WarmStartDiagnostic warm_start;
  std::vector<TerrainAttempt> terrain_attempts;
  bool success = false;
  int variable_count = 0;
  int margin_variable_count = 0;
  int constraint_count = 0;
  std::vector<ConstraintDiagnostic> constraint_groups;
  std::string status;
  int iterations = 0;
  double elapsed = 0.0;
  double cost = 0.0;
  double violation = 0.0;
  Profile profile;
  std::optional<TerrainValidation> terrain_validation;
  std::optional<bool> kinematics_valid;
  int ik_checks = 0;
  double max_ik_error = 0;
  std::optional<int> ik_failed_leg;
  std::optional<double> ik_failed_time;
  Feet footholds = Feet::Zero();
  std::vector<Phase> phases;
  std::vector<Eigen::Matrix<double, 6, 5>> coefficients;  // Physical seconds.
  std::vector<double> margins;  // 구간별 GIAC margin, 해당 변수가 없으면 0.
};

std::vector<Phase> MakeHorizon(double time, double period, double duty);
Result Solve(const Robot& robot, const Input& input, const Settings& settings = {},
             const Result* previous = nullptr);
// 이전 계획과 현재 입력의 모델·지형 스냅샷 호환성을 확인하는 식별값.
std::string RobotSignature(const Robot& robot);
std::string TerrainSignature(const std::optional<HeightMap>& terrain);
Vector6 Evaluate(const Result& result, double elapsed, int derivative = 0);
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_PLANNER_H_
