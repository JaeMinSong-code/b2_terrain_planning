/**
 * @file swing_terrain_test.cpp
 * @brief 스윙·지지발의 높이 불일치 허용과 다음 착지점의 지형 검사를 검증한다.
 * @version 0.0.10
 * @update 2026-11-09: 예산 종료의 실행 가능 반복해 반환과 불능 입력 조기 종료를 검증함.
 * @update 2026-11-05: 시간 제한 0을 허용하고 음수 설정 거부와 기존 유한 예산 진단을 검증함.
 */
#include "b2_terrain_planner/planner.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace b2_terrain_planner;

void Check(bool passed, const std::string& message) {
  if (!passed) throw std::runtime_error(message);
}

Input FlatInput(const Robot& robot, double gait_time) {
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.command.x() = 0.1;
  input.gait_time = gait_time;
  HeightMap map;
  map.origin = Eigen::Vector2d::Constant(-1.975);
  map.heights = Eigen::MatrixXd::Constant(80, 80, -0.032);  // 발 중심 z=0의 지면.
  map.valid = Eigen::ArrayXXi::Ones(80, 80);
  map.variance = Eigen::MatrixXd::Zero(80, 80);
  input.terrain = map;
  return input;
}

void Run(double gait_time) {
  Robot robot;
  Settings settings;
  settings.time_limit = 3.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input = FlatInput(robot, gait_time);
  const auto contacts = MakeHorizon(gait_time, robot.period, robot.duty).front().contacts;
  for (int leg = 0; leg < 4; ++leg) {
    if (!contacts[leg]) input.feet(leg, 2) = 0.10;
  }
  const Result result = Solve(robot, input, settings);
  Check(result.success, "airborne replan failed: " + result.status);
  Check(result.optimization_started && result.stage == "complete",
        "successful solve missing optimization/completion diagnostic");
  for (int leg = 0; leg < 4; ++leg) {
    Check(std::abs(result.footholds(leg, 2)) < settings.terrain.contact_tolerance,
          "future landing is above flat terrain");
  }
  // 측정된 현재 발 아래는 지지·스윙 모두 미관측이어도 된다. 다음 착지점은
  // 주변의 관측된 영역에서 선택하며, 현재 발 위치를 지도에 투영하지 않는다.
  Input unobserved = input;
  auto& map = *unobserved.terrain;
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector2d index =
        (input.feet.row(leg).head<2>().transpose() - map.origin) / map.resolution;
    const int col = static_cast<int>(std::floor(index.x()));
    const int row = static_cast<int>(std::floor(index.y()));
    map.valid.block<2, 2>(row, col).setZero();
  }
  const TerrainModel terrain(map, settings.terrain);
  for (int leg = 0; leg < 4; ++leg) {
    Check(terrain.Sample(input.feet.row(leg).head<2>().transpose()).occlusion > 1 - 1e-10,
          "current foot test location is still observed");
  }
  const auto replanned = Solve(robot, unobserved, settings);
  Check(replanned.optimization_started && replanned.iterations > 0,
        "unobserved current foot blocked optimization: " + replanned.status);
  Check(replanned.seed_context.initial_feet.isApprox(input.feet),
        "measured current feet changed on unobserved terrain");
  // 새 착지 후보의 지형 비용과 SQP 수렴은 별개다. 현재 위치 조회 때문에
  // problem_setup에서 거부하지 않는지 검증하며, 계산 성공으로 해석하지 않는다.
  std::cout << "PASS unobserved current feet reached SQP, gait_time=" << gait_time
            << " status=" << replanned.status << '\n';
  auto invalid_settings = settings;
  invalid_settings.time_limit = -1;
  const auto invalid = Solve(robot, input, invalid_settings);
  Check(!invalid.optimization_started && invalid.stage == "input_validation",
        "invalid settings did not preserve input validation stage");
  auto short_settings = settings;
  short_settings.time_limit = 1e-12;
  const auto timeout = Solve(robot, input, short_settings);
  // 예산 종료 계약: 현재 반복해가 실행 가능하면 feasible_time_limit 성공,
  // 아니면 time_limit 실패다. 어느 쪽이든 최적화 진입은 보장돼야 한다.
  bool timeout_contract = false;
  if (!timeout.success && timeout.status == "time_limit" && timeout.stage == "optimization") {
    timeout_contract = true;
  }
  if (timeout.success && timeout.status == "feasible_time_limit") {
    timeout_contract = true;
  }
  Check(timeout_contract && timeout.optimization_started,
        "time limit was not distinguished from pre-optimization rejection: " + timeout.status);
  std::cout << "PASS gait_time=" << gait_time << " status=" << result.status << '\n';
}

void CheckFeasibleTimeLimit() {
  // 정지 hold 입력은 초기 추정 자체가 실행 가능하다(위반 0). 예산을 사실상 0으로
  // 줄여 첫 반복에서 시간이 끝나도, 실패가 아니라 그 실행 가능 해를 반환해야 한다.
  // (보행 warm start 재구성 지점은 위반 ~0.04로 원래 실행 불가능이므로 쓰지 않는다.)
  Robot robot;
  Settings settings;
  settings.time_limit = 3.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input = FlatInput(robot, 0.4);
  input.command.setZero();
  input.body_time = 0.8;
  input.gait_hold_duration = robot.period;
  input.hold_contacts = std::array<bool, 4>{true, true, true, true};
  input.contact_estimated = input.hold_contacts;
  auto short_settings = settings;
  short_settings.time_limit = 1e-12;
  const Result kept = Solve(robot, input, short_settings);
  Check(kept.success && kept.status == "feasible_time_limit",
        "feasible iterate was discarded at time limit: " + kept.status);
  Check(kept.violation <= settings.feasibility_tolerance,
        "feasible_time_limit returned an infeasible plan");
  Check(!kept.coefficients.empty() && kept.footholds.allFinite(),
        "feasible_time_limit did not export the plan");
  Check(kept.stage == "complete", "feasible_time_limit did not complete export");
  std::cout << "PASS feasible_time_limit violation=" << kept.violation << '\n';
}

void CheckInfeasibleBudgetedExit() {
  // 도달 불가능한 확정 착지점은 무제한 예산에서 QP가 수 초씩 소모하며 실패한다
  // (OSQP maximum iterations). 실행 예산(0.1s)에서는 QP에 남은 예산이 전달되고
  // 시간 종료가 QP 실패로 둔갑하지 않아야 한다. 어느 실패 경로든 성공을 날조하지
  // 않고 예산 근방에서 종료하는 것이 계약이다. infeasible_stalled(최대 페널티
  // 위반 정체)는 온라인 발산 패턴 전용 방어망으로, QP가 먼저 실패하는 이
  // 합성 입력에서는 요구하지 않는다.
  Robot robot;
  Settings settings;
  settings.time_limit = 0.1;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.command.x() = 0.1;
  input.gait_time = 0.6;
  input.committed_footholds[0] = Eigen::Vector3d(1.5, -1.5, 0.0);
  const Result infeasible = Solve(robot, input, settings);
  Check(!infeasible.success, "unreachable committed foothold produced a plan");
  Check(infeasible.optimization_started, "infeasible input rejected before optimization");
  const bool bounded_status = infeasible.status == "time_limit" ||
                              infeasible.status == "infeasible_stalled" ||
                              infeasible.status == "line_search_failed" ||
                              infeasible.status.rfind("qp_failed", 0) == 0;
  Check(bounded_status, "unexpected infeasible exit status: " + infeasible.status);
  Check(infeasible.elapsed < 1.0,
        "infeasible input was not bounded by the budget: " +
            std::to_string(infeasible.elapsed));
  std::cout << "PASS infeasible budgeted exit status=" << infeasible.status
            << " elapsed=" << infeasible.elapsed << '\n';
}

void CheckDelayedContact() {
  Robot robot;
  Settings settings;
  settings.time_limit = 3.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input = FlatInput(robot, 0.4);
  input.feet(1, 2) = 0.05;
  input.feet(2, 2) = 0.05;
  input.contact_estimated = std::array<bool, 4>{true, false, false, true};
  const Result delayed = Solve(robot, input, settings);
  Check(delayed.optimization_started && delayed.iterations > 0,
        "estimated airborne feet rejected at scheduled touchdown");
  // 같은 높이 차이에서 추정 접촉이 ON으로 바뀌어도 최적화 입력을 거부하지 않는다.
  input.contact_estimated = std::array<bool, 4>{true, true, true, true};
  const Result supported = Solve(robot, input, settings);
  Check(supported.optimization_started && supported.iterations > 0,
        "estimated support height mismatch blocked optimization");
  std::cout << "PASS contact-independent current foot height, solve status=" << delayed.status << '\n';
}
void CheckSupportHeightOffset() {
  Robot robot;
  Settings settings;
  settings.time_limit = 3.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input = FlatInput(robot, 0.4);
  input.command.setZero();
  input.body_time = 0.8;
  input.gait_hold_duration = robot.period;
  input.hold_contacts = std::array<bool, 4>{true, true, true, true};
  input.contact_estimated = input.hold_contacts;
  // 5mm 허용치 밖의 지지 높이를 지도로 덮어쓰지 않고 정지 몸통 계획을 계산한다.
  for (double offset : {-0.01, 0.01}) {
    input.feet.col(2).setConstant(offset);
    const Result result = Solve(robot, input, settings);
    Check(result.success && result.optimization_started,
          "zero-command support hold failed with height offset: " + result.status);
    Check(result.seed_context.initial_feet.isApprox(input.feet),
          "current support feet were projected onto the terrain");
    for (int leg = 0; leg < 4; ++leg) {
      Check(std::abs(result.footholds(leg, 2)) < settings.terrain.contact_tolerance,
            "future foothold inherited current support height offset");
    }
  }
  std::cout << "PASS zero-command hold with +/-10mm support height offsets\n";
}
// 실지형 재현: 관측 높이는 알려졌지만 미관측 셀이 흩어져 필터 지원 영역(9x9)이
// 어디에서도 전부 known이 아니다. 과거에는 모든 질의가 TerrainDomainError였고
// 첫 계획 생성이 막혔다. 높이·경계 비용 유효성 분리 후에는 계획을 생성해야 한다.
void CheckUnobservedNeighborhood() {
  Robot robot;
  Settings settings;
  settings.time_limit = 3.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  Input input = FlatInput(robot, 0.4);
  HeightMap& map = *input.terrain;
  // 0.35m(7셀) 간격의 고립 구멍. 어떤 9x9 창에도 구멍이 포함돼 필터 지원 영역은
  // 비지만, 2x2 착지 블록은 대부분 관측 상태로 남는다.
  int holes = 0;
  for (int row = 3; row < map.valid.rows(); row += 7) {
    for (int col = 3; col < map.valid.cols(); col += 7) {
      map.valid(row, col) = 0;
      ++holes;
    }
  }
  Check(holes > 0, "test map had no unobserved cells");
  const Result result = Solve(robot, input, settings);
  Check(result.success && result.optimization_started,
        "plan not generated on map with scattered unobserved cells: " + result.status);
  for (int leg = 0; leg < 4; ++leg) {
    Check(std::abs(result.footholds(leg, 2)) < settings.terrain.contact_tolerance,
          "future landing left the observed ground height");
  }
  std::cout << "PASS first plan on map with " << holes
            << " scattered holes, status=" << result.status << '\n';
}
}  // namespace

int main() {
  try {
    // 양쪽 대각선 스윙 중간과 두 착지 경계에서 접촉 상태가 올바르게 전환돼야 한다.
    for (double time : {0.0, 0.2, 0.4, 0.6, 0.8}) Run(time);
    CheckDelayedContact();
    CheckSupportHeightOffset();
    CheckUnobservedNeighborhood();
    CheckFeasibleTimeLimit();
    CheckInfeasibleBudgetedExit();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
