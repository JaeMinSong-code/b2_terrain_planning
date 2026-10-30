/**
 * @file swing_terrain_test.cpp
 * @brief 스윙·지지발의 높이 불일치 허용과 다음 착지점의 지형 검사를 검증한다.
 * @version 0.0.7
 * @update 2026-10-30: 보간 높이 조회와 원본 관측 마스크를 구분해 스윙·접촉 계약을 검증함.
 * @update 2026-10-29: 지면·발 중심 변환과 반경 누락·중복 방지 회귀를 검증함.
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
  Check(!result.terrain_validation, "removed final terrain checker still ran");
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
  invalid_settings.time_limit = 0;
  const auto invalid = Solve(robot, input, invalid_settings);
  Check(!invalid.optimization_started && invalid.stage == "input_validation",
        "invalid settings did not preserve input validation stage");
  auto short_settings = settings;
  short_settings.time_limit = 1e-12;
  const auto timeout = Solve(robot, input, short_settings);
  Check(!timeout.success && timeout.optimization_started && timeout.status == "time_limit" &&
            timeout.stage == "optimization",
        "time limit was not distinguished from pre-optimization rejection");
  std::cout << "PASS gait_time=" << gait_time << " status=" << result.status << '\n';
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
    Check(!result.terrain_validation, "removed final terrain checker still ran");
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
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
