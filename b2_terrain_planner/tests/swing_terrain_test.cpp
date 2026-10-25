/**
 * @file swing_terrain_test.cpp
 * @brief 스윙·지지발의 높이 불일치 허용과 다음 착지점의 지형 검사를 검증한다.
 * @version 0.0.4
 * @update 2026-10-25: 사후 지도 검사 없이 기존 지형 비용과 착지점 계산이 유지되는지 검증함.
 * @update 2026-10-23: 지지발 높이 차이에서 정지 계획 생성과 지도 조회 오류 거부를 검증함.
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
  map.heights = Eigen::MatrixXd::Zero(80, 80);
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
  // 현재 발의 지도 조회 가능성은 높이 일치 검사와 별도로 유지한다.
  Input outside = input;
  outside.feet(0, 0) = 100.0;
  const auto rejected = Solve(robot, outside, settings);
  Check(!rejected.success && !rejected.optimization_started &&
            rejected.stage == "problem_setup" &&
            rejected.status == "input foot position not queryable",
        "out-of-map input foot was not rejected before optimization");
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
}  // namespace

int main() {
  try {
    // 양쪽 대각선 스윙 중간과 두 착지 경계에서 접촉 상태가 올바르게 전환돼야 한다.
    for (double time : {0.0, 0.2, 0.4, 0.6, 0.8}) Run(time);
    CheckDelayedContact();
    CheckSupportHeightOffset();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
