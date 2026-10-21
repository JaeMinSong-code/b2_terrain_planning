/**
 * @file swing_terrain_test.cpp
 * @brief 공중에서 시작하는 양쪽 대각선 스윙 계획과 지지 발 높이 검사를 검증한다.
 * @version 0.0.1
 * @update 2026-10-17: 스윙 입력 허용·지지 오류 거부·착지 지면 일치 회귀 시험을 추가함.
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
  Check(result.terrain_validation.has_value() && result.terrain_validation->passed,
        "landing terrain validation failed");
  for (int leg = 0; leg < 4; ++leg) {
    Check(std::abs(result.footholds(leg, 2)) < settings.terrain.contact_tolerance,
          "future landing is above flat terrain");
  }
  // 같은 높이 차이라도 지지 발이면 잘못된 입력으로 거부한다.
  for (int leg = 0; leg < 4; ++leg) {
    if (!contacts[leg]) continue;
    Input invalid = input;
    invalid.feet(leg, 2) = 0.10;
    const auto rejected = Solve(robot, invalid, settings);
    Check(!rejected.success &&
              rejected.status == "input feet do not match observed terrain height",
          "support height mismatch was accepted");
  }
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
  Check(delayed.iterations > 0 &&
            delayed.status != "input feet do not match observed terrain height",
        "estimated airborne feet rejected at scheduled touchdown");
  // 실제 접촉이라고 보고된 동일 상태는 높이 불일치로 거부해야 한다.
  input.contact_estimated = std::array<bool, 4>{true, true, true, true};
  const Result supported = Solve(robot, input, settings);
  Check(!supported.success &&
            supported.status == "input feet do not match observed terrain height",
        "estimated support height mismatch accepted");
  std::cout << "PASS delayed contact filtering, solve status=" << delayed.status << '\n';
}
}  // namespace

int main() {
  try {
    // 양쪽 대각선 스윙 중간과 두 착지 경계에서 접촉 상태가 올바르게 전환돼야 한다.
    for (double time : {0.0, 0.2, 0.4, 0.6, 0.8}) Run(time);
    CheckDelayedContact();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
