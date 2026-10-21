/**
 * @file terrain_reuse_test.cpp
 * @brief 지도 영역 이동 시 이전 궤적 재사용과 변경 지형의 착지점 재검사를 검증한다.
 * @version 0.0.1
 * @update 2026-10-17: 이동 평면·높이 변경·미관측 영역·좌표계 불일치 회귀 시험을 추가함.
 */
#include "b2_terrain_planner/planner.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace b2_terrain_planner;
void Check(bool passed, const std::string& reason) {
  if (!passed) throw std::runtime_error(reason);
}
}  // namespace

int main() {
  try {
    Robot robot;
    Settings settings;
    settings.time_limit = 3.0;
    settings.sample_intervals = 4;
    settings.structured_equalities = true;
    settings.lapack_svd = true;
    Input input;
    input.position.z() = robot.height;
    input.feet = robot.hips;
    input.command.x() = 0.1;
    HeightMap map;
    map.origin = Eigen::Vector2d::Constant(-1.975);
    map.heights = Eigen::MatrixXd::Zero(80, 80);
    map.valid = Eigen::ArrayXXi::Ones(80, 80);
    map.variance = Eigen::MatrixXd::Zero(80, 80);
    input.terrain = map;
    const Result initial = Solve(robot, input, settings);
    Check(initial.success, "initial solve: " + initial.status);
    input.gait_time = 0.02;
    input.position = Evaluate(initial, input.gait_time);
    input.velocity = Evaluate(initial, input.gait_time, 1);
    input.terrain->origin += Eigen::Vector2d(0.05, -0.1);
    const Result moved = Solve(robot, input, settings, &initial);
    Check(moved.success && moved.warm_start.used,
          "moving map rejected: " + moved.status + ", warm=" + moved.warm_start.reason);
    Check(moved.warm_start.reason == "shifted_previous_plan_updated_terrain",
          "changed map diagnostic missing");
    Check(moved.terrain_validation && moved.terrain_validation->passed,
          "moving map validation missing");
    // 평면 높이가 바뀌면 이전 착지 Z를 그대로 복사해서는 안 된다.
    input.terrain->heights.setConstant(0.02);
    input.feet.col(2).setConstant(0.02);
    input.position.z() += 0.02;
    const Result raised = Solve(robot, input, settings, &initial);
    Check(raised.success && raised.warm_start.used, "changed height solve: " + raised.status);
    Check(raised.warm_start.reused_footholds == 0, "old-height footholds reused");
    Check(raised.terrain_validation && raised.terrain_validation->passed,
          "new terrain validation failed");
    for (int leg = 0; leg < 4; ++leg) {
      Check(std::abs(raised.footholds(leg, 2) - 0.02) < settings.terrain.contact_tolerance,
            "landing height was not updated");
    }
    Input unknown = input;
    unknown.terrain->valid.setZero();
    Check(!Solve(robot, unknown, settings, &initial).success,
          "unknown terrain accepted using old plan");
    Input outside = input;
    outside.terrain->origin += Eigen::Vector2d(10, 10);
    Check(!Solve(robot, outside, settings, &initial).success,
          "old plan bypassed map bounds");
    input.frame = "other_odom";
    input.terrain->frame = input.frame;
    const Result mismatch = Solve(robot, input, settings, &initial);
    Check(!mismatch.warm_start.used && mismatch.warm_start.reason == "context_mismatch",
          "different coordinate frame reused");
    std::cout << "PASS rolling map reuse, changed landing height, unknown/bounds/frame guards\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
