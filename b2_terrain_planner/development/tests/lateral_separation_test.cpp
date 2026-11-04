/**
 * @file lateral_separation_test.cpp
 * @brief 발 주변 셀·연속 착지점의 지수 거리 비용과 회전 불변성을 검증한다.
 * @version 0.0.2
 * @update 2026-11-04: 앞/뒤 좌우 발 쌍의 Y 간격에만 근접 지수 비용을 적용함.
 * @update 2026-11-04: 타원 거리장의 미분·후보 셀 선택·회전 중 근접 착지 회피를 추가함.
 */
#include "b2_terrain_planner/planner.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <Eigen/Geometry>

namespace {
using namespace b2_terrain_planner;
void Check(bool value, const std::string& reason) {
  if (!value) throw std::runtime_error(reason);
}
HeightMap FlatMap() {
  HeightMap map;
  map.origin = Eigen::Vector2d::Constant(-2.0);
  map.heights = Eigen::MatrixXd::Constant(81, 81, -0.032);
  map.valid = Eigen::ArrayXXi::Ones(81, 81);
  map.variance = Eigen::MatrixXd::Zero(81, 81);
  return map;
}
void CheckResidual() {
  TerrainSettings settings;
  settings.foot_separation_weight = 0.1;
  const TerrainModel terrain(FlatMap(), settings);
  for (double yaw : {0.0, 0.7, 1.5707963267948966, 2.8}) {
    const Eigen::Rotation2Dd rotation(yaw);
    for (double width : {-0.05, 0.04, 0.08, 0.10, 0.20, 0.40}) {
      const Eigen::Vector2d local(0.02, width);
      const Eigen::Vector2d delta = rotation * local;
      const auto result = terrain.FootSeparation(delta, yaw);
      const auto reference = terrain.FootSeparation(local, 0.0);
      Check(std::abs(result.value - reference.value) < 1e-10,
            "rotating the feet and heading changed the cost");
      Check(result.value > 0.0 && result.gradient.allFinite(), "far/crossed cost invalid");
      for (int axis = 0; axis < 2; ++axis) {
        auto plus = delta;
        auto minus = delta;
        plus[axis] += 1e-6;
        minus[axis] -= 1e-6;
        const double numerical = (terrain.FootSeparation(plus, yaw).value -
                                  terrain.FootSeparation(minus, yaw).value) / 2e-6;
        Check(std::abs(numerical - result.gradient[axis]) <
                  1e-5 * (1.0 + std::abs(numerical)), "proximity Jacobian mismatch");
      }
    }
  }
  const auto edge = terrain.FootSeparation({0.0, 0.08}, 0);
  const auto shifted = terrain.FootSeparation({0.8, 0.08}, 0);
  Check(std::abs(edge.value * edge.value - 0.1) < 1e-9, "Y boundary mismatch");
  Check(edge.value == shifted.value && shifted.gradient.x() == 0.0,
        "X separation changed the Y-only cost");
  const auto close = terrain.FootSeparation({0.0, 0.07}, 0);
  Check(std::abs(std::pow(close.value / edge.value, 2) - std::exp(1.0)) < 1e-7,
        "one scale inward did not multiply cost by e");
  const auto center = terrain.FootSeparation({0, 0}, 0);
  Check(std::isfinite(center.value) && center.gradient.allFinite(), "center is not finite");
  const auto far = terrain.FootSeparation({0.0, 0.40}, 0);
  Check(far.value * far.value < 1e-10, "ordinary stance is still being widened");
  settings.foot_separation_scale = 0.001;
  const auto tail = TerrainModel(FlatMap(), settings).FootSeparation({0, 0.001}, 0);
  Check(std::isfinite(tail.value) && tail.gradient.allFinite() && tail.gradient.y() < 0,
        "steep exponential tail lost a finite repulsive gradient");
  settings.foot_separation_weight = 0.0;
  const auto disabled = TerrainModel(FlatMap(), settings).FootSeparation({0, 0}, 0);
  Check(disabled.value == 0 && disabled.gradient.isZero(), "zero weight is not disabled");
}
void CheckCandidateCells() {
  TerrainSettings settings;
  const SupportPositions supports{Eigen::Vector2d::Zero()};
  const auto off = TerrainModel(FlatMap(), settings).FindFoothold(
      {0, 0.08}, {-0.001, 0.08}, {0.001, 0.25}, {}, 0, {}, TerrainLayer::kGround, supports, 0.0, supports);
  settings.foot_separation_weight = 0.1;
  const TerrainModel terrain(FlatMap(), settings);
  const auto on = terrain.FindFoothold(
      {0, 0.08}, {-0.001, 0.08}, {0.001, 0.25}, {}, 0, {}, TerrainLayer::kGround, supports, 0.0, supports);
  Check(on.y() > off.y() + 0.02, "nearby candidate cells ignored the foot distance field");
  const auto unpaired = terrain.FindFoothold(
      {0, 0.08}, {-0.001, 0.08}, {0.001, 0.25}, {}, 0, {}, TerrainLayer::kGround, supports);
  Check((unpaired - off).norm() < 1e-10,
        "occupancy-only support unexpectedly added lateral cost");
  const Eigen::Vector2d cell(0, 0.15);
  const auto residual = terrain.FootSeparation(cell, 0);
  const double score = terrain.FootholdCost(cell, cell, {}, 0, TerrainLayer::kGround, supports);
  Check(std::abs(score - residual.value * residual.value) < 1e-10,
        "candidate cell and continuous SQP cost disagree");
}
void CheckFootholds() {
  Robot robot;
  for (double yaw : {0.0, 1.5707963267948966}) {
    for (double turn : {-0.25, 0.25}) {
      Input input;
      input.position.z() = robot.height;
      input.position[3] = yaw;
      input.feet = robot.hips;
      input.has_nominal = true;
      const Eigen::Rotation2Dd rotation(yaw);
      for (int leg = 0; leg < 4; ++leg) {
        input.feet.row(leg).head<2>() =
            (rotation * robot.hips.row(leg).head<2>().transpose()).transpose();
        Eigen::Vector2d narrow = robot.hips.row(leg).head<2>().transpose();
        narrow.y() *= 0.1;
        input.nominal.row(leg) = (rotation * narrow).transpose();
      }
      input.command << 0.1, 0.0, turn;
      input.terrain = FlatMap();
      Settings settings;
      settings.time_limit = 3.0;
      settings.sample_intervals = 4;
      settings.structured_equalities = true;
      settings.lapack_svd = true;
      const auto baseline = Solve(robot, input, settings);
      Check(baseline.success, "baseline solve failed: " + baseline.status);
      settings.terrain.foot_separation_weight = 0.1;
      const auto spread = Solve(robot, input, settings);
      Check(spread.success, "proximity solve failed: " + spread.status);
      for (int left : {0, 2}) {
        const double before =
            (baseline.footholds.row(left) - baseline.footholds.row(left + 1)).head<2>().norm();
        const double after =
            (spread.footholds.row(left) - spread.footholds.row(left + 1)).head<2>().norm();
        std::cout << "yaw=" << yaw << " turn=" << turn << " pair=" << left
                  << " baseline=" << before << " proximity=" << after << '\n';
        Check(after > 0.064, "turning foothold pair still overlaps the feet");
      }
    }
  }
}
}  // namespace
int main() {
  try {
    CheckResidual();
    CheckCandidateCells();
    CheckFootholds();
    std::cout << "PASS cell/continuous proximity cost, gradients and turning footholds\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
