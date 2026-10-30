/**
 * @file terrain_pipeline_test.cpp
 * @brief 희소 지도 경계·돌 후보·가상 바닥 및 수치 미분과 단계 초기화의 동작 검증.
 * @version 0.0.1
 * @update 2026-10-30: 지형 처리와 후보 보정 및 평지 재계획 회귀를 추가함.
 */
#include "b2_terrain_planner/planner.h"
#include "tamols_kernels.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace b2_terrain_planner;
void Check(bool ok, const std::string& reason) {
  if (!ok) throw std::runtime_error(reason);
}
HeightMap Flat(double height = 0.0) {
  HeightMap map;
  map.origin = Eigen::Vector2d::Constant(-1.975);
  map.heights = Eigen::MatrixXd::Constant(80, 80, height);
  map.variance = Eigen::MatrixXd::Zero(80, 80);
  map.valid = Eigen::ArrayXXi::Ones(80, 80);
  return map;
}
void CheckDerivative(const TerrainModel& terrain, const Eigen::Vector3d& point) {
  const auto residual = terrain.Residual(point);
  constexpr double step = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta[axis] = step;
    const auto finite = (terrain.Residual(point + delta).values -
                         terrain.Residual(point - delta).values) / (2 * step);
    Check((finite - residual.jacobian.col(axis)).norm() < 1e-5,
          "processed terrain Jacobian inconsistent with residual");
  }
}
void Mapping() {
  auto map = Flat();
  // 15cm 폭의 돌 두 개와 10cm 틈. 셀 중심 정렬을 고정해 실제 형상을 비교한다.
  map.heights.block(38, 36, 3, 3).setConstant(1.0);
  map.heights.block(38, 41, 3, 3).setConstant(1.0);
  // 돌 주변 관측 구멍으로 이전 구현의 넓은 halo 비용이 사라지는 경우.
  map.valid(34, 38) = 0;
  const auto original = map;
  const auto start = std::chrono::steady_clock::now();
  TerrainModel terrain(map, {});
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  Check((terrain.map().valid == original.valid).all() &&
        (terrain.map().heights - original.heights).norm() == 0,
        "processing mutated source map");
  const Eigen::Vector2d stone = map.origin + .05 * Eigen::Vector2d(37, 39);
  const Eigen::Vector2d gap = map.origin + .05 * Eigen::Vector2d(39.5, 39);
  Check(terrain.Sample(gap).virtual_height > terrain.Sample(gap).height + .3,
        "virtual floor failed to bridge the gap");
  const auto found = terrain.FindFoothold(gap, gap.array() - .2, gap.array() + .2);
  Check(found.z() > .95, "foothold selected gap floor instead of a stone");
  Check((found.head<2>() - stone).norm() < .051 ||
        (found.head<2>() - stone - Eigen::Vector2d(.25, 0)).norm() < .051,
        "foothold not inside the stone");
  CheckDerivative(terrain, Eigen::Vector3d(stone.x() + .013, stone.y() + .017, 1.032));
  std::cout << "terrain processing seconds=" << seconds << " foothold=" << found.transpose() << '\n';

  auto flat = Flat();
  flat.valid.block(38, 38, 4, 4).setZero();
  TerrainModel sparse(flat, {});
  const Eigen::Vector2d unknown = Eigen::Vector2d::Zero();
  const Eigen::Vector2d known(.3, 0);
  Check(sparse.Sample(unknown).occlusion > .99 && sparse.Sample(known).occlusion == 0,
        "original observation mask was lost");
  Check(sparse.FootholdCost(unknown, unknown, Eigen::Vector2d::Zero(), 0) >
        sparse.FootholdCost(known, unknown, Eigen::Vector2d::Zero(), 0),
        "unknown cell preferred over nearby observed flat ground");
}
void AlignmentDerivative() {
  Vector6 pose;
  pose << .1, -.2, .6, .3, .1, -.15;
  Eigen::Vector3d hip(.3, .2, -.02), point;
  Eigen::Matrix<double, 3, 6> jacobian;
  const double* args[] = {pose.data(), hip.data()};
  double* outputs[] = {point.data(), jacobian.data()};
  tamols_terrain_alignment(args, outputs, nullptr, nullptr, 0);
  for (int axis = 0; axis < 6; ++axis) {
    Vector6 shifted = pose;
    shifted[axis] += 1e-6;
    Eigen::Vector3d plus, minus;
    const double* a[] = {shifted.data(), hip.data()};
    double* b[] = {plus.data(), nullptr};
    tamols_terrain_alignment(a, b, nullptr, nullptr, 0);
    shifted[axis] -= 2e-6;
    b[0] = minus.data();
    tamols_terrain_alignment(a, b, nullptr, nullptr, 0);
    Check(((plus - minus) / 2e-6 - jacobian.col(axis)).norm() < 1e-6,
          "body alignment derivative mismatch");
  }
}
void Replanning() {
  Robot robot;
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.terrain = Flat(-.032);
  input.command.x() = .1;
  Settings settings;
  settings.time_limit = 2.0;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  const auto first = Solve(robot, input, settings);
  Check(first.success, "processed flat solve failed: " + first.status);
  Check(first.terrain_snapshot && first.terrain_processing_seconds > 0,
        "processed input diagnostics missing");
  Check(first.terrain_initialization_status == "refined_virtual_floor",
        "three-stage initialization was not exercised: " + first.terrain_initialization_status);
  input.gait_time = .02;
  input.position = Evaluate(first, .02);
  input.velocity = Evaluate(first, .02, 1);
  const auto next = Solve(robot, input, settings, &first);
  Check(next.success && next.warm_start.used, "same-slot warm start failed");
  Check(next.terrain_initialization_status == "not_needed",
        "same-slot warm start repeated coarse optimization");
  // 준비 예산이 없으면 직접 초기화하며 동일한 실제 지형 SQP를 수행한다.
  settings.graduated_budget_fraction = 1e-6;
  const auto fallback = Solve(robot, input, settings);
  Check(fallback.success, "direct initialization fallback failed: " + fallback.status);
  Check(fallback.terrain_initialization_status == "direct_initialization_budget",
        "preparation budget fallback was not exercised");
  std::cout << "solve seconds=" << first.elapsed << " warm=" << next.elapsed << '\n';
}
}  // namespace
int main() {
  try {
    Mapping();
    AlignmentDerivative();
    Replanning();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "PASS terrain processing, sparse edges, stone selection, derivatives, replanning\n";
}
