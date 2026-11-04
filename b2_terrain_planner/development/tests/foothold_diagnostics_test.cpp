/**
 * @file foothold_diagnostics_test.cpp
 * @brief 지도 보간 출처·단계별 비용 기록과 진단 ON/OFF 계획 불변성을 검증한다.
 * @version 0.0.1
 * @update 2026-11-05: 희소 지도·초기화·재사용·확정 착지점의 진단 회귀를 추가함.
 */
#include "b2_terrain_planner/planner.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace {
using namespace b2_terrain_planner;

void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

HeightMap Flat() {
  HeightMap map;
  map.origin = Eigen::Vector2d::Constant(-1.975);
  map.heights = Eigen::MatrixXd::Constant(80, 80, -.032);
  map.valid = Eigen::ArrayXXi::Ones(80, 80);
  map.variance = Eigen::MatrixXd::Zero(80, 80);
  return map;
}

void CheckMap() {
  auto map = Flat();
  map.valid(40, 40) = 0;
  map.heights(40, 40) = std::numeric_limits<double>::quiet_NaN();
  TerrainSettings settings;
  settings.foot_separation_weight = .1;
  TerrainModel terrain(map, settings);
  const Eigen::Vector2d xy = map.origin + .05 * Eigen::Vector2d(39.25, 39.75);
  const auto point = terrain.Inspect(xy);
  Check(point.valid && point.lower_cell == Eigen::Vector2i(39, 39), "wrong cells");
  Check(point.observed == std::array<bool, 4>{true, true, true, false}, "mask lost");
  Check(std::abs(point.weights[3] - .1875) < 1e-12, "wrong unknown weight");
  Check(std::abs(point.occlusion - terrain.Sample(xy).occlusion) < 1e-12,
        "occlusion differs from optimizer map");
  Check(std::abs(point.height - terrain.Sample(xy).height) < 1e-12,
        "height differs from optimizer map");
  Check(std::isnan(point.input_heights[3]) && std::isfinite(point.processed_heights[3]),
        "inpainted height not distinguishable from sensor observation");
  Check(!terrain.Inspect(Eigen::Vector2d(3, 3)).valid, "outside point accepted");
  Check(terrain.map().valid(40, 40) == 0 && std::isnan(terrain.map().heights(40, 40)),
        "diagnostic mutated map");
  const SupportPositions supports{xy + Eigen::Vector2d(0, -.1)};
  for (auto layer : {TerrainLayer::kGround, TerrainLayer::kVirtualFloor}) {
    const auto terms = terrain.CandidateCosts(xy, xy + Eigen::Vector2d(.03, .01),
        Eigen::Vector2d::UnitX(), 2., layer, supports, .2);
    const double score = terrain.FootholdCost(xy, xy + Eigen::Vector2d(.03, .01),
        Eigen::Vector2d::UnitX(), 2., layer, supports, .2);
    Check(std::abs(std::accumulate(terms.begin(), terms.end(), 0.) - score) < 1e-12,
          "candidate cost breakdown differs from selector");
  }
}

const FootholdTrace& Stage(const Result& result, const std::string& stage) {
  for (const auto& trace : result.foothold_trace) {
    if (trace.layer == "ground" && trace.stage == stage) return trace;
  }
  throw std::runtime_error("missing trace: " + stage);
}

void CheckCosts(const Result& result) {
  Check(result.foothold_diagnostic_seconds > 0, "capture time absent");
  Check(result.foothold_trace.size() <= 25, "unbounded diagnostic buffer");
  for (const auto& trace : result.foothold_trace) {
    if (!trace.sqp_valid) continue;
    const double sum = std::accumulate(trace.sqp_costs.begin(), trace.sqp_costs.end(), 0.);
    Check(std::abs(sum - trace.total_cost) < 1e-10 * (1 + std::abs(trace.total_cost)),
          "SQP cost breakdown differs from evaluated objective");
  }
  Check((Stage(result, "sqp_final").feet - result.footholds).norm() < 1e-12,
        "final trace differs from exported feet");
}

void CheckCrossing() {
  Robot robot;
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.command.x() = .2;
  auto map = Flat();
  // FL 앞쪽에 미관측 띠를 둔다. 후보는 피하지만 최종 SQP는 안쪽으로 이동한다.
  map.valid.block(42, 51, 5, 2).setZero();
  input.terrain = map;
  Settings settings;
  settings.time_limit = 2.;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  settings.graduated_initialization = false;
  settings.foothold_diagnostics = true;
  const auto result = Solve(robot, input, settings);
  Check(result.success, "sparse solve failed: " + result.status);
  CheckCosts(result);
  const auto& before = Stage(result, "first_unobserved_before");
  const auto& after = Stage(result, "first_unobserved_after");
  Check(before.crossing_leg == 0 && after.crossing_leg == 0 &&
        before.iteration == after.iteration, "crossing event pairing failed");
  Check(before.terrain[0].occlusion < 1e-9 && after.terrain[0].occlusion > .1,
        "crossing does not describe observed-to-unknown movement");
  Check(after.merit < before.merit && after.alpha > 0 && after.alpha <= 1,
        "accepted step merit or alpha missing");
  Check(std::abs(after.merit - after.total_cost - after.penalty * after.scaled_violation)
            < 1e-10, "accepted merit differs from line search");
  int events = 0;
  for (const auto& trace : result.foothold_trace) {
    if (trace.stage == "first_unobserved_after" && trace.crossing_leg == 0) ++events;
  }
  Check(events == 1, "crossing recorded repeatedly for the same leg");
}

void CheckPlanner() {
  Robot robot;
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.terrain = Flat();
  input.command.x() = .1;
  Settings settings;
  settings.time_limit = 2.;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  const auto off = Solve(robot, input, settings);
  settings.foothold_diagnostics = true;
  const auto on = Solve(robot, input, settings);
  Check(off.success && on.success, "flat solve failed");
  Check(off.foothold_trace.empty(), "disabled diagnostics still captured");
  Check((off.footholds - on.footholds).norm() < 1e-12 &&
        std::abs(off.cost - on.cost) < 1e-12 && off.iterations == on.iterations,
        "diagnostics changed solver result");
  CheckCosts(on);
  Stage(on, "refinement_selected");
  for (const auto& source : Stage(on, "sqp_start").sources) {
    Check(source == "refined_candidate", "refined seed origin missing");
  }
  input.gait_time = .02;
  input.position = Evaluate(on, .02);
  input.velocity = Evaluate(on, .02, 1);
  // 비용 비교로 새 후보가 선택되는 경로와 분리해 이전 발 재사용을 시험한다.
  settings.foothold_backward_weight = 0.;
  const auto warm = Solve(robot, input, settings, &on);
  Check(warm.success && warm.warm_start.used, "warm solve failed");
  CheckCosts(warm);
  int reused = 0;
  for (const auto& source : Stage(warm, "sqp_start").sources) {
    if (source == "previous_success") ++reused;
  }
  Check(reused == warm.warm_start.reused_footholds && reused > 0,
        "reused feet provenance mismatch: trace=" + std::to_string(reused) +
        " solver=" + std::to_string(warm.warm_start.reused_footholds));
  input.committed_footholds[0] = warm.footholds.row(0).transpose();
  const auto committed = Solve(robot, input, settings, &warm);
  Check(committed.success, "committed solve failed: " + committed.status);
  CheckCosts(committed);
  Check(Stage(committed, "sqp_start").sources[0] == "committed_swing",
        "committed seed origin missing");
  std::cout << "capture_seconds=" << on.foothold_diagnostic_seconds
            << " elapsed=" << on.elapsed << " off_elapsed=" << off.elapsed << '\n';
}
}  // namespace

int main() {
  try {
    CheckMap();
    CheckPlanner();
    CheckCrossing();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "PASS map provenance, cost breakdown, seed stages and unchanged plans\n";
}
