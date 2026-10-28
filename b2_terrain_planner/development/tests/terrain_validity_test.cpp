/**
 * @file terrain_validity_test.cpp
 * @brief 착지 높이 조회 유효성과 경계 비용 유효성의 분리를 검증한다.
 * @version 0.0.2
 * @update 2026-10-27: 분산값이 높이 조회·경계 비용·착지 후보를 차단하지 않음을 검증함.
 * @update 2026-10-26: 관측 착지 셀이 주변 미관측에도 조회 가능하고, 경계 비용은
 *                     필터 지원 영역이 known일 때만 적용됨을 검증함.
 */
#include "b2_terrain_planner/terrain.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace b2_terrain_planner;

void Check(bool passed, const std::string& message) {
  if (!passed) throw std::runtime_error(message);
}

// 좌반부 0, 우반부 0.1의 단차를 둔 24x24 지도. col 12 경계에서 기울기가 생긴다.
HeightMap StepMap() {
  HeightMap map;
  const int n = 24;
  map.origin = Eigen::Vector2d::Zero();
  map.resolution = 0.05;
  map.heights = Eigen::MatrixXd::Zero(n, n);
  map.variance = Eigen::MatrixXd::Zero(n, n);
  map.valid = Eigen::ArrayXXi::Ones(n, n);
  for (int row = 0; row < n; ++row) {
    for (int col = 12; col < n; ++col) map.heights(row, col) = 0.1;
  }
  return map;
}

Eigen::Vector2d CellCenter(const HeightMap& map, int row, int col) {
  return map.origin + map.resolution * Eigen::Vector2d(col + 0.5, row + 0.5);
}

void Run() {
  TerrainSettings settings;  // 기본값: smoothing_sigma 0.05, resolution 0.05 → halo 4.

  const TerrainModel full(StepMap(), settings);
  const Eigen::Vector2d probe = CellCenter(StepMap(), 10, 11);
  Check(full.IsQueryable(probe), "fully observed step cell not queryable");
  const auto full_sample = full.Sample(probe);
  Check(full_sample.edge.squaredNorm() > 1e-6,
        "observed step lost its boundary cost");

  // 기록된 CuPy 범위와 비정상 분산 메타데이터 모두 관측 높이를 차단하지 않는다.
  for (double variance : {0.012455, 0.107074, 0.474787, -1.0,
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
    HeightMap uncertain = StepMap();
    uncertain.variance.setConstant(variance);
    const TerrainModel terrain(uncertain, settings);
    Check(terrain.IsQueryable(probe), "variance blocked observed terrain");
    const auto sample = terrain.Sample(probe);
    Check(std::abs(sample.height - full_sample.height) < 1e-12,
          "variance changed observed height");
    Check((sample.edge - full_sample.edge).norm() < 1e-12,
          "variance changed the boundary cost");
    const Eigen::Vector2d nominal = CellCenter(uncertain, 10, 16);
    const auto found = terrain.FindFoothold(
        nominal, nominal.array() - 0.10, nominal.array() + 0.10);
    Check(std::abs(found.z() - 0.1) < 1e-6,
          "variance blocked an observed foothold candidate");
  }

  // 같은 질의점을 포함하지 않는 이웃 셀 하나만 미관측으로 바꾼다.
  // (14,7)은 (10,11)의 필터 지원 영역(halo=4) 안이지만 2x2 착지 블록 밖이다.
  HeightMap holed = StepMap();
  holed.valid(14, 7) = 0;
  const TerrainModel partial(holed, settings);
  Check(partial.IsQueryable(probe),
        "observed landing cell became unqueryable due to a neighboring hole");
  const auto partial_sample = partial.Sample(probe);
  Check(std::abs(partial_sample.height - full_sample.height) < 1e-12,
        "landing height changed when a neighbor went unobserved");
  Check(partial_sample.edge.squaredNorm() < 1e-12,
        "boundary cost not dropped where the filter support is unobserved");

  // 착지 2x2 블록이 미관측 셀을 포함하면 높이 조회는 여전히 거부한다.
  const Eigen::Vector2d on_hole = CellCenter(holed, 14, 7);
  Check(!partial.IsQueryable(on_hole),
        "unobserved landing block must stay unqueryable");
  bool threw = false;
  try {
    partial.Sample(on_hole);
  } catch (const TerrainDomainError&) {
    threw = true;
  }
  Check(threw, "sampling an unobserved landing block did not throw");

  // 관측 지형 높이(단차)는 보존된다: 후보 검색이 우반부 단에서 0.1을 되돌린다.
  const Eigen::Vector2d nominal = CellCenter(holed, 10, 16);
  const Eigen::Vector3d found = partial.FindFoothold(
      nominal, nominal.array() - 0.10, nominal.array() + 0.10);
  Check(std::abs(found.z() - 0.1) < 1e-6, "observed step height was erased");
}
}  // namespace

int main() {
  try {
    Run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "terrain_validity_test failed: %s\n", error.what());
    return 1;
  }
  std::printf("terrain_validity_test passed\n");
  return 0;
}
