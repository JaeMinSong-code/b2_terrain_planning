/**
 * @file support_patch.cpp
 * @brief 관측/미관측 셀을 접촉 발 기준의 국소 높이로 보정한다.
 * @version 0.0.1
 * @update 2026-10-17: 겹친 영역은 가장 가까운 접촉 발을 사용하고 외부 셀은 유지함.
 */
#include "b2_terrain_planner/support_patch.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace b2_terrain_planner {
double SupportPatchHalfWidth(double requested, double resolution, double sigma) {
  if (!std::isfinite(requested) || requested <= 0 || !std::isfinite(resolution) ||
      resolution <= 0 || !std::isfinite(sigma) || sigma <= 0) {
    throw std::invalid_argument("invalid support patch geometry");
  }
  // TerrainModel과 동일한 Gaussian 반경 + 중앙차분 1셀 + bilinear 1셀.
  const double radius = std::max(1.0, std::ceil(3 * sigma / resolution));
  return std::max(requested, (radius + 2) * resolution);
}

int ApplySupportPatches(HeightMap* map,
                        const std::array<Eigen::Vector3d, 4>& feet,
                        const std::array<bool, 4>& contacts, double half_width) {
  if (map == nullptr || !std::isfinite(half_width) || half_width <= 0 ||
      !std::isfinite(map->resolution) || map->resolution <= 0 ||
      !map->origin.allFinite() || map->heights.rows() != map->valid.rows() ||
      map->heights.cols() != map->valid.cols() ||
      map->heights.rows() != map->variance.rows() ||
      map->heights.cols() != map->variance.cols()) {
    throw std::invalid_argument("invalid support patch map");
  }
  for (int leg = 0; leg < 4; ++leg) {
    if (contacts[leg] && !feet[leg].allFinite()) {
      throw std::invalid_argument("nonfinite support foot");
    }
  }
  int patched = 0;
  for (int row = 0; row < map->heights.rows(); ++row) {
    for (int col = 0; col < map->heights.cols(); ++col) {
      const Eigen::Vector2d xy = map->origin + map->resolution * Eigen::Vector2d(col, row);
      int nearest = -1;
      double best = std::numeric_limits<double>::infinity();
      for (int leg = 0; leg < 4; ++leg) {
        if (!contacts[leg]) continue;
        const Eigen::Vector2d delta = xy - feet[leg].head<2>();
        if (delta.cwiseAbs().maxCoeff() > half_width + 1e-12) continue;
        if (delta.squaredNorm() < best) {
          best = delta.squaredNorm();
          nearest = leg;
        }
      }
      if (nearest < 0) continue;
      map->heights(row, col) = feet[nearest].z();
      map->valid(row, col) = 1;
      // 센서 분산이 아닌 이번 가정으로 고정한 국소 평면. 원본 지도에는 쓰지 않는다.
      map->variance(row, col) = 0.0;
      ++patched;
    }
  }
  return patched;
}
}  // namespace b2_terrain_planner
