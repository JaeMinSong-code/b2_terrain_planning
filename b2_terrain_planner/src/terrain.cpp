/**
 * @file terrain.cpp
 * @brief 유효성 보존 높이맵 필터와 일관된 보간 미분·착지 후보 검색.
 * @version 0.0.7
 * @update 2026-10-14: 모서리 비용을 유지하고 주변 평탄성·기울기 하드 판정을 제거함.
 * @update 2026-10-14: 지지면 후보에 호출자가 제공한 도달성 조건을 적용함.
 */
#include "b2_terrain_planner/terrain.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace b2_terrain_planner {
namespace {
struct Interpolation {
  double value;
  Eigen::Vector2d gradient;
};

Interpolation Bilinear(const Eigen::MatrixXd& data, int row, int col,
                       double x, double y, double resolution) {
  const double a = data(row, col);
  const double b = data(row, col + 1);
  const double c = data(row + 1, col);
  const double d = data(row + 1, col + 1);
  Interpolation result;
  result.value = (1 - y) * ((1 - x) * a + x * b) +
                 y * ((1 - x) * c + x * d);
  result.gradient << ((1 - y) * (b - a) + y * (d - c)) / resolution,
      ((1 - x) * (c - a) + x * (d - b)) / resolution;
  return result;
}
}  // namespace

TerrainModel::TerrainModel(const HeightMap& map,
                           const TerrainSettings& settings)
    : map_(map), settings_(settings) {
  const int rows = map.heights.rows();
  const int cols = map.heights.cols();
  if (rows < 3 || cols < 3 || rows != map.valid.rows() ||
      cols != map.valid.cols() || rows != map.variance.rows() ||
      cols != map.variance.cols() || !map.origin.allFinite() ||
      !std::isfinite(map.resolution) || map.resolution <= 0 || map.frame.empty()) {
    throw std::invalid_argument("invalid terrain geometry");
  }
  for (double value : {settings.smoothing_sigma, settings.max_variance,
                       settings.contact_tolerance,
                       settings.ground_weight, settings.collision_distance}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("invalid terrain settings");
    }
  }
  for (double value : {settings.edge_weight, settings.collision_weight}) {
    if (!std::isfinite(value) || value < 0) {
      throw std::invalid_argument("invalid terrain weight");
    }
  }
  const double radius_cells = std::ceil(3 * settings.smoothing_sigma / map.resolution);
  if (radius_cells > std::min(rows, cols)) {
    throw std::invalid_argument("terrain filter larger than map");
  }
  const int radius = std::max(1, static_cast<int>(radius_cells));
  Eigen::ArrayXXi known = Eigen::ArrayXXi::Zero(rows, cols);
  Eigen::MatrixXd heights = Eigen::MatrixXd::Zero(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      if (map.valid(row, col) != 0 && map.valid(row, col) != 1) {
        throw std::invalid_argument("terrain valid must contain 0 or 1");
      }
      if (map.valid(row, col) && std::isfinite(map.heights(row, col)) &&
          std::isfinite(map.variance(row, col)) && map.variance(row, col) >= 0 &&
          map.variance(row, col) <= settings.max_variance) {
        known(row, col) = 1;
        heights(row, col) = map.heights(row, col);
      }
    }
  }
  // 무효 셀의 임시 값은 필터 계산에만 사용한다. 전체 지원 영역이 알려진 곳만 조회한다.
  std::vector<double> weights(2 * radius + 1);
  double sum = 0;
  for (int offset = -radius; offset <= radius; ++offset) {
    const double distance = offset * map.resolution / settings.smoothing_sigma;
    weights[offset + radius] = std::exp(-0.5 * distance * distance);
    sum += weights[offset + radius];
  }
  for (double& weight : weights) weight /= sum;
  Eigen::MatrixXd horizontal = Eigen::MatrixXd::Zero(rows, cols);
  Eigen::MatrixXd smooth = Eigen::MatrixXd::Zero(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = radius; col < cols - radius; ++col) {
      for (int offset = -radius; offset <= radius; ++offset) {
        horizontal(row, col) += weights[offset + radius] * heights(row, col + offset);
      }
    }
  }
  for (int row = radius; row < rows - radius; ++row) {
    for (int col = radius; col < cols - radius; ++col) {
      for (int offset = -radius; offset <= radius; ++offset) {
        smooth(row, col) += weights[offset + radius] * horizontal(row + offset, col);
      }
    }
  }
  for (auto& gradient : gradients_) gradient = Eigen::MatrixXd::Zero(rows, cols);
  for (int row = 1; row < rows - 1; ++row) {
    for (int col = 1; col < cols - 1; ++col) {
      gradients_[0](row, col) = (heights(row, col + 1) - heights(row, col - 1)) /
                               (2 * map.resolution);
      gradients_[1](row, col) = (heights(row + 1, col) - heights(row - 1, col)) /
                               (2 * map.resolution);
      gradients_[2](row, col) = (smooth(row, col + 1) - smooth(row, col - 1)) /
                               (2 * map.resolution);
      gradients_[3](row, col) = (smooth(row + 1, col) - smooth(row - 1, col)) /
                               (2 * map.resolution);
    }
  }
  usable_ = Eigen::ArrayXXi::Zero(rows, cols);
  const int halo = radius + 1;
  // 필터와 중앙차분 지원 영역 전체가 유효한 내부만 사용한다.
  for (int row = halo; row < rows - halo; ++row) {
    for (int col = halo; col < cols - halo; ++col) {
      usable_(row, col) = known.block(row - halo, col - halo,
                                     2 * halo + 1, 2 * halo + 1).all();
    }
  }
}

TerrainSample TerrainModel::Sample(const Eigen::Vector2d& xy) const {
  if (!xy.allFinite()) throw TerrainDomainError("nonfinite terrain query");
  const Eigen::Vector2d index = (xy - map_.origin) / map_.resolution;
  if (index.x() < 0 || index.y() < 0 || index.x() >= map_.heights.cols() - 1 ||
      index.y() >= map_.heights.rows() - 1) {
    throw TerrainDomainError("terrain query outside map");
  }
  const int col = static_cast<int>(std::floor(index.x()));
  const int row = static_cast<int>(std::floor(index.y()));
  if (!usable_.block<2, 2>(row, col).all()) {
    throw TerrainDomainError("terrain query has unknown or uncertain support");
  }
  const double x = index.x() - col;
  const double y = index.y() - row;
  TerrainSample result;
  const auto height = Bilinear(map_.heights, row, col, x, y, map_.resolution);
  result.height = height.value;
  result.gradient = height.gradient;
  for (int axis = 0; axis < 4; ++axis) {
    const auto edge = Bilinear(gradients_[axis], row, col, x, y, map_.resolution);
    result.edge[axis] = edge.value;
    result.edge_jacobian.row(axis) = edge.gradient.transpose();
  }
  return result;
}

TerrainResidual TerrainModel::Residual(const Eigen::Vector3d& foot) const {
  const auto sample = Sample(foot.head<2>());
  TerrainResidual result;
  result.values[0] = std::sqrt(settings_.ground_weight) * (foot.z() - sample.height);
  result.values.tail<4>() = std::sqrt(settings_.edge_weight) * sample.edge;
  result.jacobian.setZero();
  result.jacobian.row(0) << -sample.gradient.x(), -sample.gradient.y(), 1.0;
  result.jacobian.row(0) *= std::sqrt(settings_.ground_weight);
  result.jacobian.bottomLeftCorner<4, 2>() =
      std::sqrt(settings_.edge_weight) * sample.edge_jacobian;
  return result;
}

CollisionResidual TerrainModel::Separation(const Eigen::Vector2d& delta) const {
  CollisionResidual result;
  const double deficit = settings_.collision_distance * settings_.collision_distance -
                         delta.squaredNorm();
  if (deficit > 0) {
    result.value = std::sqrt(settings_.collision_weight) * deficit;
    result.gradient = -2 * std::sqrt(settings_.collision_weight) * delta;
  }
  return result;
}

bool TerrainModel::IsQueryable(const Eigen::Vector2d& xy) const {
  try {
    Sample(xy);
    return true;
  } catch (const TerrainDomainError&) {
    return false;
  }
}

double TerrainModel::FootholdCost(const Eigen::Vector2d& xy,
                                  const Eigen::Vector2d& nominal,
                                  const Eigen::Vector2d& direction,
                                  double backward_weight) const {
  const Eigen::Vector2d offset = xy - nominal;
  const double behind = std::min(0.0, direction.dot(offset));
  return 0.7 * offset.squaredNorm() + backward_weight * behind * behind +
         settings_.edge_weight * Sample(xy).edge.squaredNorm();
}

Eigen::Vector3d TerrainModel::FindFoothold(const Eigen::Vector2d& nominal,
                                         const Eigen::Vector2d& lower,
                                         const Eigen::Vector2d& upper,
                                         const Eigen::Vector2d& direction,
                                         double backward_weight,
                                         const std::function<bool(const Eigen::Vector3d&)>& admissible) const {
  double best = std::numeric_limits<double>::infinity();
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  auto consider = [&](const Eigen::Vector2d& xy) {
    if ((xy.array() < lower.array()).any() || (xy.array() > upper.array()).any() ||
        !IsQueryable(xy)) return;
    const auto sample = Sample(xy);
    const Eigen::Vector3d foot(xy.x(), xy.y(), sample.height);
    if (admissible && !admissible(foot)) return;
    const double cost = FootholdCost(xy, nominal, direction, backward_weight);
    if (cost < best) {
      best = cost;
      result << xy, sample.height;
    }
  };
  consider(nominal.cwiseMax(lower).cwiseMin(upper));
  for (int row = 0; row < map_.heights.rows(); ++row) {
    for (int col = 0; col < map_.heights.cols(); ++col) {
      consider(map_.origin + map_.resolution * Eigen::Vector2d(col, row));
    }
  }
  if (!std::isfinite(best)) throw TerrainDomainError("no valid terrain foothold candidate");
  return result;
}
}  // namespace b2_terrain_planner
