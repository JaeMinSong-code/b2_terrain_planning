/**
 * @file terrain.cpp
 * @brief 유효성 보존 높이맵 필터와 일관된 보간 미분·착지 후보 검색.
 * @version 0.0.15
 * @update 2026-11-05: 착지점 단계·지도 관측·비용 진단을 추가하고 제어 계산과 분리함.
 * @update 2026-11-04: 앞/뒤 좌우 발 쌍의 Y 간격에만 근접 지수 비용을 적용함.
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

// 원본은 보존하고 연결된 미관측 영역만 그 경계의 최소 관측 높이로 채운다(IV-A).
void Inpaint(HeightMap* map) {
  const int rows = map->heights.rows(), cols = map->heights.cols();
  Eigen::ArrayXXi visited = Eigen::ArrayXXi::Zero(rows, cols);
  constexpr int dx[] = {-1, 1, 0, 0};
  constexpr int dy[] = {0, 0, -1, 1};
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      if (map->valid(row, col) || visited(row, col)) continue;
      std::vector<std::pair<int, int>> cells{{row, col}};
      visited(row, col) = 1;
      double minimum = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < cells.size(); ++i) {
        const auto [r, c] = cells[i];
        for (int d = 0; d < 4; ++d) {
          const int y = r + dy[d], x = c + dx[d];
          if (y < 0 || x < 0 || y >= rows || x >= cols) continue;
          if (map->valid(y, x)) minimum = std::min(minimum, map->heights(y, x));
          else if (!visited(y, x)) {
            visited(y, x) = 1;
            cells.emplace_back(y, x);
          }
        }
      }
      if (!std::isfinite(minimum)) continue;
      for (const auto& [r, c] : cells) {
        map->heights(r, c) = minimum;
        map->valid(r, c) = 1;
      }
    }
  }
}

Eigen::MatrixXd Median(const Eigen::MatrixXd& source, int radius) {
  Eigen::MatrixXd result(source.rows(), source.cols());
  std::vector<double> values;
  values.reserve((2 * radius + 1) * (2 * radius + 1));
  for (int row = 0; row < source.rows(); ++row) {
    for (int col = 0; col < source.cols(); ++col) {
      values.clear();
      for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
          values.push_back(source(std::clamp(row + y, 0, int(source.rows()) - 1),
                                  std::clamp(col + x, 0, int(source.cols()) - 1)));
        }
      }
      const auto middle = values.begin() + values.size() / 2;
      std::nth_element(values.begin(), middle, values.end());
      result(row, col) = *middle;
    }
  }
  return result;
}

Eigen::MatrixXd Gaussian(const Eigen::MatrixXd& source, double sigma, double resolution) {
  const int radius = std::max(1, int(std::ceil(3 * sigma / resolution - 1e-12)));
  std::vector<double> weights(2 * radius + 1);
  double sum = 0;
  for (int d = -radius; d <= radius; ++d) {
    weights[d + radius] = std::exp(-0.5 * std::pow(d * resolution / sigma, 2));
    sum += weights[d + radius];
  }
  for (double& w : weights) w /= sum;
  Eigen::MatrixXd horizontal = Eigen::MatrixXd::Zero(source.rows(), source.cols());
  Eigen::MatrixXd result = horizontal;
  for (int r = 0; r < source.rows(); ++r) {
    for (int c = 0; c < source.cols(); ++c) {
      for (int d = -radius; d <= radius; ++d) {
        horizontal(r, c) += weights[d + radius] * source(r, std::clamp(c + d, 0, int(source.cols()) - 1));
      }
    }
  }
  for (int r = 0; r < source.rows(); ++r) {
    for (int c = 0; c < source.cols(); ++c) {
      for (int d = -radius; d <= radius; ++d) {
        result(r, c) += weights[d + radius] * horizontal(std::clamp(r + d, 0, int(source.rows()) - 1), c);
      }
    }
  }
  return result;
}

// 논문 IV-B의 5점 중앙차분. 보간 Jacobian은 Bilinear의 해석 미분을 사용한다.
void Derivatives(const Eigen::MatrixXd& source, double resolution,
                 Eigen::MatrixXd* gx, Eigen::MatrixXd* gy, Eigen::MatrixXd* curvature) {
  gx->resize(source.rows(), source.cols());
  gy->resize(source.rows(), source.cols());
  if (curvature) curvature->resize(source.rows(), source.cols());
  for (int r = 0; r < source.rows(); ++r) {
    for (int c = 0; c < source.cols(); ++c) {
      auto at = [&](int y, int x) {
        return source(std::clamp(r + y, 0, int(source.rows()) - 1),
                      std::clamp(c + x, 0, int(source.cols()) - 1));
      };
      (*gx)(r, c) = (at(0, -2) - 8 * at(0, -1) + 8 * at(0, 1) - at(0, 2)) / (12 * resolution);
      (*gy)(r, c) = (at(-2, 0) - 8 * at(-1, 0) + 8 * at(1, 0) - at(2, 0)) / (12 * resolution);
      if (curvature) {
        const double xx = (-at(0, -2) + 16 * at(0, -1) - 30 * at(0, 0) + 16 * at(0, 1) - at(0, 2)) / (12 * resolution * resolution);
        const double yy = (-at(-2, 0) + 16 * at(-1, 0) - 30 * at(0, 0) + 16 * at(1, 0) - at(2, 0)) / (12 * resolution * resolution);
        (*curvature)(r, c) = std::pow(std::max(0.0, xx), 2) + std::pow(std::max(0.0, yy), 2);
      }
    }
  }
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
  for (double value : {settings.smoothing_sigma,
                       settings.contact_tolerance,
                       settings.ground_weight, settings.collision_distance,
                       settings.foot_separation_distance_y, settings.foot_separation_scale}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("invalid terrain settings");
    }
  }
  for (double value : {settings.edge_weight, settings.collision_weight, settings.foot_separation_weight,
                       settings.occlusion_weight,
                       settings.curvature_weight, settings.virtual_height_weight, settings.body_alignment_weight}) {
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
      if (map.valid(row, col) && std::isfinite(map.heights(row, col))) {
        known(row, col) = 1;
        heights(row, col) = map.heights(row, col);
      }
    }
  }
  processed_ = map_;
  processed_.heights = heights;
  processed_.valid = known;
  occlusion_ = 1.0 - known.cast<double>().matrix().array();
  curvature_cost_ = Eigen::MatrixXd::Zero(rows, cols);
  if (settings.pipeline_enabled && known.any()) {
    if (settings.median_iterations < 0 || settings.median_iterations > 5 ||
        !std::isfinite(settings.virtual_floor_sigma) || settings.virtual_floor_sigma <= 0 ||
        !std::isfinite(settings.virtual_floor_radius) || settings.virtual_floor_radius < 0) {
      throw std::invalid_argument("invalid terrain processing settings");
    }
    Inpaint(&processed_);
    for (int i = 0; i < settings.median_iterations; ++i) {
      processed_.heights = Median(processed_.heights, 1);
    }
    smooth_ = Gaussian(processed_.heights, settings.smoothing_sigma, map.resolution);
    const int wide_radius = std::min(std::min(rows, cols) - 1,
        std::max(1, int(std::ceil(settings.virtual_floor_radius / map.resolution - 1e-12))));
    const auto median = Median(processed_.heights, wide_radius);
    const Eigen::ArrayXXi mask = ((processed_.heights - median).array().abs() > 1e-9).cast<int>();
    Eigen::MatrixXd dilated = processed_.heights;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        for (int y = std::max(0, r - wide_radius); y <= std::min(rows - 1, r + wide_radius); ++y) {
          for (int x = std::max(0, c - wide_radius); x <= std::min(cols - 1, c + wide_radius); ++x) {
            if (mask(y, x)) dilated(r, c) = std::max(dilated(r, c), processed_.heights(y, x));
          }
        }
      }
    }
    virtual_ = processed_;
    virtual_.heights = Gaussian(dilated, settings.virtual_floor_sigma, map.resolution);
    Derivatives(processed_.heights, map.resolution, &gradients_[0], &gradients_[1], nullptr);
    Derivatives(smooth_, map.resolution, &gradients_[2], &gradients_[3], &curvature_cost_);
    Derivatives(virtual_.heights, map.resolution, &virtual_gradients_[0], &virtual_gradients_[1], nullptr);
    height_known_ = processed_.valid;
    usable_ = processed_.valid;
    return;
  }
  virtual_ = processed_;
  smooth_ = processed_.heights;
  virtual_gradients_[0] = Eigen::MatrixXd::Zero(rows, cols);
  virtual_gradients_[1] = Eigen::MatrixXd::Zero(rows, cols);
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
  // 착지 높이 조회 유효성: 해당 셀의 관측 높이가 알려졌는가. 주변 미관측과 무관하다.
  height_known_ = known;
  usable_ = Eigen::ArrayXXi::Zero(rows, cols);
  const int halo = radius + 1;
  // 평활·경계 비용 유효성: 필터와 중앙차분 지원 영역 전체가 known인 내부만.
  for (int row = halo; row < rows - halo; ++row) {
    for (int col = halo; col < cols - halo; ++col) {
      usable_(row, col) = known.block(row - halo, col - halo,
                                     2 * halo + 1, 2 * halo + 1).all();
    }
  }
}

TerrainSample TerrainModel::Sample(const Eigen::Vector2d& xy, TerrainLayer layer) const {
  if (!xy.allFinite()) throw TerrainDomainError("nonfinite terrain query");
  const Eigen::Vector2d index = (xy - map_.origin) / map_.resolution;
  if (index.x() < 0 || index.y() < 0 || index.x() >= map_.heights.cols() - 1 ||
      index.y() >= map_.heights.rows() - 1) {
    throw TerrainDomainError("terrain query outside map");
  }
  const int col = static_cast<int>(std::floor(index.x()));
  const int row = static_cast<int>(std::floor(index.y()));
  // 처리 지도의 보간 네 셀에 높이가 있으면 조회한다. 원본 관측 여부는
  // 별도 occlusion으로 보존하며 pipeline OFF만 관측 전용 계약을 사용한다.
  if (!height_known_.block<2, 2>(row, col).all()) {
    throw TerrainDomainError("terrain query has unknown landing height");
  }
  const double x = index.x() - col;
  const double y = index.y() - row;
  TerrainSample result;
  const Eigen::MatrixXd* heights = &processed_.heights;
  if (layer == TerrainLayer::kVirtualFloor) heights = &virtual_.heights;
  const auto height = Bilinear(*heights, row, col, x, y, map_.resolution);
  result.height = height.value;
  result.gradient = height.gradient;
  result.occlusion = Bilinear(occlusion_, row, col, x, y, map_.resolution).value;
  result.virtual_height = Bilinear(virtual_.heights, row, col, x, y, map_.resolution).value;
  result.curvature_cost = Bilinear(curvature_cost_, row, col, x, y, map_.resolution).value;
  if (usable_.block<2, 2>(row, col).all()) {
    for (int axis = 0; axis < 4; ++axis) {
      const Eigen::MatrixXd* gradient = &gradients_[axis];
      if (layer == TerrainLayer::kVirtualFloor) gradient = &virtual_gradients_[axis % 2];
      const auto edge = Bilinear(*gradient, row, col, x, y, map_.resolution);
      result.edge[axis] = edge.value;
      result.edge_jacobian.row(axis) = edge.gradient.transpose();
    }
  }
  return result;
}

double TerrainModel::LandingHeight(const Eigen::Vector2d& xy, TerrainLayer layer) const {
  return GroundToFootCenterZ(Sample(xy, layer).height, settings_.foot_radius);
}

TerrainResidual TerrainModel::Residual(const Eigen::Vector3d& foot, TerrainLayer layer) const {
  const auto sample = Sample(foot.head<2>(), layer);
  TerrainResidual result;
  result.values[0] = std::sqrt(settings_.ground_weight) *
      (foot.z() - GroundToFootCenterZ(sample.height, settings_.foot_radius));
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

CollisionResidual TerrainModel::FootSeparation(
    const Eigen::Vector2d& delta, double heading) const {
  CollisionResidual result;
  if (settings_.foot_separation_weight == 0.0) return result;
  const Eigen::Vector2d forward(std::cos(heading), std::sin(heading));
  const Eigen::Vector2d lateral(-forward.y(), forward.x());
  const double y = lateral.dot(delta);
  // 지도 셀 중심과 연속 SQP 좌표에서 같은 거리장을 평가한다.
  // 중심에서 미분이 유한하도록 1µm만 정규화한다.
  const double distance = std::sqrt(y * y + 1e-12);
  const double exponent = (settings_.foot_separation_distance_y - distance) /
                          (2.0 * settings_.foot_separation_scale);
  // r² = weight * exp((distance_y - abs_y) / scale).
  // 매우 가파른 설정도 overflow 없이 밀어내도록 큰 지수는 접선으로 연장한다.
  constexpr double kTail = 10.0;
  double value = 0.0;
  double derivative = 0.0;
  if (exponent > kTail) {
    derivative = std::exp(kTail);
    value = derivative * (1.0 + exponent - kTail);
  } else {
    value = std::exp(exponent);
    derivative = value;
  }
  const double root_weight = std::sqrt(settings_.foot_separation_weight);
  result.value = root_weight * value;
  result.gradient = -root_weight * derivative /
      (2.0 * settings_.foot_separation_scale * distance) *
      (y * lateral);
  return result;
}

TerrainPointDiagnostic TerrainModel::Inspect(const Eigen::Vector2d& xy,
                                            TerrainLayer layer) const {
  TerrainPointDiagnostic out;
  if (!xy.allFinite()) return out;
  const Eigen::Vector2d v = (xy - map_.origin) / map_.resolution;
  if (v.x() < 0 || v.y() < 0 || v.x() >= map_.heights.cols() - 1 ||
      v.y() >= map_.heights.rows() - 1) return out;
  out.lower_cell = v.array().floor().cast<int>();
  const double x = v.x() - out.lower_cell.x();
  const double y = v.y() - out.lower_cell.y();
  out.weights = {(1 - x) * (1 - y), x * (1 - y), (1 - x) * y, x * y};
  for (int k = 0; k < 4; ++k) {
    const int col = out.lower_cell.x() + k % 2;
    const int row = out.lower_cell.y() + k / 2;
    out.observed[k] = map_.valid(row, col) && std::isfinite(map_.heights(row, col));
    out.input_heights[k] = map_.heights(row, col);
    out.processed_heights[k] = processed_.heights(row, col);
    if (layer == TerrainLayer::kVirtualFloor) out.processed_heights[k] = virtual_.heights(row, col);
    out.height += out.weights[k] * out.processed_heights[k];
    if (!out.observed[k]) out.occlusion += out.weights[k];
  }
  out.valid = true;
  return out;
}

std::array<double, 7> TerrainModel::CandidateCosts(
    const Eigen::Vector2d& xy, const Eigen::Vector2d& nominal,
    const Eigen::Vector2d& direction, double backward_weight, TerrainLayer layer,
    const SupportPositions& separation_supports, double heading) const {
  const auto sample = Sample(xy, layer);
  const Eigen::Vector2d offset = xy - nominal;
  const double behind = std::min(0.0, direction.dot(offset));
  std::array<double, 7> costs{0.7 * offset.squaredNorm(), backward_weight * behind * behind,
                            settings_.edge_weight * sample.edge.squaredNorm(), 0, 0, 0, 0};
  if (settings_.pipeline_enabled && layer == TerrainLayer::kGround) {
    costs[3] = settings_.occlusion_weight * sample.occlusion;
    costs[4] = settings_.curvature_weight * sample.curvature_cost;
    costs[5] = settings_.virtual_height_weight * std::pow(sample.height - sample.virtual_height, 2);
  }
  for (const auto& support : separation_supports) {
    const auto residual = FootSeparation(xy - support, heading);
    costs[6] += residual.value * residual.value;
  }
  return costs;
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
                                  double backward_weight, TerrainLayer layer,
                                  const SupportPositions& separation_supports, double heading) const {
  const auto sample = Sample(xy, layer);
  const Eigen::Vector2d offset = xy - nominal;
  const double behind = std::min(0.0, direction.dot(offset));
  double cost = 0.7 * offset.squaredNorm() + backward_weight * behind * behind +
      settings_.edge_weight * sample.edge.squaredNorm();
  if (settings_.pipeline_enabled && layer == TerrainLayer::kGround) {
    cost += settings_.occlusion_weight * sample.occlusion +
        settings_.curvature_weight * sample.curvature_cost +
        settings_.virtual_height_weight * std::pow(sample.height - sample.virtual_height, 2);
  }
  for (const auto& support : separation_supports) {
    const auto residual = FootSeparation(xy - support, heading);
    cost += residual.value * residual.value;
  }
  return cost;
}

bool TerrainModel::IsOccupiedCell(
    const Eigen::Vector2d& xy, const SupportPositions& occupied) const {
  const Eigen::Array2d cell = ((xy - map_.origin) / map_.resolution).array() + 0.5;
  for (const auto& support : occupied) {
    const Eigen::Array2d support_cell =
        ((support - map_.origin) / map_.resolution).array() + 0.5;
    if ((cell.floor() - support_cell.floor()).abs().maxCoeff() <= 1.0) return true;
  }
  return false;
}

Eigen::Vector3d TerrainModel::FindFoothold(const Eigen::Vector2d& nominal,
                                         const Eigen::Vector2d& lower,
                                         const Eigen::Vector2d& upper,
                                         const Eigen::Vector2d& direction,
                                         double backward_weight,
                                         const std::function<bool(const Eigen::Vector3d&)>& admissible,
                                         TerrainLayer layer,
                                         const SupportPositions& occupied, double heading,
                                         const SupportPositions& separation_supports) const {
  double best = std::numeric_limits<double>::infinity();
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  bool occupied_candidate = false;
  auto consider = [&](const Eigen::Vector2d& xy) {
    if ((xy.array() < lower.array()).any() || (xy.array() > upper.array()).any() ||
        !IsQueryable(xy)) return;
    const auto sample = Sample(xy, layer);
    const Eigen::Vector3d foot(
        xy.x(), xy.y(), GroundToFootCenterZ(sample.height, settings_.foot_radius));
    if (admissible && !admissible(foot)) return;
    if (IsOccupiedCell(xy, occupied)) {
      occupied_candidate = true;
      return;
    }
    const double cost = FootholdCost(xy, nominal, direction, backward_weight, layer,
                                    separation_supports, heading);
    if (cost < best) {
      best = cost;
      result = foot;
    }
  };
  consider(nominal.cwiseMax(lower).cwiseMin(upper));
  const Eigen::Vector2i first = ((lower - map_.origin) / map_.resolution).array().ceil().cast<int>();
  const Eigen::Vector2i last = ((upper - map_.origin) / map_.resolution).array().floor().cast<int>();
  for (int row = std::max(0, first.y()); row <= std::min(int(map_.heights.rows()) - 2, last.y()); ++row) {
    for (int col = std::max(0, first.x()); col <= std::min(int(map_.heights.cols()) - 2, last.x()); ++col) {
      consider(map_.origin + map_.resolution * Eigen::Vector2d(col, row));
    }
  }
  if (!std::isfinite(best)) {
    if (occupied_candidate) throw OccupiedFootholdError("all terrain foothold candidates occupied");
    throw TerrainDomainError("no valid terrain foothold candidate");
  }
  return result;
}
}  // namespace b2_terrain_planner
