/**
 * @file support_patch.cpp
 * @brief 좁은 접촉 영역의 높이를 로컬 지도에 기억하고 입력 복사본에 적용한다.
 * @version 0.0.4
 * @update 2026-11-03: 호출자 없는 SupportPatchHalfWidth shim을 제거함.
 * @update 2026-10-29: 지도 지면과 발 중심의 높이 계약 및 반경 변환을 통일함.
 */
#include "b2_terrain_planner/support_patch.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace b2_terrain_planner {
int ApplySupportPatches(HeightMap* map,
                        const std::array<Eigen::Vector3d, 4>& feet,
                        const std::array<bool, 4>& contacts, double half_width,
                        Eigen::MatrixXd* stamps, double stamp_sec, double foot_radius) {
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
  // 지도 전체를 매 상태 콜백에서 훑지 않고 작은 접촉 영역의 셀만 갱신한다.
  for (int leg = 0; leg < 4; ++leg) {
    if (!contacts[leg]) continue;
    const Eigen::Vector2d low =
        (feet[leg].head<2>().array() - half_width - map->origin.array()) / map->resolution;
    const Eigen::Vector2d high =
        (feet[leg].head<2>().array() + half_width - map->origin.array()) / map->resolution;
    const int col_begin = static_cast<int>(std::clamp(std::ceil(low.x() - 1e-12),
        0.0, static_cast<double>(map->heights.cols())));
    const int row_begin = static_cast<int>(std::clamp(std::ceil(low.y() - 1e-12),
        0.0, static_cast<double>(map->heights.rows())));
    const int col_end = static_cast<int>(std::clamp(std::floor(high.x() + 1e-12),
        -1.0, static_cast<double>(map->heights.cols() - 1)));
    const int row_end = static_cast<int>(std::clamp(std::floor(high.y() + 1e-12),
        -1.0, static_cast<double>(map->heights.rows() - 1)));
    for (int row = row_begin; row <= row_end; ++row) {
      for (int col = col_begin; col <= col_end; ++col) {
        const Eigen::Vector2d xy = map->origin + map->resolution * Eigen::Vector2d(col, row);
        int nearest = -1;
        double best = std::numeric_limits<double>::infinity();
        for (int other = 0; other < 4; ++other) {
          if (!contacts[other]) continue;
          const Eigen::Vector2d delta = xy - feet[other].head<2>();
          if (delta.cwiseAbs().maxCoeff() > half_width + 1e-12) continue;
          if (delta.squaredNorm() < best) {
            best = delta.squaredNorm();
            nearest = other;
          }
        }
        if (nearest != leg) continue;
        map->heights(row, col) = FootCenterToGroundZ(feet[leg].z(), foot_radius);
        map->valid(row, col) = 1;
        map->variance(row, col) = 0.0;
        if (stamps != nullptr) (*stamps)(row, col) = stamp_sec;
        ++patched;
      }
    }
  }
  return patched;
}

void SupportPatchMemory::Reset() { memory_.reset(); }

void SupportPatchMemory::SyncLayout(const HeightMap& layout) {
  if (memory_ && memory_->frame == layout.frame &&
      memory_->resolution == layout.resolution &&
      memory_->heights.rows() == layout.heights.rows() &&
      memory_->heights.cols() == layout.heights.cols() &&
      memory_->origin.isApprox(layout.origin, 1e-12)) return;
  HeightMap next;
  next.frame = layout.frame;
  next.origin = layout.origin;
  next.resolution = layout.resolution;
  next.heights = Eigen::MatrixXd::Zero(layout.heights.rows(), layout.heights.cols());
  next.variance = next.heights;
  next.valid = Eigen::ArrayXXi::Zero(layout.heights.rows(), layout.heights.cols());
  Eigen::MatrixXd next_stamps =
      Eigen::MatrixXd::Zero(layout.heights.rows(), layout.heights.cols());
  if (memory_ && memory_->frame == layout.frame && memory_->resolution == layout.resolution) {
    for (int row = 0; row < memory_->heights.rows(); ++row) {
      for (int col = 0; col < memory_->heights.cols(); ++col) {
        if (!memory_->valid(row, col)) continue;
        const Eigen::Vector2d xy = memory_->origin + memory_->resolution * Eigen::Vector2d(col, row);
        const Eigen::Vector2d index = (xy - next.origin) / next.resolution;
        if (index.x() <= -0.5 || index.y() <= -0.5 ||
            index.x() >= next.heights.cols() - 0.5 ||
            index.y() >= next.heights.rows() - 0.5) continue;
        const int next_col = static_cast<int>(std::lround(index.x()));
        const int next_row = static_cast<int>(std::lround(index.y()));
        next.heights(next_row, next_col) = memory_->heights(row, col);
        next.valid(next_row, next_col) = 1;
        next_stamps(next_row, next_col) = stamps_(row, col);
      }
    }
  }
  memory_ = std::move(next);
  stamps_ = std::move(next_stamps);
}

void SupportPatchMemory::Observe(const std::array<Eigen::Vector3d, 4>& feet,
                                 const std::array<bool, 4>& contacts, double half_width,
                                 double stamp_sec, double foot_radius) {
  if (memory_) {
    ApplySupportPatches(&*memory_, feet, contacts, half_width, &stamps_, stamp_sec,
                        foot_radius);
  }
}

int SupportPatchMemory::ApplyTo(HeightMap* map, double now_sec, double max_age_sec) const {
  if (!memory_) return 0;
  int count = 0;
  for (int row = 0; row < map->heights.rows(); ++row) {
    for (int col = 0; col < map->heights.cols(); ++col) {
      if (!memory_->valid(row, col)) continue;
      // 옛 datum으로 기록된 화석 패치는 적용하지 않는다(수명 초과).
      if (now_sec - stamps_(row, col) > max_age_sec) continue;
      map->heights(row, col) = memory_->heights(row, col);
      map->variance(row, col) = 0.0;
      map->valid(row, col) = 1;
      ++count;
    }
  }
  return count;
}
}  // namespace b2_terrain_planner
