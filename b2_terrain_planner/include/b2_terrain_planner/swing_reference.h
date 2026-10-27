/**
 * @file swing_reference.h
 * @brief 재계획 시 절대 착지/정점 시각과 위치·속도를 보존하는 발 궤적.
 * @version 0.0.3
 * @update 2026-10-27: 지형 통과 높이와 C2 상승·이동·하강 곡선을 추가함.
 * @update 2026-10-17: 진행 중인 스윙의 착지점을 최적화 제약과 발 궤적에 고정함.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Core>

#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
struct SwingReference {
  double start = 0.0;
  double end = 0.0;
  double apex_time = 0.0;
  double apex_z = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = Eigen::Vector3d::Zero();

  bool terrain_shaped = false;
  double traverse_start = 0.0;
  double traverse_end = 0.0;

  // 지도 셀과 발이 지나가는 capsule의 교차를 조회한다. 미관측은 발행 거부로 쓰지 않는다.
  void ShapeOverTerrain(const HeightMap& map, double clearance,
                        double foot_radius, double foot_half_width,
                        double flat_tolerance) {
    const Eigen::Vector2d a = position.head<2>();
    const Eigen::Vector2d delta = target.head<2>() - a;
    const double length = delta.norm();
    const double footprint = std::hypot(foot_radius, foot_half_width);
    const double corridor = footprint + map.resolution / std::sqrt(2.0);
    const Eigen::Vector2d lo = a.cwiseMin(target.head<2>()).array() - corridor;
    const Eigen::Vector2d hi = a.cwiseMax(target.head<2>()).array() + corridor;
    const int x0 = std::max(0, static_cast<int>(
        std::floor((lo.x() - map.origin.x()) / map.resolution)));
    const int y0 = std::max(0, static_cast<int>(
        std::floor((lo.y() - map.origin.y()) / map.resolution)));
    const int x1 = std::min(static_cast<int>(map.heights.cols()) - 1,
        static_cast<int>(std::ceil((hi.x() - map.origin.x()) / map.resolution)));
    const int y1 = std::min(static_cast<int>(map.heights.rows()) - 1,
        static_cast<int>(std::ceil((hi.y() - map.origin.y()) / map.resolution)));
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (int row = y0; row <= y1; ++row) {
      for (int col = x0; col <= x1; ++col) {
        if (!map.valid(row, col) || !std::isfinite(map.heights(row, col))) {
          continue;
        }
        const Eigen::Vector2d xy = map.origin + map.resolution * Eigen::Vector2d(col, row);
        double along = 0.0;
        if (length > 1e-9) {
          along = std::clamp((xy - a).dot(delta) / (length * length), 0.0, 1.0);
        }
        if ((xy - a - along * delta).norm() > corridor) continue;
        low = std::min(low, map.heights(row, col));
        high = std::max(high, map.heights(row, col));
      }
    }
    // 평면·관측 부재는 기존 곡선을 유지한다. 접촉 오차 설정은 모드 선택에만 사용한다.
    if (!std::isfinite(high) || end <= start) return;
    if (high - low <= flat_tolerance &&
        std::abs(target.z() - position.z()) <= flat_tolerance) {
      return;
    }
    apex_z = std::max({position.z(), target.z(), high + foot_radius}) + clearance;
    const double up = apex_z - position.z();
    const double down = apex_z - target.z();
    const double travel = up + length + down;
    traverse_start = start + (end - start) * up / travel;
    traverse_end = traverse_start + (end - start) * length / travel;
    terrain_shaped = true;
  }

  static Eigen::Vector2d Quintic(double p, double v, double goal, double duration,
                                double elapsed) {
    if (duration <= 1e-9) return {goal, 0.0};
    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double d = goal - p;
    const double w = duration * v;
    const double c3 = 10 * d - 6 * w;
    const double c4 = -15 * d + 8 * w;
    const double c5 = 6 * d - 3 * w;
    return {p + w * u + c3 * u3 + c4 * u4 + c5 * u5,
            (w + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / duration};
  }

  bool ActiveAt(double time) const {
    return end > start && time >= start - 1e-9 && time < end - 1e-9;
  }

  static Eigen::Vector2d Hermite(double p, double v, double goal, double duration,
                                double elapsed) {
    if (duration <= 1e-9) return {goal, 0.0};
    const double s = std::clamp(elapsed / duration, 0.0, 1.0);
    const double s2 = s*s, s3 = s2*s;
    return {(2*s3-3*s2+1)*p + (s3-2*s2+s)*duration*v + (-2*s3+3*s2)*goal,
            ((6*s2-6*s)*p + (-6*s2+6*s)*goal)/duration + (3*s2-4*s+1)*v};
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> Sample(double time) const {
    Eigen::Vector3d p, v;
    if (terrain_shaped) {
      Eigen::Vector3d begin = position;
      Eigen::Vector3d finish = position;
      Eigen::Vector3d initial_velocity = velocity;
      double t0 = start;
      double t1 = traverse_start;
      finish.z() = apex_z;
      if (time >= traverse_start) {
        begin.z() = apex_z;
        finish = target;
        finish.z() = apex_z;
        initial_velocity.setZero();
        t0 = traverse_start;
        t1 = traverse_end;
      }
      if (time >= traverse_end) {
        begin = target;
        begin.z() = apex_z;
        finish = target;
        initial_velocity.setZero();
        t0 = traverse_end;
        t1 = end;
      }
      for (int axis = 0; axis < 3; ++axis) {
        const auto value = Quintic(begin[axis], initial_velocity[axis], finish[axis],
                                   t1 - t0, time - t0);
        p[axis] = value[0];
        v[axis] = value[1];
      }
      return {p, v};
    }
    for (int axis = 0; axis < 3; ++axis) {
      auto value = Hermite(position[axis], velocity[axis], target[axis], end-start, time-start);
      if (axis == 2 && start < apex_time - 1e-9) {
        if (time < apex_time) {
          value = Hermite(position.z(), velocity.z(), apex_z, apex_time-start, time-start);
        } else {
          value = Hermite(apex_z, 0.0, target.z(), end-apex_time, time-apex_time);
        }
      }
      p[axis] = value[0];
      v[axis] = value[1];
    }
    return {p, v};
  }
};
}  // namespace b2_terrain_planner
