/**
 * @file swing_reference.h
 * @brief 재계획 시 절대 착지/정점 시각과 위치·속도를 보존하는 발 궤적.
 * @version 0.0.2
 * @update 2026-10-17: 진행 중인 스윙의 착지점을 최적화 제약과 발 궤적에 고정함.
 * @update 2026-10-17: 부분 스윙의 반복 lift를 제거하고 C1 연결을 구현함.
 */
#pragma once
#include <algorithm>
#include <utility>
#include <Eigen/Core>

namespace b2_terrain_planner {
struct SwingReference {
  double start = 0.0;
  double end = 0.0;
  double apex_time = 0.0;
  double apex_z = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = Eigen::Vector3d::Zero();

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
