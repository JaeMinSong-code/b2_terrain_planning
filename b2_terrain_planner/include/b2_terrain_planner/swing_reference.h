/**
 * @file swing_reference.h
 * @brief 절대 스윙 시각을 보존하는 5차 발 궤적과 지형 여유 기반 Z 보정.
 * @version 0.0.5
 * @update 2026-10-27: 부분 재연결의 시작 가속도와 해석적 가속도 샘플을 보존함.
 * @update 2026-10-27: 전체 시간의 XY 5차 곡선과 부족 여유의 sin⁴ Z 보정으로 변경함.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Core>

#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
struct SwingReference {
  struct Kinematics {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  };
  double start = 0.0;
  double end = 0.0;
  double apex_time = 0.0;
  double apex_z = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  double terrain_lift = 0.0;

  bool ActiveAt(double time) const {
    return end > start && time >= start - 1e-9 && time < end - 1e-9;
  }

  // 시작 p/v/a, 끝 p와 끝 v=a=0을 만족하는 5차 경계값 곡선.
  static Eigen::Vector3d Quintic(double p, double v, double a, double goal,
                                double duration, double elapsed) {
    if (duration <= 1e-9) return {goal, 0.0, 0.0};
    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double d = goal - p;
    const double w = duration * v;
    const double b = duration * duration * a;
    const double c3 = 10 * d - 6 * w - 1.5 * b;
    const double c4 = -15 * d + 8 * w + 1.5 * b;
    const double c5 = 6 * d - 3 * w - 0.5 * b;
    return {p + w * u + 0.5 * b * u2 + c3 * u3 + c4 * u4 + c5 * u5,
            (w + b * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / duration,
            (b + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3) /
                (duration * duration)};
  }

  // sin⁴ 봉우리와 진행률에 대한 1·2차 미분. 양 끝의 세 값은 0이다.
  static Eigen::Vector3d Bump(double phase) {
    const double u = std::clamp(phase, 0.0, 1.0);
    constexpr double kPi = 3.14159265358979323846;
    const double sine = std::sin(kPi * u);
    const double cosine = std::cos(kPi * u);
    const double square = sine * sine;
    return {square * square, 4.0 * kPi * square * sine * cosine,
            4.0 * kPi * kPi * (3.0 * square * cosine * cosine - square * square)};
  }

  // XY는 스윙 전체 시간을 사용한다. Z만 정점을 경유하는 두 5차 구간이다.
  Kinematics SampleBaseKinematics(double time) const {
    Kinematics sample;
    for (int axis = 0; axis < 3; ++axis) {
      auto value = Quintic(position[axis], velocity[axis], acceleration[axis],
                           target[axis], end - start, time - start);
      if (axis == 2 && start < apex_time - 1e-9) {
        if (time < apex_time) {
          value = Quintic(position.z(), velocity.z(), acceleration.z(), apex_z,
                          apex_time - start, time - start);
        } else {
          value = Quintic(apex_z, 0.0, 0.0, target.z(), end - apex_time,
                          time - apex_time);
        }
      }
      sample.position[axis] = value[0];
      sample.velocity[axis] = value[1];
      sample.acceleration[axis] = value[2];
    }
    return sample;
  }

  Kinematics SampleKinematics(double time) const {
    auto sample = SampleBaseKinematics(time);
    if (end > start) {
      const double duration = end - start;
      const auto bump = Bump((time - start) / duration);
      sample.position.z() += terrain_lift * bump[0];
      sample.velocity.z() += terrain_lift * bump[1] / duration;
      sample.acceleration.z() += terrain_lift * bump[2] / (duration * duration);
    }
    return sample;
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> Sample(double time) const {
    const auto sample = SampleKinematics(time);
    return {sample.position, sample.velocity};
  }

  // 생성된 XY 경로의 셀에서 기본 발바닥 궤적의 부족 여유만 읽는다.
  // 부족량의 최대값을 단일 봉우리 진폭으로 사용한다. 충돌 보장/발행 거부 검사가 아니다.
  void ShapeOverTerrain(const HeightMap& map, double clearance,
                        double foot_radius, double foot_half_width) {
    terrain_lift = 0.0;
    if (end <= start) return;
    const double travel = (target.head<2>() - position.head<2>()).norm() +
                          (end - start) * velocity.head<2>().norm();
    const int intervals = std::max(16, static_cast<int>(
        std::ceil(2.0 * travel / map.resolution)));
    const double footprint = std::hypot(foot_radius, foot_half_width);
    const double corridor = footprint + map.resolution / std::sqrt(2.0);
    for (int i = 1; i < intervals; ++i) {
      const double phase = static_cast<double>(i) / intervals;
      const auto base = SampleBaseKinematics(start + phase * (end - start)).position;
      const double margin = clearance * Bump(phase)[0];
      const int x0 = std::max(0, static_cast<int>(std::floor(
          (base.x() - corridor - map.origin.x()) / map.resolution)));
      const int y0 = std::max(0, static_cast<int>(std::floor(
          (base.y() - corridor - map.origin.y()) / map.resolution)));
      const int x1 = std::min(static_cast<int>(map.heights.cols()) - 1,
          static_cast<int>(std::ceil(
              (base.x() + corridor - map.origin.x()) / map.resolution)));
      const int y1 = std::min(static_cast<int>(map.heights.rows()) - 1,
          static_cast<int>(std::ceil(
              (base.y() + corridor - map.origin.y()) / map.resolution)));
      for (int row = y0; row <= y1; ++row) {
        for (int col = x0; col <= x1; ++col) {
          if (!map.valid(row, col) || !std::isfinite(map.heights(row, col))) {
            continue;
          }
          const Eigen::Vector2d xy =
              map.origin + map.resolution * Eigen::Vector2d(col, row);
          if ((xy - base.head<2>()).norm() > corridor) continue;
          const double deficit =
              map.heights(row, col) + foot_radius + margin - base.z();
          terrain_lift = std::max(terrain_lift, deficit);
        }
      }
    }
  }
};
}  // namespace b2_terrain_planner
