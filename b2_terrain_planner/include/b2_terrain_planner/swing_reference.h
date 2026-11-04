/**
 * @file swing_reference.h
 * @brief 일반·단차 회피 스윙 스플라인 선택과 지형 부족 구간의 Z 보정.
 * @version 0.0.9
 * @update 2026-11-04: 발 주변 단차에 따라 둥근 상승·하강 스플라인을 선택하고 경계 p/v/a를 보존함.
 * @update 2026-10-29: 지형 기반 XY 후퇴와 전용 재조회를 제거하고 기본 XY·Z 보정을 유지함.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Core>

#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
// 탐색 거리는 발 표면 기준이며 후퇴량은 진행률 0.25에서의 이륙점 기준 거리다.
struct StepSwingSettings {
  bool enabled = true;
  double proximity = 0.05;
  double height_threshold = 0.05;
  double retract_distance = 0.05;
};

struct SwingReference {
  enum class Shape { kNormal, kStepUp, kStepDown };
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
  // Z 보정: 부족 최대값과 부족이 나타난 진행률 구간(유지 구간 양 끝).
  Shape shape = Shape::kNormal;
  Eigen::Vector2d retreat = Eigen::Vector2d::Zero();
  double rounded_height = 0.0;
  double terrain_lift = 0.0;
  double lift_rise_end = 0.5;
  double lift_fall_start = 0.5;

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

  // sin⁴ 봉우리와 진행률에 대한 1·2차 미분. 양 끝의 세 값은 0이다. 요구 여유를
  // 출발·착지에서 0으로 줄이는 가중치로만 사용한다.
  static Eigen::Vector3d Bump(double phase) {
    const double u = std::clamp(phase, 0.0, 1.0);
    constexpr double kPi = 3.14159265358979323846;
    const double sine = std::sin(kPi * u);
    const double cosine = std::cos(kPi * u);
    const double square = sine * sine;
    return {square * square, 4.0 * kPi * square * sine * cosine,
            4.0 * kPi * kPi * (3.0 * square * cosine * cosine - square * square)};
  }

  // 5차 smoothstep과 입력에 대한 1·2차 미분. 양 끝에서 (0,0,0)과 (1,0,0)이다.
  static Eigen::Vector3d SmoothStep(double u) {
    const double c = std::clamp(u, 0.0, 1.0);
    const double c2 = c * c;
    return {c2 * c * (10.0 - 15.0 * c + 6.0 * c2),
            30.0 * c2 * (c - 1.0) * (c - 1.0),
            60.0 * c * (2.0 * c - 1.0) * (c - 1.0)};
  }

  // 상승[0,rise]→유지[rise,hold]→하강[hold,clear] 모양과 진행률 1·2차 미분.
  // 시작·clear 밖의 세 값은 0이고 구간 경계는 C2로 연결된다.
  static Eigen::Vector3d HoldBump(double phase, double rise, double hold,
                                  double clear) {
    if (phase <= 0.0 || phase >= clear) return Eigen::Vector3d::Zero();
    if (phase < rise) {
      const auto value = SmoothStep(phase / rise);
      return {value[0], value[1] / rise, value[2] / (rise * rise)};
    }
    if (phase <= hold) return {1.0, 0.0, 0.0};
    const double width = clear - hold;
    const auto value = SmoothStep((clear - phase) / width);
    return {value[0], -value[1] / width, value[2] / (width * width)};
  }

  // 양 끝 p/v/a가 0인 다항 봉우리와 진행률 미분. 정지 경유점 없이 둥근
  // 스플라인을 만들며, (3,5)는 초반 후퇴, (3,3)은 전체 구간의 상승에 쓴다.
  static Eigen::Vector3d PolynomialBump(double phase, int left, int right) {
    const double u = std::clamp(phase, 0.0, 1.0);
    const double v = 1.0 - u;
    const double peak = static_cast<double>(left) / (left + right);
    const double scale = 1.0 / (std::pow(peak, left) * std::pow(1.0 - peak, right));
    const double p = std::pow(u, left) * std::pow(v, right);
    const double d = left * std::pow(u, left - 1) * std::pow(v, right) -
                     right * std::pow(u, left) * std::pow(v, right - 1);
    const double dd = left * (left - 1) * std::pow(u, left - 2) * std::pow(v, right) -
        2 * left * right * std::pow(u, left - 1) * std::pow(v, right - 1) +
        right * (right - 1) * std::pow(u, left) * std::pow(v, right - 2);
    return scale * Eigen::Vector3d(p, d, dd);
  }

  Kinematics SampleRoundedKinematics(double time) const {
    Kinematics sample;
    const double duration = end - start;
    if (duration <= 1e-9) {
      sample.position = target;
      return sample;
    }
    const double phase = std::clamp((time - start) / duration, 0.0, 1.0);
    for (int axis = 0; axis < 3; ++axis) {
      const auto value = Quintic(position[axis], velocity[axis], acceleration[axis],
                                 target[axis], duration, time - start);
      sample.position[axis] = value[0];
      sample.velocity[axis] = value[1];
      sample.acceleration[axis] = value[2];
    }
    if (shape == Shape::kStepUp) {
      const auto pull = PolynomialBump(phase, 3, 5);
      sample.position.head<2>() -= retreat * pull[0];
      sample.velocity.head<2>() -= retreat * pull[1] / duration;
      sample.acceleration.head<2>() -= retreat * pull[2] / (duration * duration);
    }
    Eigen::Vector3d lift = rounded_height * PolynomialBump(phase, 3, 3);
    if (shape == Shape::kStepDown) {
      // 하강 진행률을 s에서 s³로 옮긴다. XY 이동 시간은 압축하지 않으며
      // 시작의 측정 v/a와 착지의 v=a=0을 그대로 보존한다.
      const auto early = SmoothStep(phase);
      const double square = phase * phase;
      const auto late = SmoothStep(square * phase);
      const Eigen::Vector3d delayed(late[0], 3.0 * square * late[1],
          9.0 * square * square * late[2] + 6.0 * phase * late[1]);
      lift += (target.z() - position.z()) * (delayed - early);
    }
    sample.position.z() += lift[0];
    sample.velocity.z() += lift[1] / duration;
    sample.acceleration.z() += lift[2] / (duration * duration);
    return sample;
  }

  // 발 중심 앞의 좁은 반원에서 높이 차만 조회한다. 링크 충돌 검사가 아니며
  // 유효 셀이 없으면 일반 궤적을 유지한다. 지면 기준에 발 반경은 한 번만 뺀다.
  bool NearbyStep(const HeightMap& map, const Eigen::Vector3d& foot,
                  const Eigen::Vector2d& direction, double sign,
                  const StepSwingSettings& settings, double foot_radius) const {
    const double radius = foot_radius + settings.proximity;
    const Eigen::Vector2d index = (foot.head<2>() - map.origin) / map.resolution;
    const double cells = radius / map.resolution;
    // 지도 밖의 먼 좌표를 정수로 변환하지 않는다.
    if (index.x() + cells < 0 || index.y() + cells < 0 ||
        index.x() - cells > map.heights.cols() - 1 ||
        index.y() - cells > map.heights.rows() - 1) return false;
    const int x0 = static_cast<int>(std::max(0.0, std::floor(index.x() - cells)));
    const int y0 = static_cast<int>(std::max(0.0, std::floor(index.y() - cells)));
    const int x1 = static_cast<int>(std::min<double>(
        map.heights.cols() - 1, std::ceil(index.x() + cells)));
    const int y1 = static_cast<int>(std::min<double>(
        map.heights.rows() - 1, std::ceil(index.y() + cells)));
    const double ground = foot.z() - foot_radius;
    for (int row = y0; row <= y1; ++row) {
      for (int col = x0; col <= x1; ++col) {
        if (!map.valid(row, col) || !std::isfinite(map.heights(row, col))) continue;
        const Eigen::Vector2d delta = map.origin +
            map.resolution * Eigen::Vector2d(col, row) - foot.head<2>();
        const double along = delta.dot(direction);
        const double across = std::abs(delta.x() * direction.y() -
                                       delta.y() * direction.x());
        if (along <= 0.0 || delta.squaredNorm() > radius * radius ||
            across > foot_radius + 0.5 * map.resolution) continue;
        if (sign * (map.heights(row, col) - ground) >= settings.height_threshold) {
          return true;
        }
      }
    }
    return false;
  }

  // 이륙 전 후보를 만들 때 한 번 선택한다. 단차 선택 실패는 계획 거부가 아니라
  // 기존 스윙 사용이다. 최적화의 착지점·위상·실행 중 곡선은 바꾸지 않는다.
  void SelectStepShape(const HeightMap& map, const StepSwingSettings& settings,
                       double foot_radius) {
    shape = Shape::kNormal;
    retreat.setZero();
    rounded_height = 0.0;
    if (!settings.enabled || end <= start || !std::isfinite(map.resolution) ||
        map.resolution <= 0.0 || map.heights.size() == 0 ||
        map.heights.rows() != map.valid.rows() || map.heights.cols() != map.valid.cols() ||
        !map.origin.allFinite() || !position.allFinite() || !target.allFinite()) return;
    const Eigen::Vector2d travel = target.head<2>() - position.head<2>();
    if (travel.norm() < 1e-6) return;
    const Eigen::Vector2d direction = travel.normalized();
    if (target.z() < position.z() - settings.height_threshold) {
      if (NearbyStep(map, position, direction, -1.0, settings, foot_radius) ||
          NearbyStep(map, target, -direction, 1.0, settings, foot_radius)) {
        shape = Shape::kStepDown;
        rounded_height = std::max(0.0, apex_z - position.z());
      }
    } else if (NearbyStep(map, position, direction, 1.0, settings, foot_radius)) {
      shape = Shape::kStepUp;
      rounded_height = std::max(0.0, apex_z - 0.5 * (position.z() + target.z()));
      constexpr double kPullPhase = 0.25;
      Eigen::Vector2d early;
      for (int axis = 0; axis < 2; ++axis) {
        early[axis] = Quintic(position[axis], velocity[axis], acceleration[axis],
                              target[axis], end - start, kPullPhase * (end - start))[0];
      }
      const double pull = std::max(0.0, settings.retract_distance +
          (early - position.head<2>()).dot(direction));
      retreat = direction * pull / PolynomialBump(kPullPhase, 3, 5)[0];
    }
  }

  // XY는 스윙 전체 시간을 사용한다. Z만 정점을 경유하는 두 5차 구간이다.
  Kinematics SampleBaseKinematics(double time) const {
    if (shape != Shape::kNormal) return SampleRoundedKinematics(time);
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
    if (end <= start) return sample;
    const double duration = end - start;
    const double phase = (time - start) / duration;
    if (terrain_lift > 0.0) {
      const auto lift = HoldBump(phase, lift_rise_end, lift_fall_start, 1.0);
      sample.position.z() += terrain_lift * lift[0];
      sample.velocity.z() += terrain_lift * lift[1] / duration;
      sample.acceleration.z() += terrain_lift * lift[2] / (duration * duration);
    }
    return sample;
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> Sample(double time) const {
    const auto sample = SampleKinematics(time);
    return {sample.position, sample.velocity};
  }

  // 경계 부족을 그대로 반영하면 Z 상승·하강 구간이 지나치게 짧아진다.
  // 기존 경계 제외를 유지하며 선택한 XY 경로에 추가 우회를 넣지 않는다.
  static constexpr double kLiftEdgePhase = 0.15;

  // 기본 XY 경로에서 Z 부족 최대값과 유지 구간을 구한다.
  void ScanDeficits(const HeightMap& map, double clearance, double foot_radius,
                    int intervals, double corridor) {
    terrain_lift = 0.0;
    double first_deficit = 1.0;
    double last_deficit = 0.0;
    for (int i = 1; i < intervals; ++i) {
      const double phase = static_cast<double>(i) / intervals;
      Eigen::Vector3d base =
          SampleBaseKinematics(start + phase * (end - start)).position;
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
          if (deficit <= 0.0) continue;
          if (phase < kLiftEdgePhase || phase > 1.0 - kLiftEdgePhase) continue;
          terrain_lift = std::max(terrain_lift, deficit);
          first_deficit = std::min(first_deficit, phase);
          last_deficit = std::max(last_deficit, phase);
        }
      }
    }
    if (terrain_lift > 0.0) {
      lift_rise_end = first_deficit;
      lift_fall_start = last_deficit;
    } else {
      lift_rise_end = 0.5;
      lift_fall_start = 0.5;
    }
  }

  // 선택한 스플라인의 XY 경로에서 기존 상승→유지→하강 Z 보정을 적용한다.
  // 착지점·착지 시각·양 끝 p/v/a를 유지하며 발행 거부 조건을 추가하지 않는다.
  void ShapeOverTerrain(const HeightMap& map, double clearance,
                        double foot_radius) {
    terrain_lift = 0.0;
    lift_rise_end = 0.5;
    lift_fall_start = 0.5;
    if (end <= start) return;
    const double travel = (target.head<2>() - position.head<2>()).norm() +
                          2.0 * retreat.norm() +
                          (end - start) * velocity.head<2>().norm();
    const int intervals = std::max(16, static_cast<int>(
        std::ceil(2.0 * travel / map.resolution)));
    const double footprint = foot_radius;
    const double corridor = footprint + map.resolution / std::sqrt(2.0);
    ScanDeficits(map, clearance, foot_radius, intervals, corridor);
  }
};
}  // namespace b2_terrain_planner
