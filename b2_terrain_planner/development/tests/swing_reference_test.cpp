/**
 * @file swing_reference_test.cpp
 * @brief 전체 시간 XY 5차 연결과 지형 여유 기반 Z 보정의 경계·속도 검증.
 * @version 0.0.7
 * @update 2026-10-29: 이륙 장애물에서도 XY p/v/a가 바뀌지 않는 지형 보정을 검증함.
 * @update 2026-10-28: 32mm 구형 발의 조회 범위와 기존 스윙 경계를 검증함.
 */
#include "b2_terrain_planner/swing_reference.h"
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
using b2_terrain_planner::HeightMap;
using b2_terrain_planner::SwingReference;
constexpr double kRadius = 0.032;
constexpr double kMargin = 0.03;

HeightMap MakeMap() {
  HeightMap map;
  map.resolution = 0.01;
  map.origin = Eigen::Vector2d(-0.2, -0.2);
  map.heights = Eigen::MatrixXd::Zero(41, 101);
  map.valid = Eigen::ArrayXXi::Ones(41, 101);
  return map;
}

SwingReference MakeCurve() {
  SwingReference curve;
  curve.start = 0.2;
  curve.end = 0.55;
  curve.apex_time = 0.375;
  curve.position = Eigen::Vector3d(0.0, 0.0, kRadius);
  curve.target = Eigen::Vector3d(0.6, 0.0, kRadius);
  curve.apex_z = kRadius + 0.1;
  return curve;
}

void Shape(SwingReference* curve, const HeightMap& map) {
  curve->ShapeOverTerrain(map, kMargin, kRadius);
}

void CheckConnections(const SwingReference& curve) {
  assert(curve.Sample(curve.start).first.isApprox(curve.position, 1e-10));
  assert((curve.Sample(curve.start).second - curve.velocity).norm() < 1e-10);
  assert(curve.Sample(curve.end).first.isApprox(curve.target, 1e-10));
  assert(curve.Sample(curve.end).second.norm() < 1e-10);
  constexpr double kDt = 1e-7;
  for (double time : {curve.start, curve.apex_time, curve.end}) {
    const auto left = curve.Sample(time - kDt);
    const auto middle = curve.Sample(time);
    const auto right = curve.Sample(time + kDt);
    assert((left.first - right.first).norm() < 1e-6);
    // 비영 가속도에서는 좌우 속도가 dt에 비례해 달라진다. dt를 반으로
    // 줄일 때 차이도 수렴하는지 확인하고 아래에서 가속도 연속성을 검사한다.
    const double velocity_difference = (left.second - right.second).norm();
    const double half_difference =
        (curve.Sample(time - kDt / 2).second -
         curve.Sample(time + kDt / 2).second).norm();
    assert(half_difference <= 0.6 * velocity_difference + 1e-9);
    const Eigen::Vector3d a_left = (middle.second - left.second) / kDt;
    const Eigen::Vector3d a_right = (right.second - middle.second) / kDt;
    assert((a_left - a_right).norm() < 0.02);
  }
}

void CheckTerrain() {
  auto map = MakeMap();
  const auto original = MakeCurve();
  auto curve = original;
  Shape(&curve, map);
  assert(curve.terrain_lift < 1e-10);  // 평지에서 접지점의 가까움만으로 발을 더 들지 않는다.
  CheckConnections(curve);
  map.valid.setZero();
  Shape(&curve, map);
  assert(curve.terrain_lift == 0.0);
  map.valid.setOnes();
  map.heights.block(18, 48, 5, 5).setConstant(0.02);
  Shape(&curve, map);
  assert(curve.terrain_lift < 1e-10);  // 충분히 낮은 장애물은 기본 곡선을 유지한다.
  map.heights.block(18, 48, 5, 5).setConstant(0.12);
  Shape(&curve, map);
  assert(curve.terrain_lift > 0.049 && curve.terrain_lift < 0.09);
  const double lift = curve.terrain_lift;
  map.heights(40, 50) = 1.0;  // 경로 밖의 높은 셀은 무시한다.
  map.valid(20, 40) = 0;      // 미관측은 보정·발행 거부 조건이 아니다.
  Shape(&curve, map);
  assert(std::abs(curve.terrain_lift - lift) < 1e-12);
  CheckConnections(curve);
  for (double time = curve.start; time <= curve.end; time += 0.001) {
    const auto shaped = curve.Sample(time);
    const auto base = original.Sample(time);
    assert((shaped.first.head<2>() - base.first.head<2>()).norm() < 1e-12);
    assert((shaped.second.head<2>() - base.second.head<2>()).norm() < 1e-12);
    assert(shaped.first.z() >= base.first.z() - 1e-12);
    const double dt = 1e-6;
    if (time > curve.start + dt && time < curve.end - dt) {
      const Eigen::Vector3d numeric_velocity =
          (curve.Sample(time + dt).first - curve.Sample(time - dt).first) / (2 * dt);
      assert((numeric_velocity - shaped.second).norm() < 1e-6);
      constexpr double kAccelerationDt = 1e-7;
      const Eigen::Vector3d numeric_acceleration =
          (curve.Sample(time + kAccelerationDt).second -
           curve.Sample(time - kAccelerationDt).second) / (2 * kAccelerationDt);
      assert((numeric_acceleration - curve.SampleKinematics(time).acceleration).norm() < 1e-4);
    }
  }
  // 재조회 시 보정이 누적되지 않고, 평지에서는 해제된다.
  map.heights.setZero();
  Shape(&curve, map);
  assert(curve.terrain_lift < 1e-10);
  // 오르기/내려가기 모두 착지 XYZ와 시각을 유지한다.
  for (bool ascending : {true, false}) {
    curve = original;
    if (ascending) {
      curve.target.z() += 0.2;
    } else {
      curve.position.z() += 0.2;
    }
    curve.apex_z = std::max(curve.position.z(), curve.target.z()) + 0.1;
    for (int col = 0; col < map.heights.cols(); ++col) {
      const double x = map.origin.x() + col * map.resolution;
      double height = 0.0;
      if ((ascending && x >= 0.3) || (!ascending && x < 0.3)) height = 0.2;
      map.heights.col(col).setConstant(height);
    }
    Shape(&curve, map);
    CheckConnections(curve);
    assert(curve.start == original.start && curve.end == original.end);
  }
  curve.velocity = Eigen::Vector3d(0.02, -0.01, 0.03);
  assert((curve.Sample(curve.start).second - curve.velocity).norm() < 1e-12);
}

// 착지 전(진행률 ~0.6-0.7)에 낮은 블록이 있으면 보정이 그 위치의 여유를 실제로
// 채워야 한다. 중앙 고정 봉우리에서는 후반 보정이 소멸해 발바닥이 지면 아래로 갔다.
void CheckLateObstacle() {
  auto map = MakeMap();
  const auto original = MakeCurve();
  auto curve = original;
  for (int col = 0; col < map.heights.cols(); ++col) {
    const double x = map.origin.x() + col * map.resolution;
    if (x >= 0.46 && x <= 0.50) map.heights.col(col).setConstant(0.12);
  }
  Shape(&curve, map);
  assert(curve.terrain_lift > 0.05 && curve.terrain_lift < 0.12);
  assert(curve.lift_fall_start > 0.65);
  CheckConnections(curve);
  for (double time = curve.start; time <= curve.end; time += 0.0005) {
    const auto shaped = curve.Sample(time).first;
    assert((shaped.head<2>() -
            original.Sample(time).first.head<2>()).norm() < 1e-12);
    if (shaped.x() < 0.46 || shaped.x() > 0.50) continue;
    assert(shaped.z() - kRadius >= 0.12 - 1e-3);  // 블록 위에서 발바닥이 지면 위.
  }
  assert(curve.Sample(curve.end).first.isApprox(curve.target, 1e-10));
}

// 이륙 인접 단차에서도 지도 보정은 기본 XY의 위치·속도·가속도를 바꾸지 않는다.
void CheckTakeoffRiser() {
  auto map = MakeMap();
  auto curve = MakeCurve();
  for (int col = 0; col < map.heights.cols(); ++col) {
    const double x = map.origin.x() + col * map.resolution;
    if (x >= 0.03) map.heights.col(col).setConstant(0.15);
  }
  curve.target.z() = 0.15 + kRadius;
  curve.apex_z = curve.target.z() + 0.1;
  Shape(&curve, map);
  assert(curve.terrain_lift > 0.0);
  CheckConnections(curve);
  for (double t = curve.start; t < curve.end; t += 0.0005) {
    const auto base = curve.SampleBaseKinematics(t);
    const auto shaped = curve.SampleKinematics(t);
    assert(shaped.position.head<2>().isApprox(base.position.head<2>(), 1e-12));
    assert(shaped.velocity.head<2>().isApprox(base.velocity.head<2>(), 1e-12));
    assert(shaped.acceleration.head<2>().isApprox(base.acceleration.head<2>(), 1e-12));
  }
}

// 착지점 바로 앞 회랑 안에 다음 단차가 있는 경우. 착지 경계 구간의 부족은
// 보정을 만들지 않으므로 착지 직전 Z 목표가 폭주하지 않고 착지를 유지한다.
void CheckLandingRiser() {
  auto map = MakeMap();
  auto curve = MakeCurve();
  for (int col = 0; col < map.heights.cols(); ++col) {
    const double x = map.origin.x() + col * map.resolution;
    if (x >= 0.64) map.heights.col(col).setConstant(0.12);
  }
  Shape(&curve, map);
  assert(curve.terrain_lift < 0.02);
  CheckConnections(curve);
  double peak = 0.0;
  for (double time = curve.start; time <= curve.end; time += 0.0005) {
    peak = std::max(peak, std::abs(curve.Sample(time).second.z()));
  }
  assert(peak < 3.0);  // 하강 구간이 스캔 한 칸으로 붕괴하면 10 m/s를 넘는다.
  assert(curve.Sample(curve.end).first.isApprox(curve.target, 1e-10));
}

void CheckFailureDistance() {
  // 18:20 런의 FR 이동거리/시간. 지형 보정을 켜도 XY 이동 시간이 줄지 않는다.
  auto curve = MakeCurve();
  curve.start = 6.48;
  curve.end = 6.8;
  curve.apex_time = 6.64;
  curve.position = Eigen::Vector3d(2.1422, 0.152, -0.0809);
  curve.target = Eigen::Vector3d(1.7354, 0.134, -0.116);
  curve.apex_z = 0.0191;
  curve.terrain_lift = 0.05;
  double peak = 0.0;
  double sampled_peak = 0.0;
  for (double time = curve.start; time <= curve.end; time += 0.0001) {
    peak = std::max(peak, std::abs(curve.Sample(time).second.x()));
  }
  // 현재 실행기와 같은 20ms 선형 샘플 속도도 확인한다.
  for (int i = 0; i < 16; ++i) {
    const double time = curve.start + i * 0.02;
    const double speed = std::abs(
        (curve.Sample(time + 0.02).first.x() - curve.Sample(time).first.x()) / 0.02);
    sampled_peak = std::max(sampled_peak, speed);
  }
  assert(std::abs(peak - 1.875 * 0.4068 / 0.32) < 1e-6);
  assert(sampled_peak < 2.4);
  std::cout << "FR regression: XY peak=" << peak
            << " m/s, sampled=" << sampled_peak << " m/s\n";
  CheckConnections(curve);
}
}  // namespace

int main() {
  CheckTerrain();
  CheckLateObstacle();
  CheckTakeoffRiser();
  CheckLandingRiser();
  CheckFailureDistance();
  b2_terrain_planner::SwingReference curve;
  curve.start = .08; curve.end = .4; curve.apex_time = .24; curve.apex_z = .1;
  curve.target.x() = .04;
  for (double t : {.16, .22, .28, .34}) {
    const auto initial = curve.SampleKinematics(t);
    auto next = curve;
    next.start = t;
    next.position = initial.position;
    next.velocity = initial.velocity;
    next.acceleration = initial.acceleration;
    next.target.x() += .001;
    const auto joined = next.SampleKinematics(t);
    assert(joined.position.isApprox(initial.position, 1e-10));
    assert(joined.velocity.isApprox(initial.velocity, 1e-10));
    assert((joined.acceleration - initial.acceleration).norm() < 1e-9);
    for (double u = t; u < .4; u += .001) {
      assert(next.Sample(u).first.z() <= .100001);
      assert(std::abs(next.Sample(u).first.z() - curve.Sample(u).first.z()) < 1e-10);
    }
    assert(next.Sample(.4).first.isApprox(next.target));
    assert(next.Sample(.4).second.norm() < 1e-10);
    curve = next;
  }
}
