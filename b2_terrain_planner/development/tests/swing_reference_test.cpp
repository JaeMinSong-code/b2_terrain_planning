/**
 * @file swing_reference_test.cpp
 * @brief 일반·단차 스윙 선택과 지형 보정의 경계·미분·방향·후퇴 동작 검증.
 * @version 0.0.8
 * @update 2026-11-04: 단차 스윙 선택·회전 방향·후퇴·하강 지연 및 경계 미분을 검증함.
 * @update 2026-10-29: 이륙 장애물에서도 XY p/v/a가 바뀌지 않는 지형 보정을 검증함.
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

// 지형과 궤적을 함께 회전시켜 전진·후진·옆걸음에서 같은 선택과 후퇴를 확인한다.
void CheckStepShapes() {
  using b2_terrain_planner::StepSwingSettings;
  using ShapeKind = SwingReference::Shape;
  const StepSwingSettings settings;
  for (const Eigen::Vector2d direction : {Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0),
                                        Eigen::Vector2d(0, 1), Eigen::Vector2d(0, -1)}) {
    auto map = MakeMap();
    map.origin = Eigen::Vector2d(-0.5, -0.5);
    map.heights = Eigen::MatrixXd::Zero(101, 101);
    map.valid = Eigen::ArrayXXi::Ones(101, 101);
    const auto fill_step = [&](double edge, bool ascending) {
      for (int row = 0; row < map.heights.rows(); ++row) {
        for (int col = 0; col < map.heights.cols(); ++col) {
          const Eigen::Vector2d xy = map.origin + map.resolution * Eigen::Vector2d(col, row);
          double height = 0.0;
          if ((ascending && xy.dot(direction) >= edge) ||
              (!ascending && xy.dot(direction) < edge)) height = 0.15;
          map.heights(row, col) = height;
        }
      }
    };
    auto curve = MakeCurve();
    curve.target.head<2>() = 0.25 * direction;
    curve.target.z() += 0.15;
    curve.apex_z = curve.target.z() + 0.1;
    const auto original = curve;
    fill_step(0.06, true);  // 발 중심 6cm, 발 표면에서 2.8cm 앞의 단차.
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == ShapeKind::kStepUp);
    const double quarter = curve.start + 0.25 * (curve.end - curve.start);
    assert(std::abs((curve.Sample(quarter).first.head<2>() -
                    curve.position.head<2>()).dot(direction) + settings.retract_distance) < 1e-10);
    assert(curve.Sample(quarter).first.z() > curve.position.z());
    // 선택한 XY를 기존 지형 보정에 사용하며 종점·시각을 바꾸지 않는다.
    Shape(&curve, map);
    CheckConnections(curve);
    assert(curve.target.isApprox(original.target) && curve.end == original.end);
    // 방향 전환에도 Z 속도가 남아 후퇴 뒤 전축이 정지하지 않는다.
    double min_speed = 1e9;
    double peak_speed = 0.0;
    for (int i = 10; i <= 90; ++i) {
      const double t = curve.start + 0.01 * i * (curve.end - curve.start);
      const auto value = curve.SampleKinematics(t);
      min_speed = std::min(min_speed, value.velocity.norm());
      peak_speed = std::max(peak_speed, value.velocity.norm());
      constexpr double dt = 1e-7;
      const auto before = curve.SampleKinematics(t - dt);
      const auto after = curve.SampleKinematics(t + dt);
      assert(((after.position - before.position) / (2 * dt) - value.velocity).norm() < 1e-6);
      assert(((after.velocity - before.velocity) / (2 * dt) - value.acceleration).norm() < 1e-4);
    }
    assert(min_speed > 0.01);
    std::cout << "step-up speed=" << peak_speed << " m/s\n";
    auto off = settings;
    off.enabled = false;
    curve.SelectStepShape(map, off, kRadius);
    assert(curve.shape == ShapeKind::kNormal);
    fill_step(0.12, true);
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == ShapeKind::kNormal);  // 먼 단차에서는 교체하지 않는다.
    fill_step(0.06, true);
    map.valid.setZero();
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == ShapeKind::kNormal);
    map.valid.setOnes();

    fill_step(0.06, false);
    curve = MakeCurve();
    curve.target.head<2>() = 0.25 * direction;
    curve.position.z() += 0.15;
    curve.apex_z = curve.position.z() + 0.1;
    const auto normal_down = curve;
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == ShapeKind::kStepDown);
    Shape(&curve, map);
    CheckConnections(curve);
    const double late = curve.start + 0.8 * (curve.end - curve.start);
    assert(curve.Sample(late).first.z() > normal_down.Sample(late).first.z());
    for (int i = 1; i < 100; ++i) {
      const double t = curve.start + 0.01 * i * (curve.end - curve.start);
      const auto value = curve.SampleKinematics(t);
      assert((value.position.head<2>() - normal_down.Sample(t).first.head<2>()).norm() < 1e-12);
      constexpr double dt = 1e-7;
      const auto before = curve.SampleKinematics(t - dt);
      const auto after = curve.SampleKinematics(t + dt);
      assert(((after.position - before.position) / (2 * dt) - value.velocity).norm() < 1e-6);
      assert(((after.velocity - before.velocity) / (2 * dt) - value.acceleration).norm() < 1e-4);
    }
    // 착지점 뒤 벽이 가까우면 이륙점이 멀어도 하강형을 선택한다.
    fill_step(0.20, false);
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == ShapeKind::kStepDown);
  }
}

void CheckStepSelectionAndBoundaries() {
  const b2_terrain_planner::StepSwingSettings settings;
  auto curve = MakeCurve();
  curve.target.x() = 0.25;
  auto map = MakeMap();
  for (double height : {0.0, 0.049}) {
    map.heights.rightCols(75).setConstant(height);
    curve.SelectStepShape(map, settings, kRadius);
    assert(curve.shape == SwingReference::Shape::kNormal);
  }
  map.heights.setZero();
  map.heights(26, 20) = 0.15;  // 옆 단차는 진행 방향에 없다.
  map.heights(20, 14) = 0.15;  // 뒤 단차도 선택에서 제외한다.
  curve.SelectStepShape(map, settings, kRadius);
  assert(curve.shape == SwingReference::Shape::kNormal);
  map.heights.setZero();
  map.heights.rightCols(75).setConstant(0.15);
  curve.target.z() += 0.15;
  curve.apex_z = curve.target.z() + 0.1;
  curve.velocity = Eigen::Vector3d(0.02, -0.01, 0.03);
  curve.acceleration = Eigen::Vector3d(0.1, -0.2, 0.3);
  curve.SelectStepShape(map, settings, kRadius);
  assert(curve.shape == SwingReference::Shape::kStepUp);
  Shape(&curve, map);
  for (auto shape : {SwingReference::Shape::kStepUp, SwingReference::Shape::kStepDown}) {
    curve.shape = shape;
    const auto first = curve.SampleKinematics(curve.start);
    const auto last = curve.SampleKinematics(curve.end);
    assert((first.position - curve.position).norm() < 1e-10);
    assert((first.velocity - curve.velocity).norm() < 1e-10);
    assert((first.acceleration - curve.acceleration).norm() < 1e-9);
    assert((last.position - curve.target).norm() < 1e-10);
    assert(last.velocity.norm() < 1e-10 && last.acceleration.norm() < 1e-9);
  }
  curve.target.head<2>() = curve.position.head<2>();
  curve.SelectStepShape(map, settings, kRadius);
  assert(curve.shape == SwingReference::Shape::kNormal);
}

}  // namespace

int main() {
  CheckStepShapes();
  CheckStepSelectionAndBoundaries();
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
