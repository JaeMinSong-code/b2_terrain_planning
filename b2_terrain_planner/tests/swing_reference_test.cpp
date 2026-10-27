/**
 * @file swing_reference_test.cpp
 * @brief 부분 스윙 재계획의 위치·속도 연속성과 반복 lift 방지 검증.
 * @version 0.0.2
 * @update 2026-10-27: 계단·장애물 통과와 C2 연결, 평지·미관측 동작을 검증함.
 * @update 2026-10-17: 절대 착지 시각과 C1 경계를 회귀 검증함.
 */
#include "b2_terrain_planner/swing_reference.h"
#undef NDEBUG
#include <cassert>
#include <cmath>
namespace {
using b2_terrain_planner::HeightMap;
using b2_terrain_planner::SwingReference;
constexpr double kRadius = 0.0351;
constexpr double kHalfWidth = 0.0275;
constexpr double kClearance = 0.1;

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
  curve.apex_z = kRadius + kClearance;
  return curve;
}

void Shape(SwingReference* curve, const HeightMap& map) {
  curve->ShapeOverTerrain(map, kClearance, kRadius, kHalfWidth, 0.005);
}

void CheckConnections(const SwingReference& curve) {
  assert(curve.Sample(curve.start).first.isApprox(curve.position, 1e-10));
  assert((curve.Sample(curve.start).second - curve.velocity).norm() < 1e-10);
  assert(curve.Sample(curve.end).first.isApprox(curve.target, 1e-10));
  assert(curve.Sample(curve.end).second.norm() < 1e-10);
  constexpr double kDt = 1e-7;
  for (double time : {curve.start, curve.traverse_start, curve.traverse_end, curve.end}) {
    const auto left = curve.Sample(time - kDt);
    const auto middle = curve.Sample(time);
    const auto right = curve.Sample(time + kDt);
    assert((left.first - right.first).norm() < 1e-6);
    assert((left.second - right.second).norm() < 1e-6);
    const auto a_left = (middle.second - left.second) / kDt;
    const auto a_right = (right.second - middle.second) / kDt;
    assert((a_left - a_right).norm() < 0.02);
  }
}

void CheckTerrain() {
  auto map = MakeMap();
  const auto original = MakeCurve();
  auto flat = original;
  Shape(&flat, map);
  assert(!flat.terrain_shaped);
  map.valid.setZero();
  auto unknown = original;
  Shape(&unknown, map);
  assert(!unknown.terrain_shaped);
  for (double t = original.start; t <= original.end; t += 0.001) {
    assert(flat.Sample(t).first.isApprox(original.Sample(t).first));
    assert(unknown.Sample(t).first.isApprox(original.Sample(t).first));
  }
  map.valid.setOnes();
  // 계단 오르기·내려가기. 발의 전체 바닥이 모서리를 넘는 구간을 확인한다.
  for (bool ascending : {true, false}) {
    auto curve = original;
    if (ascending) {
      curve.target.z() += 0.2;
    } else {
      curve.position.z() += 0.2;
    }
    for (int col = 0; col < map.heights.cols(); ++col) {
      const double x = map.origin.x() + col * map.resolution;
      double height = 0.0;
      if ((ascending && x >= 0.2) || (!ascending && x < 0.2)) height = 0.2;
      map.heights.col(col).setConstant(height);
    }
    Shape(&curve, map);
    assert(curve.terrain_shaped);
    assert(curve.apex_z >= 0.2 + kRadius + kClearance - 1e-12);
    assert(curve.start == original.start && curve.end == original.end);
    CheckConnections(curve);
    for (double t = curve.start; t <= curve.end; t += 0.0005) {
      const auto p = curve.Sample(t).first;
      double ground = 0.0;
      if ((ascending && p.x() + kRadius >= 0.195) ||
          (!ascending && p.x() - kRadius < 0.205)) ground = 0.2;
      assert(p.z() - kRadius >= ground - 1e-10);
      if (t <= curve.traverse_start) assert(std::abs(p.x()) < 1e-12);
      if (t >= curve.traverse_end) assert(std::abs(p.x() - curve.target.x()) < 1e-12);
    }
  }
  // 시작/착지 높이가 같아도 중간 돌출부와 발 폭 안쪽 장애물을 반영한다.
  map.heights.setZero();
  map.heights(23, 50) = 0.25;  // x=0.3, y=0.03 (발 중심선 옆).
  map.heights(40, 50) = 1.0;   // 경로 밖 장애물은 높이에 반영하지 않는다.
  map.valid(20, 40) = 0;       // 중간 미관측 셀은 발행 거부 사유가 아니다.
  auto obstacle = original;
  Shape(&obstacle, map);
  assert(obstacle.terrain_shaped);
  assert(std::abs(obstacle.apex_z - (0.25 + kRadius + kClearance)) < 1e-12);
  CheckConnections(obstacle);
  // 첫 곡선 생성 시 측정 속도 경계도 유지한다.
  obstacle = original;
  obstacle.velocity = Eigen::Vector3d(0.02, -0.01, 0.03);
  Shape(&obstacle, map);
  assert((obstacle.Sample(obstacle.start).second - obstacle.velocity).norm() < 1e-12);
}
}  // namespace

int main() {
  CheckTerrain();
  b2_terrain_planner::SwingReference curve;
  curve.start = .08; curve.end = .4; curve.apex_time = .24; curve.apex_z = .1;
  curve.target.x() = .04;
  for (double t : {.16, .22, .28, .34}) {
    const auto initial = curve.Sample(t);
    auto next = curve;
    next.start = t;
    next.position = initial.first;
    next.velocity = initial.second;
    next.target.x() += .001;
    const auto joined = next.Sample(t);
    assert(joined.first.isApprox(initial.first, 1e-10));
    assert(joined.second.isApprox(initial.second, 1e-10));
    for (double u=t; u<.4; u+=.001) assert(next.Sample(u).first.z() <= .100001);
    assert(next.Sample(.4).first.isApprox(next.target));
    assert(next.Sample(.4).second.norm() < 1e-10);
    curve = next;
  }
}
