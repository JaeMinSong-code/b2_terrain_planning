/**
 * @file swing_reference_test.cpp
 * @brief 부분 스윙 재계획의 위치·속도 연속성과 반복 lift 방지 검증.
 * @version 0.0.1
 * @update 2026-10-17: 절대 착지 시각과 C1 경계를 회귀 검증함.
 */
#include "b2_terrain_planner/swing_reference.h"
#undef NDEBUG
#include <cassert>
#include <cmath>
int main() {
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
