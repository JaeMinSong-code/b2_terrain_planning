/**
 * @file map_datum.h
 * @brief 관측 지도의 z 기준(datum)을 확인 지지발에 재정착한다(지형 모양 보존).
 * @version 0.0.2
 * @update 2026-10-27: 첫 측정 점프를 제거하고 0에서 변화율 제한으로 추적함.
 * @update 2026-10-27: odom z 드리프트 상쇄용 전역 오프셋 추적·적용을 분리함.
 */
#pragma once

#include <vector>
#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {

// 변화율 제한으로 datum 오프셋을 추적하는 상태. 호출 간 유지한다.
struct DatumOffsetState {
  double offset = 0.0;
  bool valid = false;
};

// diffs: 확인 지지발별 (foot_z − 관측 지도높이 − 접촉 오프셋). 접촉 오프셋(발 링크
// 중심과 접촉면의 상수 차, B2 실측 ~0.032m)은 호출자가 빼서 전달한다 — 남는 값이
// 순수 드리프트다. 2개 미만이면 직전 오프셋을 유지한다(스윙 전환·접촉 깜빡임 거부).
// 2개 이상이면 median을 목표로, |변화|를 max_rate*dt로 제한해 추적한다. 첫 측정도
// 0에서 ramp하므로(점프 없음) 측정 지연으로 쌓인 드리프트도 부드럽게 따라잡는다.
// dt·max_rate는 양수여야 한다. 상태를 갱신하고 현재 오프셋을 반환한다.
double UpdateDatumOffset(DatumOffsetState* state, const std::vector<double>& diffs,
                         double dt, double max_rate);

// 지도의 모든 유효 셀 높이를 offset만큼 수직 이동한다(상대 지형 모양 보존, 평탄화 아님).
// 이동한 셀 수를 반환한다. offset이 0이면 아무것도 하지 않는다.
int ApplyDatumOffset(HeightMap* map, double offset);

}  // namespace b2_terrain_planner
