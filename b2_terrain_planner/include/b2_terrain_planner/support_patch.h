/**
 * @file support_patch.h
 * @brief 현재 접촉 발 높이로 계획 입력 지도에 국소 평면을 덧씌운다.
 * @version 0.0.1
 * @update 2026-10-17: 접촉 마스크·최근접 발 선택과 필터 지원 영역을 반영함.
 */
#pragma once

#include <array>
#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
// half_width는 XY 각 축의 반폭이다. 필터·미분·보간에 필요한 반폭보다 작으면
// 그 최소값을 사용한다. 호출자가 복사한 계획 입력만 변경하며 이력을 누적하지 않는다.
double SupportPatchHalfWidth(double requested, double resolution, double sigma);
int ApplySupportPatches(HeightMap* map,
                        const std::array<Eigen::Vector3d, 4>& feet,
                        const std::array<bool, 4>& contacts, double half_width);
}  // namespace b2_terrain_planner
