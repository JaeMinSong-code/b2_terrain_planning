/**
 * @file support_patch.h
 * @brief 작은 접촉 발 영역을 로컬 지도 안에 기억하고 계획 입력에 반영한다.
 * @version 0.0.2
 * @update 2026-10-27: 패치 자동 확장을 제거하고 이륙 후에도 접촉 셀을 보존함.
 * @update 2026-10-17: 접촉 마스크·최근접 발 선택과 필터 지원 영역을 반영함.
 */
#pragma once

#include <array>
#include <optional>
#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
// half_width는 XY 각 축의 반폭이며 필터 크기에 맞춰 자동 확대하지 않는다.
// 기존 호출 호환을 위해 resolution/sigma 인자를 유지한다.
double SupportPatchHalfWidth(double requested, double resolution, double sigma);
int ApplySupportPatches(HeightMap* map,
                        const std::array<Eigen::Vector3d, 4>& feet,
                        const std::array<bool, 4>& contacts, double half_width);
// 관측 지도와 별개인 접촉 증거. 로컬 지도 이동 시 겹치는 셀만 유지한다.
class SupportPatchMemory {
 public:
  void Reset();
  void SyncLayout(const HeightMap& layout);
  void Observe(const std::array<Eigen::Vector3d, 4>& feet,
               const std::array<bool, 4>& contacts, double half_width);
  int ApplyTo(HeightMap* map) const;

 private:
  std::optional<HeightMap> memory_;
};
}  // namespace b2_terrain_planner
