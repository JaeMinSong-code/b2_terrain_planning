/**
 * @file support_patch.h
 * @brief 작은 접촉 발 영역을 로컬 지도 안에 기억하고 계획 입력에 반영한다.
 * @version 0.0.4
 * @update 2026-11-03: 호출자 없는 SupportPatchHalfWidth shim을 제거함.
 * @update 2026-10-29: 지도 지면과 발 중심의 높이 계약 및 반경 변환을 통일함.
 */
#pragma once

#include <array>
#include <limits>
#include <optional>
#include "b2_terrain_planner/terrain.h"

namespace b2_terrain_planner {
// feet는 발 중심이다. 기록 시 반경을 한 번 빼서 지면 높이로 보관한다.
// stamps가 주어지면 기록한 셀에 stamp_sec을 적는다(수명 관리용, 선택적).
int ApplySupportPatches(HeightMap* map,
                        const std::array<Eigen::Vector3d, 4>& feet,
                        const std::array<bool, 4>& contacts, double half_width,
                        Eigen::MatrixXd* stamps = nullptr, double stamp_sec = 0.0,
                        double foot_radius = kDefaultFootRadius);
// 관측 지도와 별개인 접촉 증거. 로컬 지도 이동 시 겹치는 셀만 유지한다.
// KF odom z drift(~3mm/s) 때문에 오래된 패치는 옛 datum의 화석이 되어 현재
// datum으로 정렬된 지도 위에 가짜 둔덕을 만든다(2026-10-29 10:27 로그, 4cm 섬).
// 그래서 셀마다 관측 시각을 기록하고 적용 시 수명을 검사한다.
class SupportPatchMemory {
 public:
  void Reset();
  void SyncLayout(const HeightMap& layout);
  // stamp_sec: 접촉 관측 시각(초). ApplyTo의 수명 판정 기준이 된다.
  void Observe(const std::array<Eigen::Vector3d, 4>& feet,
               const std::array<bool, 4>& contacts, double half_width,
               double stamp_sec = 0.0, double foot_radius = kDefaultFootRadius);
  // now_sec − 기록 시각 > max_age_sec인 셀은 건너뛴다. 기본값은 무제한(기존 동작).
  int ApplyTo(HeightMap* map, double now_sec = 0.0,
              double max_age_sec = std::numeric_limits<double>::infinity()) const;

 private:
  std::optional<HeightMap> memory_;
  Eigen::MatrixXd stamps_;
};
}  // namespace b2_terrain_planner
