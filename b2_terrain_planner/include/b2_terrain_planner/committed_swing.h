/**
 * @file committed_swing.h
 * @brief WBC 보행 위상에서 이미 시작된 스윙의 확정 착지점을 최적화에 전달한다.
 * @version 0.0.1
 * @update 2026-10-17: 현재 스윙의 목표 고정과 다음 스윙의 재계획을 분리함.
 */
#pragma once
#include <cmath>
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/swing_reference.h"

namespace b2_terrain_planner {
inline bool CommitActiveSwingFootholds(const Robot& robot,
                                       const std::array<SwingReference, 4>& references,
                                       Input* input) {
  input->committed_footholds = {};
  const auto phases = MakeHorizon(input->gait_time, robot.period, robot.duty);
  for (int leg = 0; leg < 4; ++leg) {
    if (phases.front().contacts[leg]) continue;
    double touchdown = input->gait_time + robot.period;
    for (const auto& phase : phases) {
      if (phase.contacts[leg]) {
        touchdown = input->gait_time + phase.start;
        break;
      }
    }
    const auto& swing = references[leg];
    if (!swing.ActiveAt(input->gait_time) || std::abs(swing.end - touchdown) > 1e-6)
      return false;
    input->committed_footholds[leg] = swing.target;
  }
  return true;
}
}  // namespace b2_terrain_planner
