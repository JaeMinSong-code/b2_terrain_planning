/**
 * @file solver_diagnostics.h
 * @brief SQP 재생 입력과 제한된 비동기 진단 저장 계약.
 * @version 0.0.1
 * @update 2026-11-05: 상태·처리 전 지도·설정·성공해와 후보별 계측 저장을 추가함.
 */
#ifndef B2_TERRAIN_PLANNER_SOLVER_DIAGNOSTICS_H_
#define B2_TERRAIN_PLANNER_SOLVER_DIAGNOSTICS_H_
#include "b2_terrain_planner/planner.h"
#include <cstdint>
#include <memory>
#include <string>

namespace b2_terrain_planner {
struct SolverSnapshot {
  std::int64_t solve_start_ns = 0;
  std::int64_t state_stamp_ns = 0;
  std::int64_t map_stamp_ns = 0;
  Robot robot;
  Input input;
  Settings settings;
  std::optional<Result> previous;
  std::string status;
  double elapsed = 0;
  std::vector<SolveAttemptDiagnostic> attempts;
};
// YAML double은 17자리로 저장한다. 현재 URDF 내용이 달라지면 재생을 거부한다.
void SaveSolverSnapshot(const std::string& path, SolverSnapshot& snapshot);
SolverSnapshot LoadSolverSnapshot(const std::string& path);
std::string SolverDiagnosticsYaml(std::vector<SolveAttemptDiagnostic>& attempts);

class SolverSnapshotWriter {
 public:
  explicit SolverSnapshotWriter(std::string directory);
  ~SolverSnapshotWriter();
  // 대기 슬롯은 하나이며 새 작업이 오면 오래된 대기 작업을 교체한다. 제어를 기다리지 않는다.
  void Submit(SolverSnapshot snapshot);
  std::uint64_t dropped() const;
  std::uint64_t errors() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_SOLVER_DIAGNOSTICS_H_
