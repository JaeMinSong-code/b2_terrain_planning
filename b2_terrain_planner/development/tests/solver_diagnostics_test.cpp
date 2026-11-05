/**
 * @file solver_diagnostics_test.cpp
 * @brief snapshot 왕복 재생·상세 진단 불변성·비동기 저장을 검증한다.
 * @version 0.0.1
 * @update 2026-11-05: 성공해·확정 착지·지도·ns 정밀도와 실패 후보 계측을 확인함.
 */
#include "b2_terrain_planner/solver_diagnostics.h"
#include "b2_terrain_planner/robot_model.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
using namespace b2_terrain_planner;
void Check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
std::string Text(const std::string& path) {
  std::ifstream file(path);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
void SamePlan(const Result& a, const Result& b) {
  Check(a.success == b.success && a.status == b.status && a.iterations == b.iterations,
        "replay changed solver decisions");
  Check(a.cost == b.cost && a.violation == b.violation && a.footholds == b.footholds,
        "replay changed output values");
  Check(a.coefficients.size() == b.coefficients.size(), "coefficient count changed");
  for (std::size_t i = 0; i < a.coefficients.size(); ++i) {
    Check(a.coefficients[i] == b.coefficients[i], "coefficient values changed");
  }
}
void Run(const std::filesystem::path& directory) {
  SolverSnapshot snapshot;
  snapshot.solve_start_ns = 1793865228001175436LL;
  snapshot.state_stamp_ns = 1793865227997606227LL;
  snapshot.map_stamp_ns = 1793865227987123456LL;
  auto model = std::make_shared<RobotModel>(B2_BUILD_MODEL_PATH);
  ModelState state;
  for (int leg = 0; leg < 4; ++leg) {
    state.joints.segment<3>(3 * leg) << 0, 44.30 * std::acos(-1) / 180,
        -85.94 * std::acos(-1) / 180;
  }
  state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
  model->PopulatePlanner(state, &snapshot.robot, &snapshot.input);
  auto& input = snapshot.input;
  HeightMap map;
  map.origin = Eigen::Vector2d::Constant(-1.975);
  map.heights = Eigen::MatrixXd::Constant(80, 80, -.032);
  map.valid = Eigen::ArrayXXi::Ones(80, 80);
  map.variance = Eigen::MatrixXd::Constant(80, 80, .00012345678901234567);
  map.heights(0, 0) = std::numeric_limits<double>::quiet_NaN();
  map.valid(0, 0) = 0;
  input.terrain = map;
  input.body_time = 0;
  input.command.x() = .1;
  auto& settings = snapshot.settings;
  settings.time_limit = 0;
  settings.sample_intervals = 4;
  settings.max_reach_margin = .02;
  settings.qp_working_set = true;
  settings.qp_reuse = QpReuseMode::kWarmStart;
  settings.structured_equalities = true;
  settings.block_assembly = true;
  settings.lapack_svd = true;
  settings.realtime_iterations = 2;
  snapshot.previous = Solve(snapshot.robot, input, settings);
  Check(snapshot.previous->success, "initial solve failed");
  input.gait_time = .02;
  input.body_time = .02;
  input.position = Evaluate(*snapshot.previous, .02);
  input.velocity = Evaluate(*snapshot.previous, .02, 1);
  input.committed_footholds[0] = snapshot.previous->footholds.row(0).transpose();
  input.contact_estimated = std::array<bool, 4>{true, true, true, true};
  const auto baseline = Solve(snapshot.robot, input, settings, &*snapshot.previous);
  Check(baseline.success, "warm baseline failed");
  snapshot.status = baseline.status;
  snapshot.elapsed = baseline.elapsed;
  snapshot.attempts = baseline.solver_attempts;
  const auto path = (directory / "snapshot.yaml").string();
  SaveSolverSnapshot(path, snapshot);
  auto loaded = LoadSolverSnapshot(path);
  Check(loaded.solve_start_ns == snapshot.solve_start_ns &&
        loaded.state_stamp_ns == snapshot.state_stamp_ns, "ns precision lost");
  Check(std::isnan(loaded.input.terrain->heights(0, 0)) &&
        loaded.input.terrain->valid(0, 0) == 0, "map NaN or mask lost");
  const auto second = (directory / "roundtrip.yaml").string();
  SaveSolverSnapshot(second, loaded);
  Check(Text(path) == Text(second), "snapshot fields changed during roundtrip");
  const auto replay = Solve(loaded.robot, loaded.input, loaded.settings, &*loaded.previous);
  SamePlan(baseline, replay);
  loaded.settings.line_search_diagnostics = true;
  const auto detailed = Solve(loaded.robot, loaded.input, loaded.settings, &*loaded.previous);
  SamePlan(baseline, detailed);
  int records = 0;
  for (const auto& attempt : detailed.solver_attempts) {
    Check(attempt.line_search.size() <= static_cast<std::size_t>(settings.max_iterations * 10),
          "unbounded line-search capture");
    Check(attempt.profile.line_search_seconds >= attempt.profile.projection_seconds,
          "nested timing reported incorrectly");
    for (const auto& trial : attempt.line_search) {
      ++records;
      Check(trial.before_cost.has_value() && trial.before_violation.has_value(), "raw candidate missing");
      if (trial.after_cost && trial.after_scaled_violation) {
        const double merit = *trial.after_cost + trial.penalty * *trial.after_scaled_violation;
        Check(trial.accepted == (merit <= trial.acceptance_bound), "acceptance record mismatch");
      }
    }
  }
  Check(records > 0, "no line-search records");
  for (const auto& attempt : baseline.solver_attempts) {
    for (const auto& trial : attempt.line_search) Check(!trial.before_cost, "online raw evaluation enabled");
  }
  // prefix·hold·nominal 등 선택 입력도 손실 없이 보존한다. 여기서는 solver 실행과 분리한다.
  loaded.input.reference_prefix = std::make_shared<Result>(*loaded.previous);
  loaded.input.prefix_duration = .08;
  loaded.input.gait_hold_duration = .006;
  loaded.input.hold_contacts = std::array<bool, 4>{true, false, false, true};
  loaded.input.has_nominal = true;
  loaded.input.nominal.setConstant(.12345678901234567);
  SaveSolverSnapshot(path, loaded);
  auto optional = LoadSolverSnapshot(path);
  SaveSolverSnapshot(second, optional);
  Check(Text(path) == Text(second), "optional inputs lost");
  {
    SolverSnapshotWriter writer((directory / "async").string());
    writer.Submit(snapshot);
  }
  auto asynchronous = LoadSolverSnapshot((directory / "async/solve_0.yaml").string());
  Check(asynchronous.state_stamp_ns == snapshot.state_stamp_ns, "async writer lost input");
  // 알려진 비실행 가능 입력도 실패로 보존하며 후보별 Profile을 누락하지 않는다.
  loaded.input.reference_prefix.reset();
  loaded.input.gait_hold_duration = 0;
  loaded.input.hold_contacts.reset();
  loaded.input.has_nominal = false;
  loaded.input.committed_footholds = {};
  loaded.input.position.z() += .5;
  loaded.settings.max_iterations = 1;
  loaded.settings.realtime_iterations = 0;
  for (int row = 0; row < loaded.input.terrain->heights.rows(); ++row) {
    loaded.input.terrain->heights.row(row).segment(40, 40).array() += .05;
  }
  const auto failed = Solve(loaded.robot, loaded.input, loaded.settings, &*loaded.previous);
  Check(!failed.success, "failure fixture unexpectedly feasible");
  Check(!failed.solver_attempts.empty(), "failed attempt profile absent");
  std::size_t ground = 0;
  for (const auto& attempt : failed.solver_attempts) if (attempt.layer == "ground") ++ground;
  Check(ground == failed.terrain_attempts.size(), "discarded retry profiles lost");
  Check(ground > 1, "fixture did not exercise alternative retry");
  std::cout << "line_search_records=" << records << " failed_ground_attempts=" << ground << '\n';
}
}  // namespace
int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("b2_solver_diagnostics_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  try { Run(directory); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  std::cout << "PASS solver snapshot replay, tracing and asynchronous storage: " << directory << '\n';
}
