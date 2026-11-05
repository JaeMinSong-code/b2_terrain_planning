/**
 * @file solver_diagnostics.cpp
 * @brief solver 입력의 명시적 YAML 직렬화와 한 슬롯 비동기 파일 기록.
 * @version 0.0.3
 * @update 2026-11-05: 대체 후보 재시도 설정을 직렬화하고 이전 snapshot 기본값을 유지함.
 * @update 2026-11-05: 보정 생략 설정을 저장하고 이전 snapshot의 기본 보정 동작을 유지함.
 */
#include "b2_terrain_planner/solver_diagnostics.h"
#include "b2_terrain_planner/robot_model.h"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>

namespace b2_terrain_planner {
namespace {
struct Archive;
template <class T> void Visit(Archive& archive, T& value);
template <> void Visit(Archive& archive, Robot& value);
template <> void Visit(Archive& archive, Input& value);
template <> void Visit(Archive& archive, Phase& value);
template <> void Visit(Archive& archive, Settings& value);
template <> void Visit(Archive& archive, SeedContext& value);
template <> void Visit(Archive& archive, QpDiagnostic& value);
template <> void Visit(Archive& archive, Profile& value);
template <> void Visit(Archive& archive, LineSearchDiagnostic& value);
template <> void Visit(Archive& archive, SolveAttemptDiagnostic& value);
template <> void Visit(Archive& archive, HeightMap& value);
template <> void Visit(Archive& archive, TerrainSettings& value);
template <> void Visit(Archive& archive, SolverSnapshot& value);
template <> void Visit(Archive& archive, Result& value);
template <class T> void Transfer(YAML::Node node, T& value, bool reading);
template <class T, std::size_t N>
void Transfer(YAML::Node node, std::array<T, N>& value, bool reading);
template <class T>
void Transfer(YAML::Node node, std::vector<T>& value, bool reading);
template <class T>
void Transfer(YAML::Node node, std::optional<T>& value, bool reading);
template <class S, int R, int C, int O, int MR, int MC>
void Transfer(YAML::Node node, Eigen::Matrix<S, R, C, O, MR, MC>& value, bool reading);
template <class S, int R, int C, int O, int MR, int MC>
void Transfer(YAML::Node node, Eigen::Array<S, R, C, O, MR, MC>& value, bool reading);
void Transfer(YAML::Node node, std::shared_ptr<const Result>& value, bool reading);
struct Archive {
  YAML::Node node;
  bool reading;
  template <class T> void Field(const char* key, T& value) {
    Transfer(node[key], value, reading);
  }
};
template <class T> void Transfer(YAML::Node node, T& value, bool reading) {
  if constexpr (std::is_arithmetic_v<T> || std::is_same_v<T, std::string>) {
    if (reading) value = node.as<T>();
    else node = value;
  } else if constexpr (std::is_enum_v<T>) {
    int number = static_cast<int>(value);
    Transfer(node, number, reading);
    if (reading) value = static_cast<T>(number);
  } else {
    Archive archive{node, reading};
    Visit(archive, value);
  }
}
template <class T, std::size_t N>
void Transfer(YAML::Node node, std::array<T, N>& value, bool reading) {
  if (reading && (!node.IsSequence() || node.size() != N)) throw std::runtime_error("snapshot array size");
  if (!reading) node = YAML::Node(YAML::NodeType::Sequence);
  for (std::size_t i = 0; i < N; ++i) {
    if (!reading) node.push_back(YAML::Node());
    Transfer(node[i], value[i], reading);
  }
}
template <class T>
void Transfer(YAML::Node node, std::vector<T>& value, bool reading) {
  if (reading) {
    if (!node.IsSequence() || node.size() > 1000000) throw std::runtime_error("snapshot vector size");
    value.resize(node.size());
  } else node = YAML::Node(YAML::NodeType::Sequence);
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (!reading) node.push_back(YAML::Node());
    Transfer(node[i], value[i], reading);
  }
}
template <class T>
void Transfer(YAML::Node node, std::optional<T>& value, bool reading) {
  if (reading) {
    if (!node.IsDefined()) throw std::runtime_error("snapshot optional missing");
    if (node.IsNull()) { value.reset(); return; }
    value.emplace();
  } else if (!value) { node = YAML::Null; return; }
  Transfer(node, *value, reading);
}
template <class T> void TransferDense(YAML::Node node, T& value, bool reading) {
  int rows = value.rows(), cols = value.cols();
  Transfer(node["rows"], rows, reading);
  Transfer(node["cols"], cols, reading);
  if (reading) {
    if (rows < 0 || cols < 0 || static_cast<long long>(rows) * cols > 10000000 ||
        (T::RowsAtCompileTime != Eigen::Dynamic && rows != T::RowsAtCompileTime) ||
        (T::ColsAtCompileTime != Eigen::Dynamic && cols != T::ColsAtCompileTime)) {
      throw std::runtime_error("snapshot matrix shape");
    }
    value.resize(rows, cols);
    if (!node["data"].IsSequence() || node["data"].size() != static_cast<std::size_t>(rows) * cols) {
      throw std::runtime_error("snapshot matrix data size");
    }
  } else node["data"] = YAML::Node(YAML::NodeType::Sequence);
  std::size_t index = 0;
  for (int row = 0; row < rows; ++row) for (int col = 0; col < cols; ++col) {
    if (!reading) node["data"].push_back(YAML::Node());
    Transfer(node["data"][index++], value(row, col), reading);
  }
}
template <class S, int R, int C, int O, int MR, int MC>
void Transfer(YAML::Node node, Eigen::Matrix<S, R, C, O, MR, MC>& value, bool reading) {
  TransferDense(node, value, reading);
}
template <class S, int R, int C, int O, int MR, int MC>
void Transfer(YAML::Node node, Eigen::Array<S, R, C, O, MR, MC>& value, bool reading) {
  TransferDense(node, value, reading);
}
void Transfer(YAML::Node node, std::shared_ptr<const Result>& value, bool reading) {
  std::optional<Result> seed;
  if (!reading && value) seed = *value;
  Transfer(node, seed, reading);
  if (reading) {
    value.reset();
    if (seed) value = std::make_shared<Result>(std::move(*seed));
  }
}
std::string ReadText(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read URDF: " + path);
  std::ostringstream text;
  text << stream.rdbuf();
  return text.str();
}
template <> void Visit(Archive& a, Robot& v) {
  a.Field("mass", v.mass);
  a.Field("inertia", v.inertia);
  a.Field("hips", v.hips);
  a.Field("nominal_hips", v.nominal_hips);
  a.Field("nominal_offsets", v.nominal_offsets);
  a.Field("reference_legs", v.reference_legs);
  a.Field("reach_min", v.reach_min);
  a.Field("reach_max", v.reach_max);
  a.Field("use_reference_legs", v.use_reference_legs);
  a.Field("com_offset", v.com_offset);
  a.Field("reference_joints", v.reference_joints);
  a.Field("height", v.height);
  a.Field("friction", v.friction);
  a.Field("leg_min", v.leg_min);
  a.Field("leg_max", v.leg_max);
  a.Field("period", v.period);
  a.Field("duty", v.duty);
  std::string model_path, model_urdf;
  if (!a.reading && v.model) { model_path = v.model->path(); model_urdf = ReadText(model_path); }
  a.Field("model_path", model_path);
  a.Field("model_urdf", model_urdf);
  if (a.reading && !model_path.empty()) {
    if (ReadText(model_path) != model_urdf) throw std::runtime_error("snapshot URDF contents changed");
    v.model = std::make_shared<RobotModel>(model_path);
  }
}
template <> void Visit(Archive& a, Input& v) {
  a.Field("position", v.position);
  a.Field("velocity", v.velocity);
  a.Field("reference_prefix", v.reference_prefix);
  a.Field("prefix_duration", v.prefix_duration);
  a.Field("body_time", v.body_time);
  a.Field("gait_hold_duration", v.gait_hold_duration);
  a.Field("hold_contacts", v.hold_contacts);
  a.Field("feet", v.feet);
  a.Field("command", v.command);
  a.Field("gait_time", v.gait_time);
  a.Field("has_nominal", v.has_nominal);
  a.Field("nominal", v.nominal);
  a.Field("fixed", v.fixed);
  a.Field("committed_footholds", v.committed_footholds);
  a.Field("contact_estimated", v.contact_estimated);
  a.Field("frame", v.frame);
  a.Field("terrain", v.terrain);
}
template <> void Visit(Archive& a, Phase& v) {
  a.Field("start", v.start);
  a.Field("duration", v.duration);
  a.Field("contacts", v.contacts);
  a.Field("landed", v.landed);
}
template <> void Visit(Archive& a, Settings& v) {
  a.Field("qp_reuse", v.qp_reuse);
  a.Field("full_evaluation", v.full_evaluation);
  // 기존 schema 1 snapshot에는 이 설정이 없으며 기본 보정을 유지한다.
  if (!a.reading || a.node["skip_line_search_projection"]) {
    a.Field("skip_line_search_projection", v.skip_line_search_projection);
  }
  if (!a.reading || a.node["terrain_candidate_retries"]) {
    a.Field("terrain_candidate_retries", v.terrain_candidate_retries);
  }
  a.Field("line_search_diagnostics", v.line_search_diagnostics);
  a.Field("foothold_diagnostics", v.foothold_diagnostics);
  a.Field("qp_working_set", v.qp_working_set);
  a.Field("structured_equalities", v.structured_equalities);
  a.Field("block_assembly", v.block_assembly);
  a.Field("polynomial_tail", v.polynomial_tail);
  a.Field("projection_l2", v.projection_l2);
  a.Field("chord_projection", v.chord_projection);
  a.Field("equality_rank_tolerance", v.equality_rank_tolerance);
  a.Field("projection_rank_tolerance", v.projection_rank_tolerance);
  a.Field("projection_residual_tolerance", v.projection_residual_tolerance);
  a.Field("lapack_svd", v.lapack_svd);
  a.Field("max_iterations", v.max_iterations);
  a.Field("realtime_iterations", v.realtime_iterations);
  a.Field("orientation_limit", v.orientation_limit);
  a.Field("max_reach_margin", v.max_reach_margin);
  a.Field("pitch_weight", v.pitch_weight);
  a.Field("nominal_kinematics_weight", v.nominal_kinematics_weight);
  a.Field("vz_weight", v.vz_weight);
  a.Field("foothold_backward_weight", v.foothold_backward_weight);
  a.Field("previous_foothold_weight", v.previous_foothold_weight);
  a.Field("qp_regularization", v.qp_regularization);
  a.Field("qp_eps_abs", v.qp_eps_abs);
  a.Field("qp_eps_rel", v.qp_eps_rel);
  a.Field("qp_max_iterations", v.qp_max_iterations);
  a.Field("sample_intervals", v.sample_intervals);
  a.Field("feasibility_tolerance", v.feasibility_tolerance);
  a.Field("step_tolerance", v.step_tolerance);
  a.Field("graduated_initialization", v.graduated_initialization);
  a.Field("graduated_budget_fraction", v.graduated_budget_fraction);
  a.Field("terrain", v.terrain);
  a.Field("time_limit", v.time_limit);
}
template <> void Visit(Archive& a, SeedContext& v) {
  a.Field("gait_time", v.gait_time);
  a.Field("period", v.period);
  a.Field("duty", v.duty);
  a.Field("frame", v.frame);
  a.Field("robot_signature", v.robot_signature);
  a.Field("terrain", v.terrain);
  a.Field("initial_feet", v.initial_feet);
  a.Field("terrain_signature", v.terrain_signature);
  a.Field("body_time", v.body_time);
  a.Field("gait_hold_duration", v.gait_hold_duration);
  a.Field("hold_contacts", v.hold_contacts);
}
template <> void Visit(Archive& a, QpDiagnostic& v) {
  a.Field("working_set_pass", v.working_set_pass);
  a.Field("working_set_total_rows", v.working_set_total_rows);
  a.Field("working_set_omitted_violation", v.working_set_omitted_violation);
  a.Field("working_set_full_violation", v.working_set_full_violation);
  a.Field("workspace_reused", v.workspace_reused);
  a.Field("workspace_reason", v.workspace_reason);
  a.Field("update_return", v.update_return);
  a.Field("update_seconds", v.update_seconds);
  a.Field("warm_start_return", v.warm_start_return);
  a.Field("warm_start_seconds", v.warm_start_seconds);
  a.Field("warm_start_used", v.warm_start_used);
  a.Field("warm_start_reason", v.warm_start_reason);
  a.Field("reused_duals", v.reused_duals);
  a.Field("variables", v.variables);
  a.Field("rows", v.rows);
  a.Field("warm_start_roundtrip_error", v.warm_start_roundtrip_error);
  a.Field("warm_start_projection_error", v.warm_start_projection_error);
  a.Field("sqp_iteration", v.sqp_iteration);
  a.Field("kind", v.kind);
  a.Field("setup_return", v.setup_return);
  a.Field("solve_return", v.solve_return);
  a.Field("status_code", v.status_code);
  a.Field("status", v.status);
  a.Field("iterations", v.iterations);
  a.Field("primal_residual", v.primal_residual);
  a.Field("dual_residual", v.dual_residual);
  a.Field("polish_status", v.polish_status);
  a.Field("rho_updates", v.rho_updates);
  a.Field("setup_seconds", v.setup_seconds);
  a.Field("solve_seconds", v.solve_seconds);
  a.Field("accepted", v.accepted);
  a.Field("acceptance_reason", v.acceptance_reason);
}
template <> void Visit(Archive& a, Profile& v) {
  a.Field("initialization_seconds", v.initialization_seconds);
  a.Field("ik_seconds", v.ik_seconds);
  a.Field("final_validation_seconds", v.final_validation_seconds);
  a.Field("full_evaluation_seconds", v.full_evaluation_seconds);
  a.Field("values_evaluation_seconds", v.values_evaluation_seconds);
  a.Field("constraint_jacobian_seconds", v.constraint_jacobian_seconds);
  a.Field("svd_seconds", v.svd_seconds);
  a.Field("qp_prepare_seconds", v.qp_prepare_seconds);
  a.Field("qp_setup_seconds", v.qp_setup_seconds);
  a.Field("qp_solve_seconds", v.qp_solve_seconds);
  a.Field("qp_cleanup_seconds", v.qp_cleanup_seconds);
  a.Field("qp_update_seconds", v.qp_update_seconds);
  a.Field("qp_warm_start_seconds", v.qp_warm_start_seconds);
  a.Field("projection_seconds", v.projection_seconds);
  a.Field("line_search_seconds", v.line_search_seconds);
  a.Field("full_evaluations", v.full_evaluations);
  a.Field("values_evaluations", v.values_evaluations);
  a.Field("constraint_jacobian_evaluations", v.constraint_jacobian_evaluations);
  a.Field("svd_calls", v.svd_calls);
  a.Field("qp_calls", v.qp_calls);
  a.Field("qp_setups", v.qp_setups);
  a.Field("qp_updates", v.qp_updates);
  a.Field("qp_warm_starts", v.qp_warm_starts);
  a.Field("qp_iterations", v.qp_iterations);
  a.Field("qp_iterations_per_solve", v.qp_iterations_per_solve);
  a.Field("qp_diagnostics", v.qp_diagnostics);
}
template <> void Visit(Archive& a, LineSearchDiagnostic& v) {
  a.Field("iteration", v.iteration);
  a.Field("search", v.search);
  a.Field("alpha", v.alpha);
  a.Field("penalty", v.penalty);
  a.Field("current_cost", v.current_cost);
  a.Field("current_violation", v.current_violation);
  a.Field("current_scaled_violation", v.current_scaled_violation);
  a.Field("slope", v.slope);
  a.Field("predicted_change", v.predicted_change);
  a.Field("acceptance_bound", v.acceptance_bound);
  a.Field("step_norm", v.step_norm);
  a.Field("before_cost", v.before_cost);
  a.Field("before_violation", v.before_violation);
  a.Field("before_scaled_violation", v.before_scaled_violation);
  a.Field("after_cost", v.after_cost);
  a.Field("after_violation", v.after_violation);
  a.Field("after_scaled_violation", v.after_scaled_violation);
  a.Field("correction_norm", v.correction_norm);
  a.Field("projection_seconds", v.projection_seconds);
  a.Field("diagnostic_seconds", v.diagnostic_seconds);
  a.Field("accepted", v.accepted);
  a.Field("terrain_domain_error", v.terrain_domain_error);
}
template <> void Visit(Archive& a, SolveAttemptDiagnostic& v) {
  a.Field("layer", v.layer);
  a.Field("leg", v.leg);
  a.Field("status", v.status);
  a.Field("success", v.success);
  a.Field("iterations", v.iterations);
  a.Field("elapsed", v.elapsed);
  a.Field("profile", v.profile);
  a.Field("line_search", v.line_search);
}
template <> void Visit(Archive& a, HeightMap& v) {
  a.Field("heights", v.heights);
  a.Field("valid", v.valid);
  a.Field("variance", v.variance);
  a.Field("origin", v.origin);
  a.Field("resolution", v.resolution);
  a.Field("frame", v.frame);
}
template <> void Visit(Archive& a, TerrainSettings& v) {
  a.Field("pipeline_enabled", v.pipeline_enabled);
  a.Field("median_iterations", v.median_iterations);
  a.Field("virtual_floor_sigma", v.virtual_floor_sigma);
  a.Field("virtual_floor_radius", v.virtual_floor_radius);
  a.Field("occlusion_weight", v.occlusion_weight);
  a.Field("curvature_weight", v.curvature_weight);
  a.Field("virtual_height_weight", v.virtual_height_weight);
  a.Field("body_alignment_weight", v.body_alignment_weight);
  a.Field("foot_radius", v.foot_radius);
  a.Field("smoothing_sigma", v.smoothing_sigma);
  a.Field("contact_tolerance", v.contact_tolerance);
  a.Field("ground_weight", v.ground_weight);
  a.Field("edge_weight", v.edge_weight);
  a.Field("collision_weight", v.collision_weight);
  a.Field("foot_separation_weight", v.foot_separation_weight);
  a.Field("foot_separation_distance_y", v.foot_separation_distance_y);
  a.Field("foot_separation_scale", v.foot_separation_scale);
  a.Field("collision_distance", v.collision_distance);
}
template <> void Visit(Archive& a, SolverSnapshot& v) {
  a.Field("solve_start_ns", v.solve_start_ns);
  a.Field("state_stamp_ns", v.state_stamp_ns);
  a.Field("map_stamp_ns", v.map_stamp_ns);
  a.Field("robot", v.robot);
  a.Field("input", v.input);
  a.Field("settings", v.settings);
  a.Field("previous", v.previous);
  a.Field("status", v.status);
  a.Field("elapsed", v.elapsed);
  a.Field("attempts", v.attempts);
}
template <> void Visit(Archive& a, Result& v) {
  a.Field("success", v.success);
  a.Field("initialization_available", v.initialization_available);
  a.Field("seed_context", v.seed_context);
  a.Field("footholds", v.footholds);
  a.Field("phases", v.phases);
  a.Field("coefficients", v.coefficients);
  a.Field("margins", v.margins);
}
}  // namespace

void SaveSolverSnapshot(const std::string& path, SolverSnapshot& snapshot) {
  YAML::Node root;
  root["schema_version"] = 1;
  root["build"] = __DATE__ " " __TIME__;
  root["timing_note"] = "line_search includes projection; detailed raw evaluation adds diagnostic overhead";
  Transfer(root["snapshot"], snapshot, false);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << root;
  if (!emitter.good()) throw std::runtime_error(emitter.GetLastError());
  std::ofstream stream(path, std::ios::trunc);
  stream << emitter.c_str() << '\n';
  stream.close();
  if (!stream) throw std::runtime_error("snapshot write failed: " + path);
}
SolverSnapshot LoadSolverSnapshot(const std::string& path) {
  auto root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1) throw std::runtime_error("unsupported snapshot schema");
  SolverSnapshot snapshot;
  Transfer(root["snapshot"], snapshot, true);
  return snapshot;
}
std::string SolverDiagnosticsYaml(std::vector<SolveAttemptDiagnostic>& attempts) {
  YAML::Node root;
  Transfer(root["solver_attempts"], attempts, false);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << root;
  return emitter.c_str();
}

struct SolverSnapshotWriter::Impl {
  explicit Impl(std::string path) : directory(std::move(path)), worker([this] { Run(); }) {}
  ~Impl() {
    { std::lock_guard<std::mutex> lock(mutex); stopping = true; }
    ready.notify_one();
    worker.join();
  }
  void Run() {
    for (;;) {
      std::optional<SolverSnapshot> job;
      {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock, [this] { return stopping || pending.has_value(); });
        if (!pending && stopping) break;
        job = std::move(pending);
        pending.reset();
      }
      try {
        std::filesystem::create_directories(directory);
        // 최대 32개를 순환 저장한다. 완료된 파일만 rename으로 공개한다.
        const auto path = std::filesystem::path(directory) /
            ("solve_" + std::to_string(slot++ % 32) + ".yaml");
        const auto temporary = path.string() + ".tmp";
        SaveSolverSnapshot(temporary, *job);
        std::filesystem::rename(temporary, path);
      } catch (const std::exception& error) {
        ++errors;
        std::cerr << "[SqpDiagnostics] " << error.what() << '\n';
      }
    }
  }
  std::string directory;
  std::mutex mutex;
  std::condition_variable ready;
  std::optional<SolverSnapshot> pending;
  bool stopping = false;
  std::uint64_t slot = 0;
  std::atomic<std::uint64_t> dropped{0}, errors{0};
  std::thread worker;
};
SolverSnapshotWriter::SolverSnapshotWriter(std::string directory)
    : impl_(std::make_unique<Impl>(std::move(directory))) {}
SolverSnapshotWriter::~SolverSnapshotWriter() = default;
void SolverSnapshotWriter::Submit(SolverSnapshot snapshot) {
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->pending) ++impl_->dropped;
    impl_->pending = std::move(snapshot);
  }
  impl_->ready.notify_one();
}
std::uint64_t SolverSnapshotWriter::dropped() const { return impl_->dropped.load(); }
std::uint64_t SolverSnapshotWriter::errors() const { return impl_->errors.load(); }
}  // namespace b2_terrain_planner
