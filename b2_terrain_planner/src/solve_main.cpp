/**
 * @file solve_main.cpp
 * @brief ROS 없이 평지·지형 입력을 읽고 검증된 몸통 계획을 YAML로 출력한다.
 * @version 0.0.24
 * @update 2026-10-15: B2 YAML 실행과 warm start를 유지하고 합성 모델·재생·제약 해제·상세 추적 도구를 제거함.
 * @update 2026-10-14: 폐기한 추가 접촉력 검증기와 전용 시간의 YAML 출력을 제거함.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"

#include <filesystem>
#include <cmath>

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace {
template <typename Derived>
void ReadVector(const YAML::Node& node, Eigen::MatrixBase<Derived>& destination) {
  if (!node.IsSequence() || node.size() != static_cast<size_t>(destination.size())) {
    throw std::invalid_argument("input vector size mismatch");
  }
  for (int k = 0; k < destination.size(); ++k) destination.derived().data()[k] = node[k].as<double>();
}

template <typename Derived>
void ReadMatrix(const YAML::Node& node, Eigen::MatrixBase<Derived>& destination) {
  if (!node.IsSequence() || node.size() != static_cast<size_t>(destination.rows())) {
    throw std::invalid_argument("input matrix row count mismatch");
  }
  for (int row = 0; row < destination.rows(); ++row) {
    if (!node[row].IsSequence() || node[row].size() != static_cast<size_t>(destination.cols())) {
      throw std::invalid_argument("input matrix column count mismatch");
    }
    for (int col = 0; col < destination.cols(); ++col) destination(row, col) = node[row][col].as<double>();
  }
}

std::string DefaultModelPath() {
  const auto executable = std::filesystem::canonical("/proc/self/exe");
  const auto installed = executable.parent_path().parent_path().parent_path() /
      "share/b2_terrain_planner/model/unitree_b2/b2.urdf";
  if (std::filesystem::exists(installed)) return installed.string();
  return B2_BUILD_MODEL_PATH;
}

void InitializeB2(b2_terrain_planner::Robot* robot, b2_terrain_planner::Input* input) {
  auto model = std::make_shared<b2_terrain_planner::RobotModel>(DefaultModelPath());
  b2_terrain_planner::ModelState state;
  for (int leg = 0; leg < 4; ++leg) {
    state.joints.segment<3>(3 * leg) << 0, 44.30 * std::acos(-1) / 180,
        -85.94 * std::acos(-1) / 180;
  }
  // 예제에서만 전체 로봇을 평행 이동한다. 개별 발 좌표는 FK 그대로 유지한다.
  state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
  model->PopulatePlanner(state, robot, input);
}

void ReadInput(const std::string& path, b2_terrain_planner::Robot* robot,
               b2_terrain_planner::Input* input,
               b2_terrain_planner::Settings* settings) {
  const auto node = YAML::LoadFile(path);
  if (robot->model) {
    if (node["robot"] || node["position"] || node["velocity"] || node["feet"]) {
      throw std::invalid_argument("B2 input requires model_state; legacy robot/position/velocity/feet keys are unsupported");
    }
    if (node["model_state"]) {
      const auto config = node["model_state"];
      b2_terrain_planner::ModelState state;
      ReadVector(config["base_position"], state.base_position);
      Eigen::Vector4d quaternion;
      ReadVector(config["base_quaternion_xyzw"], quaternion);
      state.base_orientation = Eigen::Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
      ReadVector(config["base_linear_velocity_world"], state.base_linear_velocity);
      ReadVector(config["base_angular_velocity_body"], state.base_angular_velocity);
      ReadVector(config["joint_positions"], state.joints);
      ReadVector(config["joint_velocities"], state.joint_velocities);
      std::filesystem::path model_path = DefaultModelPath();
      if (node["model_path"]) {
        model_path = node["model_path"].as<std::string>();
        if (model_path.is_relative()) model_path = std::filesystem::path(path).parent_path() / model_path;
      }
      auto model = std::make_shared<b2_terrain_planner::RobotModel>(model_path.string());
      model->PopulatePlanner(state, robot, input);
    } else if (node["model_path"]) {
      throw std::invalid_argument("model_path requires model_state");
    }
  }
  if (node["command"]) ReadVector(node["command"], input->command);
  if (node["nominal"]) {
    ReadMatrix(node["nominal"], input->nominal);
    input->has_nominal = true;
  }
  if (node["gait_time"]) input->gait_time = node["gait_time"].as<double>();
  if (node["fixed"]) {
    if (!node["fixed"].IsSequence() || node["fixed"].size() != 4) throw std::invalid_argument("fixed needs four flags");
    for (int leg = 0; leg < 4; ++leg) input->fixed[leg] = node["fixed"][leg].as<bool>();
  }
  if (node["frame"]) input->frame = node["frame"].as<std::string>();
  if (node["terrain"]) {
    const auto config = node["terrain"];
    b2_terrain_planner::HeightMap map;
    map.frame = config["frame"].as<std::string>();
    map.resolution = config["resolution"].as<double>();
    ReadVector(config["origin"], map.origin);
    const auto heights = config["heights"];
    if (!heights.IsSequence() || heights.size() < 3 ||
        !heights[0].IsSequence() || heights[0].size() < 3) {
      throw std::invalid_argument("terrain heights must be a matrix of at least 3x3");
    }
    const int rows = heights.size();
    const int cols = heights[0].size();
    map.heights.resize(rows, cols);
    map.variance.resize(rows, cols);
    ReadMatrix(heights, map.heights);
    ReadMatrix(config["variance"], map.variance);
    Eigen::MatrixXd validity(rows, cols);
    ReadMatrix(config["valid"], validity);
    if (!validity.allFinite() ||
        ((validity.array() != 0) && (validity.array() != 1)).any()) {
      throw std::invalid_argument("terrain valid must contain 0 or 1");
    }
    map.valid = validity.cast<int>().array();
    input->terrain = std::move(map);
  }
  if (node["terrain_settings"]) {
    const auto config = node["terrain_settings"];
    for (const char* removed : {"foot_radius", "surface_tolerance",
                                "max_contact_slope", "min_foot_distance"}) {
      if (config[removed]) {
        throw std::invalid_argument(std::string("removed terrain setting; delete key: ") + removed);
      }
    }
    auto& t = settings->terrain;
    for (auto item : {
             std::make_pair("smoothing_sigma", &t.smoothing_sigma),
             {"max_variance", &t.max_variance},
             {"contact_tolerance", &t.contact_tolerance},
             {"ground_weight", &t.ground_weight}, {"edge_weight", &t.edge_weight},
             {"collision_weight", &t.collision_weight},
             {"collision_distance", &t.collision_distance}}) {
      if (config[item.first]) *item.second = config[item.first].as<double>();
    }
  }
}

template <typename T>
void PrintOptional(YAML::Emitter& output, const char* name,
                   const std::optional<T>& value) {
  output << YAML::Key << name << YAML::Value;
  if (value.has_value()) {
    output << *value;
  } else {
    output << YAML::Null;
  }
}

// 숫자 뒤에 남는 문자가 있어도 설정 오류로 처리한다.
double ReadDoubleOption(const std::string& text) {
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size()) throw std::invalid_argument("invalid numeric option: " + text);
  return value;
}

int ReadIntOption(const std::string& text) {
  size_t consumed = 0;
  const int value = std::stoi(text, &consumed);
  if (consumed != text.size()) throw std::invalid_argument("invalid integer option: " + text);
  return value;
}

void PrintGroups(YAML::Emitter& output,
                 const std::vector<b2_terrain_planner::ConstraintDiagnostic>& groups) {
  output << YAML::Key << "constraint_groups" << YAML::Value << YAML::BeginSeq;
  for (const auto& group : groups) {
    output << YAML::BeginMap
           << YAML::Key << "name" << YAML::Value << group.name
           << YAML::Key << "rows" << YAML::Value << group.rows
           << YAML::Key << "max_violation" << YAML::Value << group.max_violation
           << YAML::Key << "phase" << YAML::Value << group.phase
           << YAML::Key << "sample" << YAML::Value << group.sample
           << YAML::Key << "row" << YAML::Value << group.row << YAML::EndMap;
  }
  output << YAML::EndSeq;
}

b2_terrain_planner::Result ReadPlan(const std::string& path) {
  using namespace b2_terrain_planner;
  const auto node = YAML::LoadFile(path);
  Result result;
  result.success = node["success"].as<bool>();
  // 이전 버전에서 제약을 해제해 만든 계획은 warm start로 사용하지 않는다.
  if ((node["diagnostic_only"] && node["diagnostic_only"].as<bool>()) ||
      (node["constraints_reduced"] && node["constraints_reduced"].as<bool>())) {
    throw std::invalid_argument("previous plan uses removed diagnostic mode");
  }
  if (!result.success) return result;
  const auto context = node["seed_context"];
  result.seed_context.gait_time = context["gait_time"].as<double>();
  result.seed_context.period = context["period"].as<double>();
  result.seed_context.duty = context["duty"].as<double>();
  result.seed_context.frame = context["frame"].as<std::string>();
  result.seed_context.robot_signature = context["robot_signature"].as<std::string>();
  result.seed_context.terrain = context["terrain"].as<bool>();
  if (context["terrain_signature"]) {
    result.seed_context.terrain_signature = context["terrain_signature"].as<std::string>();
  }
  ReadMatrix(node["model"]["initial_feet"], result.seed_context.initial_feet);
  ReadMatrix(node["footholds"], result.footholds);
  if (!node["segments"].IsSequence() || node["segments"].size() > 16) {
    throw std::invalid_argument("invalid previous segments");
  }
  double start = 0;
  for (const auto& segment : node["segments"]) {
    Phase phase;
    phase.start = start;
    phase.duration = segment["duration"].as<double>();
    if (!std::isfinite(phase.duration) || phase.duration <= 0 ||
        segment["contacts"].size() != 4 || segment["landed"].size() != 4) {
      throw std::invalid_argument("invalid previous phase");
    }
    for (int leg = 0; leg < 4; ++leg) {
      phase.contacts[leg] = segment["contacts"][leg].as<bool>();
      phase.landed[leg] = segment["landed"][leg].as<bool>();
    }
    Eigen::Matrix<double, 6, 5> coefficients;
    ReadMatrix(segment["coefficients"], coefficients);
    result.phases.push_back(phase);
    result.coefficients.push_back(coefficients);
    result.margins.push_back(segment["margin"].as<double>());
    start += phase.duration;
  }
  return result;
}

void Print(const b2_terrain_planner::Result& result, bool show_profile,
           const b2_terrain_planner::Settings& settings,
           const b2_terrain_planner::Robot& robot, const b2_terrain_planner::Input& input) {
  YAML::Emitter output;
  output.SetDoublePrecision(17);
  output << YAML::BeginMap << YAML::Key << "success" << YAML::Value << result.success
         << YAML::Key << "status" << YAML::Value << result.status
         << YAML::Key << "iterations" << YAML::Value << result.iterations
         << YAML::Key << "elapsed" << YAML::Value << result.elapsed
         << YAML::Key << "cost" << YAML::Value << result.cost
         << YAML::Key << "violation" << YAML::Value << result.violation;
  const auto& seed = result.seed_context;
  output << YAML::Key << "seed_context" << YAML::Value << YAML::BeginMap
         << YAML::Key << "gait_time" << YAML::Value << seed.gait_time
         << YAML::Key << "period" << YAML::Value << seed.period
         << YAML::Key << "duty" << YAML::Value << seed.duty
         << YAML::Key << "frame" << YAML::Value << seed.frame
         << YAML::Key << "robot_signature" << YAML::Value << seed.robot_signature
         << YAML::Key << "terrain_signature" << YAML::Value << seed.terrain_signature
         << YAML::Key << "terrain" << YAML::Value << seed.terrain << YAML::EndMap;
  const auto& warm = result.warm_start;
  output << YAML::Key << "warm_start" << YAML::Value << YAML::BeginMap
         << YAML::Key << "requested" << YAML::Value << warm.requested
         << YAML::Key << "used" << YAML::Value << warm.used
         << YAML::Key << "reason" << YAML::Value << warm.reason
         << YAML::Key << "elapsed" << YAML::Value << warm.elapsed
         << YAML::Key << "shifted_phases" << YAML::Value << warm.shifted_phases
         << YAML::Key << "polynomial_extended_phases" << YAML::Value << warm.polynomial_extended_phases
         << YAML::Key << "extended_phases" << YAML::Value << warm.extended_phases
         << YAML::Key << "reach_reselected_footholds" << YAML::Value << warm.reach_reselected_footholds
         << YAML::Key << "reach_candidate_fallbacks" << YAML::Value << warm.reach_candidate_fallbacks
         << YAML::Key << "reused_footholds" << YAML::Value << warm.reused_footholds
         << YAML::EndMap;
  output << YAML::Key << "model" << YAML::Value << YAML::BeginMap;
  const std::string model_name = robot.model->path();
  output << YAML::Key << "urdf" << YAML::Value << model_name
         << YAML::Key << "mass" << YAML::Value << robot.mass
         << YAML::Key << "height" << YAML::Value << robot.height
         << YAML::Key << "leg_min" << YAML::Value << robot.leg_min
         << YAML::Key << "leg_max" << YAML::Value << robot.leg_max
         << YAML::Key << "inertia_body" << YAML::Value << YAML::BeginSeq;
  for (int row = 0; row < 3; ++row) {
    output << YAML::Flow << YAML::BeginSeq;
    for (int col = 0; col < 3; ++col) output << robot.inertia(row, col);
    output << YAML::EndSeq;
  }
  output << YAML::EndSeq << YAML::Key << "joint_limited_reach" << YAML::Value << YAML::BeginSeq;
  if (robot.model) {
    for (int leg = 0; leg < 4; ++leg) {
      output << YAML::Flow << YAML::BeginSeq << robot.reach_min[leg]
             << robot.reach_max[leg] << YAML::EndSeq;
    }
  }
  output << YAML::EndSeq << YAML::Key << "hips_com_body" << YAML::Value << YAML::BeginSeq;
  for (int leg = 0; leg < 4; ++leg) {
    output << YAML::Flow << YAML::BeginSeq;
    for (int axis = 0; axis < 3; ++axis) output << robot.hips(leg, axis);
    output << YAML::EndSeq;
  }
  output << YAML::EndSeq << YAML::Key << "initial_position" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int axis = 0; axis < 6; ++axis) output << input.position[axis];
  output << YAML::EndSeq << YAML::Key << "initial_velocity" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int axis = 0; axis < 6; ++axis) output << input.velocity[axis];
  output << YAML::EndSeq << YAML::Key << "initial_feet" << YAML::Value << YAML::BeginSeq;
  for (int leg = 0; leg < 4; ++leg) {
    output << YAML::Flow << YAML::BeginSeq;
    for (int axis = 0; axis < 3; ++axis) output << input.feet(leg, axis);
    output << YAML::EndSeq;
  }
  output << YAML::EndSeq << YAML::Key << "com_offset_body" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int axis = 0; axis < 3; ++axis) output << robot.com_offset[axis];
  output << YAML::EndSeq << YAML::Key << "reference_joints" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int index = 0; index < 12; ++index) output << robot.reference_joints[index];
  output << YAML::EndSeq << YAML::EndMap;
  PrintOptional(output, "kinematics_valid", result.kinematics_valid);
  PrintOptional(output, "ik_failed_leg", result.ik_failed_leg);
  PrintOptional(output, "ik_failed_time", result.ik_failed_time);
  output << YAML::Key << "ik_checks" << YAML::Value << result.ik_checks
         << YAML::Key << "max_ik_error" << YAML::Value << result.max_ik_error;
  output << YAML::Key << "variable_count" << YAML::Value << result.variable_count
         << YAML::Key << "margin_variable_count" << YAML::Value << result.margin_variable_count
         << YAML::Key << "constraint_count" << YAML::Value << result.constraint_count;
  PrintGroups(output, result.constraint_groups);
  if (result.terrain_validation.has_value()) {
    const auto& validation = *result.terrain_validation;
    output << YAML::Key << "terrain_validation" << YAML::Value << YAML::BeginMap
           << YAML::Key << "passed" << YAML::Value << validation.passed
           << YAML::Key << "reason" << YAML::Value << validation.reason
           << YAML::Key << "max_contact_error" << YAML::Value << validation.max_contact_error
           << YAML::Key << "min_foot_distance" << YAML::Value << validation.min_foot_distance
           << YAML::EndMap;
  }
  if (show_profile) {
    const auto& p = result.profile;
    output << YAML::Key << "profile" << YAML::Value << YAML::BeginMap
           << YAML::Key << "orientation_limit_radians" << YAML::Value << settings.orientation_limit
           << YAML::Key << "pitch_weight" << YAML::Value << settings.pitch_weight
           << YAML::Key << "foothold_backward_weight" << YAML::Value << settings.foothold_backward_weight
           << YAML::Key << "leg_vertical_weight" << YAML::Value << settings.leg_vertical_weight;
    for (const auto& item : {
             std::make_pair("full_evaluation_seconds", p.full_evaluation_seconds),
             {"values_evaluation_seconds", p.values_evaluation_seconds},
             {"constraint_jacobian_seconds", p.constraint_jacobian_seconds},
             {"svd_seconds", p.svd_seconds},
             {"qp_prepare_seconds", p.qp_prepare_seconds},
             {"qp_setup_seconds", p.qp_setup_seconds},
             {"qp_update_seconds", p.qp_update_seconds},
             {"qp_warm_start_seconds", p.qp_warm_start_seconds},
             {"qp_solve_seconds", p.qp_solve_seconds},
             {"qp_cleanup_seconds", p.qp_cleanup_seconds},
             {"initialization_seconds", p.initialization_seconds},
             {"ik_seconds", p.ik_seconds},
             {"final_validation_seconds_inclusive", p.final_validation_seconds},
             {"projection_seconds_inclusive", p.projection_seconds},
             {"line_search_seconds_inclusive", p.line_search_seconds}}) {
      output << YAML::Key << item.first << YAML::Value << item.second;
    }
    for (const auto& item : {
             std::make_pair("full_evaluations", p.full_evaluations),
             {"values_evaluations", p.values_evaluations},
             {"constraint_jacobian_evaluations", p.constraint_jacobian_evaluations},
             {"svd_calls", p.svd_calls}, {"qp_calls", p.qp_calls},
             {"qp_setups", p.qp_setups}, {"qp_updates", p.qp_updates},
             {"qp_warm_starts", p.qp_warm_starts}}) {
      output << YAML::Key << item.first << YAML::Value << item.second;
    }
    output << YAML::Key << "qp_iterations" << YAML::Value << p.qp_iterations
           << YAML::Key << "qp_iterations_per_solve" << YAML::Value << YAML::Flow
           << YAML::BeginSeq;
    for (int iterations : p.qp_iterations_per_solve) output << iterations;
    output << YAML::EndSeq;
    output << YAML::Key << "realtime_iterations" << YAML::Value << settings.realtime_iterations;
    output << YAML::Key << "sample_intervals" << YAML::Value << settings.sample_intervals;
    output << YAML::Key << "chord_projection" << YAML::Value << settings.chord_projection;
    output << YAML::Key << "projection_l2" << YAML::Value << settings.projection_l2;
    output << YAML::Key << "polynomial_tail" << YAML::Value << settings.polynomial_tail;
    output << YAML::Key << "block_assembly" << YAML::Value << settings.block_assembly;
    output << YAML::Key << "structured_equalities" << YAML::Value << settings.structured_equalities;
    output << YAML::Key << "projection_residual_tolerance" << YAML::Value << settings.projection_residual_tolerance;
    output << YAML::Key << "projection_rank_tolerance" << YAML::Value << settings.projection_rank_tolerance;
    output << YAML::Key << "equality_rank_tolerance" << YAML::Value << settings.equality_rank_tolerance;
    output << YAML::Key << "qp_working_set" << YAML::Value << settings.qp_working_set;
    output << YAML::Key << "lapack_svd" << YAML::Value << settings.lapack_svd;
    output << YAML::Key << "qp_settings" << YAML::Value << YAML::BeginMap
           << YAML::Key << "reuse_mode" << YAML::Value << b2_terrain_planner::QpReuseModeName(settings.qp_reuse)
           << YAML::Key << "regularization" << YAML::Value << settings.qp_regularization
           << YAML::Key << "eps_abs" << YAML::Value << settings.qp_eps_abs
           << YAML::Key << "eps_rel" << YAML::Value << settings.qp_eps_rel
           << YAML::Key << "max_iterations" << YAML::Value << settings.qp_max_iterations
           << YAML::EndMap;
    output << YAML::Key << "qp_diagnostics" << YAML::Value << YAML::BeginSeq;
    for (const auto& d : p.qp_diagnostics) {
      output << YAML::BeginMap
             << YAML::Key << "sqp_iteration" << YAML::Value << d.sqp_iteration
             << YAML::Key << "kind" << YAML::Value << d.kind
             << YAML::Key << "status" << YAML::Value << d.status
             << YAML::Key << "accepted" << YAML::Value << d.accepted
             << YAML::Key << "acceptance_reason" << YAML::Value << d.acceptance_reason;
      output << YAML::Key << "workspace_reused" << YAML::Value << d.workspace_reused
             << YAML::Key << "workspace_reason" << YAML::Value << d.workspace_reason
             << YAML::Key << "warm_start_used" << YAML::Value << d.warm_start_used
             << YAML::Key << "warm_start_reason" << YAML::Value << d.warm_start_reason
             << YAML::Key << "reused_duals" << YAML::Value << d.reused_duals
             << YAML::Key << "variables" << YAML::Value << d.variables
             << YAML::Key << "rows" << YAML::Value << d.rows;
      PrintOptional(output, "update_return", d.update_return);
      output << YAML::Key << "working_set_pass" << YAML::Value << d.working_set_pass
             << YAML::Key << "working_set_total_rows" << YAML::Value << d.working_set_total_rows;
      PrintOptional(output, "working_set_omitted_violation", d.working_set_omitted_violation);
      PrintOptional(output, "working_set_full_violation", d.working_set_full_violation);
      PrintOptional(output, "update_seconds", d.update_seconds);
      PrintOptional(output, "warm_start_return", d.warm_start_return);
      PrintOptional(output, "warm_start_seconds", d.warm_start_seconds);
      PrintOptional(output, "warm_start_roundtrip_error", d.warm_start_roundtrip_error);
      PrintOptional(output, "warm_start_projection_error", d.warm_start_projection_error);
      PrintOptional(output, "setup_return", d.setup_return);
      PrintOptional(output, "solve_return", d.solve_return);
      PrintOptional(output, "status_code", d.status_code);
      PrintOptional(output, "iterations", d.iterations);
      PrintOptional(output, "primal_residual", d.primal_residual);
      PrintOptional(output, "dual_residual", d.dual_residual);
      PrintOptional(output, "polish_status", d.polish_status);
      PrintOptional(output, "rho_updates", d.rho_updates);
      PrintOptional(output, "setup_seconds", d.setup_seconds);
      PrintOptional(output, "solve_seconds", d.solve_seconds);
      output << YAML::EndMap;
    }
    output << YAML::EndSeq << YAML::EndMap;
  }
  output << YAML::Key << "terrain_attempts" << YAML::Value << YAML::BeginSeq;
  for (const auto& attempt : result.terrain_attempts) {
    output << YAML::BeginMap << YAML::Key << "leg" << YAML::Value << attempt.leg
           << YAML::Key << "seed" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << attempt.seed.x() << attempt.seed.y() << attempt.seed.z() << YAML::EndSeq
           << YAML::Key << "status" << YAML::Value << attempt.status
           << YAML::Key << "elapsed" << YAML::Value << attempt.elapsed
           << YAML::Key << "success" << YAML::Value << attempt.success << YAML::EndMap;
  }
  output << YAML::EndSeq;
  if (result.success) {
    output << YAML::Key << "footholds" << YAML::Value << YAML::BeginSeq;
    for (int leg = 0; leg < 4; ++leg) output << YAML::Flow << YAML::BeginSeq << result.footholds(leg, 0)
        << result.footholds(leg, 1) << result.footholds(leg, 2) << YAML::EndSeq;
    output << YAML::EndSeq << YAML::Key << "segments" << YAML::Value << YAML::BeginSeq;
    for (size_t k = 0; k < result.phases.size(); ++k) {
      output << YAML::BeginMap << YAML::Key << "duration" << YAML::Value << result.phases[k].duration
             << YAML::Key << "margin" << YAML::Value << result.margins[k]
             << YAML::Key << "contacts" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (bool contact : result.phases[k].contacts) output << contact;
      output << YAML::EndSeq << YAML::Key << "landed" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (bool landed : result.phases[k].landed) output << landed;
      output << YAML::EndSeq << YAML::Key << "coefficients" << YAML::Value << YAML::BeginSeq;
      for (int axis = 0; axis < 6; ++axis) {
        output << YAML::Flow << YAML::BeginSeq;
        for (int power = 0; power < 5; ++power) output << result.coefficients[k](axis, power);
        output << YAML::EndSeq;
      }
      output << YAML::EndSeq << YAML::EndMap;
    }
    output << YAML::EndSeq;
  }
  output << YAML::EndMap;
  std::cout << output.c_str() << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  b2_terrain_planner::Robot robot;
  b2_terrain_planner::Input input;
  b2_terrain_planner::Settings settings;
  bool show_profile = false;
  std::string warm_path;
  try {
    InitializeB2(&robot, &input);
    for (int i = 1; i < argc; ++i) {
      std::string option = argv[i];
      if (option == "--help") {
        std::cout << "tamols_solve [--input YAML] [--speed M/S] [--gait-time S] "
                     "[--time-limit S] [--iterations N] [--fixed-feet] "
                     "[--profile] [--full-evaluation] "
                     "[--warm-start PLAN.yaml] "
                     "[--orientation-limit-deg VALUE] [--pitch-weight VALUE] [--foothold-backward-weight VALUE] [--leg-vertical-weight VALUE] [--vz-weight VALUE] [--fast-replanning] [--qp-reuse cold|reuse|warm] [--qp-regularization VALUE] [--svd jacobi|lapack] [--qp-working-set] [--structured-equalities] [--block-assembly] [--polynomial-tail|--hermite-tail] [--projection-l2] [--chord-projection] [--realtime-iterations N] [--sample-intervals N] [--equality-rank-tolerance TOL] [--projection-rank-tolerance TOL] [--projection-residual-tolerance TOL] "
                     "[--qp-eps-abs VALUE] [--qp-eps-rel VALUE] [--qp-max-iterations N]\n"
                     "B2 평지·지형 스냅샷의 독립 계획 계산. ROS 명령이나 모터 출력을 발행하지 않습니다.\n";
        return 0;
      }
      if (option == "--profile") {
        show_profile = true;
        continue;
      }
      if (option == "--fast-replanning") {
        settings.qp_reuse = b2_terrain_planner::QpReuseMode::kWarmStart;
        settings.qp_working_set = true;
        settings.structured_equalities = true;
        settings.block_assembly = true;
        settings.polynomial_tail = true;
        settings.lapack_svd = true;
        settings.equality_rank_tolerance = 1e-10;
        settings.projection_rank_tolerance = 1e-9;
        settings.projection_residual_tolerance = 5e-7;
        settings.qp_regularization = 1.0;
        settings.realtime_iterations = 1;
        settings.sample_intervals = 12;
        settings.chord_projection = false;
        settings.projection_l2 = false;
        continue;
      }
      if (option == "--chord-projection") {
        settings.chord_projection = true;
        continue;
      }
      if (option == "--projection-l2") {
        settings.projection_l2 = true;
        continue;
      }
      if (option == "--polynomial-tail") {
        settings.polynomial_tail = true;
        continue;
      }
      if (option == "--hermite-tail") {
        settings.polynomial_tail = false;
        continue;
      }
      if (option == "--block-assembly") {
        settings.block_assembly = true;
        continue;
      }
      if (option == "--structured-equalities") {
        settings.structured_equalities = true;
        continue;
      }
      if (option == "--qp-working-set") {
        settings.qp_working_set = true;
        continue;
      }
      if (option == "--full-evaluation") {
        settings.full_evaluation = true;
        continue;
      }
      if (option == "--fixed-feet") {
        input.fixed.fill(true);
        input.nominal = input.feet.leftCols<2>();
        input.has_nominal = true;
        continue;
      }
      if (i + 1 >= argc) throw std::invalid_argument("missing option value: " + option);
      std::string value = argv[++i];
      if (option == "--input") {
        ReadInput(value, &robot, &input, &settings);
      }
      else if (option == "--warm-start") warm_path = value;
      else if (option == "--speed") input.command.x() = std::stod(value);
      else if (option == "--gait-time") {
        input.gait_time = std::stod(value);
      }
      else if (option == "--time-limit") settings.time_limit = std::stod(value);
      else if (option == "--realtime-iterations") settings.realtime_iterations = ReadIntOption(value);
      else if (option == "--sample-intervals") settings.sample_intervals = ReadIntOption(value);
      else if (option == "--iterations") settings.max_iterations = std::stoi(value);
      else if (option == "--orientation-limit-deg") {
        settings.orientation_limit = ReadDoubleOption(value) * std::acos(-1.0) / 180.0;
      }
      else if (option == "--pitch-weight") settings.pitch_weight = ReadDoubleOption(value);
      else if (option == "--foothold-backward-weight") settings.foothold_backward_weight = ReadDoubleOption(value);
      else if (option == "--leg-vertical-weight") settings.leg_vertical_weight = ReadDoubleOption(value);
      else if (option == "--vz-weight") settings.vz_weight = ReadDoubleOption(value);
      else if (option == "--qp-regularization") settings.qp_regularization = ReadDoubleOption(value);
      else if (option == "--qp-eps-abs") settings.qp_eps_abs = ReadDoubleOption(value);
      else if (option == "--qp-eps-rel") settings.qp_eps_rel = ReadDoubleOption(value);
      else if (option == "--qp-max-iterations") settings.qp_max_iterations = ReadIntOption(value);
      else if (option == "--projection-residual-tolerance") settings.projection_residual_tolerance = ReadDoubleOption(value);
      else if (option == "--projection-rank-tolerance") settings.projection_rank_tolerance = ReadDoubleOption(value);
      else if (option == "--equality-rank-tolerance") settings.equality_rank_tolerance = ReadDoubleOption(value);
      else if (option == "--svd") {
        if (value == "jacobi") settings.lapack_svd = false;
        else if (value == "lapack") settings.lapack_svd = true;
        else throw std::invalid_argument("unknown SVD mode: " + value);
      }
      else if (option == "--qp-reuse") {
        if (value == "cold") settings.qp_reuse = b2_terrain_planner::QpReuseMode::kNone;
        else if (value == "reuse") settings.qp_reuse = b2_terrain_planner::QpReuseMode::kWorkspace;
        else if (value == "warm") settings.qp_reuse = b2_terrain_planner::QpReuseMode::kWarmStart;
        else throw std::invalid_argument("unknown QP reuse mode: " + value);
      }
      else throw std::invalid_argument("unknown option: " + option);
    }
    std::optional<b2_terrain_planner::Result> previous;
    const b2_terrain_planner::Result* previous_pointer = nullptr;
    if (!warm_path.empty()) {
      previous = ReadPlan(warm_path);
      previous_pointer = &*previous;
    }
    auto result = b2_terrain_planner::Solve(robot, input, settings, previous_pointer);
    Print(result, show_profile, settings, robot, input);
    if (result.success) return 0;
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
