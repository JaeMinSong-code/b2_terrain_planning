/**
 * @file terrain_planner_node.cpp
 * @brief elevation map + WBC 상태를 구독해 TAMOLS 계획을 LocomotionPlan으로 발행하는 ROS2 노드.
 * @version 0.0.47
 * @update 2026-11-05: 원본·보정 지도와 단계별 착지점 진단을 solve_status로 발행함.
 * @update 2026-11-04: 앞/뒤 좌우 발 쌍의 Y 간격에만 근접 지수 비용을 적용함.
 *
 * 모든 계산은 odom 프레임에서 수행한다. 발행 궤적(몸통/네 발)은 WBC 추종 모드가 소비한다.
 * 공통 센서 기반 CoM/발 상태를 초기 조건으로 사용하고 입력 만료를 검사한다.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"
#include "b2_terrain_planner/support_patch.h"
#include "b2_terrain_planner/map_datum.h"
#include "b2_terrain_planner/swing_reference.h"
#include "b2_terrain_planner/committed_swing.h"

#include <algorithm>
#include <array>
#include <deque>
#include <cmath>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <limits>
#include <optional>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include "b2_terrain_planner/msg/locomotion_plan.hpp"
#include "b2_terrain_planner/msg/locomotion_state.hpp"

namespace {
using namespace b2_terrain_planner;

// ZYX(yaw,pitch,roll) 쿼터니언 변환.
Eigen::Vector3d QuatToZyx(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  Eigen::Vector3d ypr;
  ypr[0] = std::atan2(r(1, 0), r(0, 0));
  ypr[1] = std::atan2(-r(2, 0), std::hypot(r(2, 1), r(2, 2)));
  ypr[2] = std::atan2(r(2, 1), r(2, 2));
  return ypr;  // yaw, pitch, roll
}

Eigen::Quaterniond ZyxToQuat(double yaw, double pitch, double roll) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}
}  // namespace

class TerrainPlannerNode : public rclcpp::Node {
 public:
  TerrainPlannerNode() : rclcpp::Node("terrain_planner_node") {
    state_topic_ = declare_parameter<std::string>("state_topic", "/robot/locomotion_state");
    state_frame_ = declare_parameter<std::string>("state_frame", "odom");
    state_max_age_ = declare_parameter<double>("state_max_age", 0.25);
    map_max_age_ = declare_parameter<double>("map_max_age", 1.0);
    if (!std::isfinite(state_max_age_) || state_max_age_ <= 0.0 ||
        !std::isfinite(map_max_age_) || map_max_age_ <= 0.0) {
      throw std::invalid_argument("state_max_age/map_max_age must be positive and finite");
    }
    map_topic_ = declare_parameter<std::string>(
        "map_topic", "/elevation_mapping_node/elevation_map_raw");
    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/terrain_planner/plan");
    frame_ = declare_parameter<std::string>("frame", "odom");
    plan_rate_hz_ = declare_parameter<double>("plan_rate_hz", 10.0);
    sample_dt_ = declare_parameter<double>("sample_dt", 0.02);
    // 핸드오버 마감시각·착지 대기 재개 유예. 계획이 상태 스냅샷 후 이 시간 안에
    // 도착한다고 약속하는 값으로, 실측 전달 시간(~20-75ms)보다 커야 한다.
    // 0 이하이면 기존 보수 공식(time_limit + 2*sample_dt)을 쓴다.
    handover_grace_ = declare_parameter<double>("handover_grace", -1.0);
    if (std::isnan(handover_grace_)) {
      throw std::invalid_argument("handover_grace must not be NaN");
    }
    swing_height_ = declare_parameter<double>("swing_height", 0.10);
    step_swing_.enabled = declare_parameter<bool>("step_swing.enabled", true);
    step_swing_.proximity = declare_parameter<double>("step_swing.proximity", 0.05);
    step_swing_.height_threshold =
        declare_parameter<double>("step_swing.height_threshold", 0.05);
    step_swing_.retract_distance =
        declare_parameter<double>("step_swing.retract_distance", 0.05);
    for (double value : {step_swing_.proximity, step_swing_.height_threshold,
                         step_swing_.retract_distance}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("step_swing distances must be finite and positive");
      }
    }
    // 구형 발 반경은 착지·패치·스윙·datum의 중심-지면 변환에 공통 사용한다.
    swing_terrain_clearance_ = declare_parameter<double>("swing_terrain_clearance", 0.03);
    foot_radius_ = declare_parameter<double>("foot_radius", kDefaultFootRadius);
    settings_.terrain.foot_radius = foot_radius_;
    require_map_ = declare_parameter<bool>("require_map", true);
    support_patch_enabled_ = declare_parameter<bool>("support_patch.enabled", false);
    support_patch_half_width_ = declare_parameter<double>("support_patch.half_width", 0.025);
    if (!std::isfinite(support_patch_half_width_) || support_patch_half_width_ <= 0) {
      throw std::invalid_argument("support_patch.half_width must be positive and finite");
    }
    support_patch_max_age_sec_ =
        declare_parameter<double>("support_patch.max_age_sec", 2.0);
    if (std::isnan(support_patch_max_age_sec_) || support_patch_max_age_sec_ <= 0) {
      throw std::invalid_argument("support_patch.max_age_sec must be positive");
    }
    // 지도 z 기준(datum) 재정착: 확인 지지발 기준으로 지도 전체를 수직 이동해 odom z
    // 드리프트를 상쇄한다. 지형 상대 모양은 보존하고(평탄화 아님) 전역 오프셋만 적용한다.
    // 오프셋은 지도 갱신 틱에만 변하고 solve 사이에는 상수다(line search 안정).
    map_datum_reanchor_enabled_ =
        declare_parameter<bool>("map_datum_reanchor.enabled", false);
    map_datum_max_rate_ =
        declare_parameter<double>("map_datum_reanchor.max_rate_mps", 0.01);
    if (!std::isfinite(map_datum_max_rate_) || map_datum_max_rate_ <= 0 ||
        !std::isfinite(foot_radius_)) {
      throw std::invalid_argument("invalid map_datum_reanchor parameters");
    }
    const auto cmd = declare_parameter<std::vector<double>>(
        "command", std::vector<double>{0.1, 0.0, 0.0});
    if (cmd.size() != 3) throw std::invalid_argument("command must have three components");
    command_ = Eigen::Vector3d(cmd[0], cmd[1], cmd[2]);
    if (!command_.allFinite()) throw std::invalid_argument("command must be finite");
    command_topic_ = declare_parameter<std::string>("command_topic", "");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0) {
      throw std::invalid_argument("command_timeout must be positive and finite");
    }
    command_linear_accel_ = declare_parameter<double>("command_ramp.linear_accel", 0.5);
    command_angular_accel_ = declare_parameter<double>("command_ramp.angular_accel", 1.0);
    if (!std::isfinite(command_linear_accel_) || command_linear_accel_ <= 0.0 ||
        !std::isfinite(command_angular_accel_) || command_angular_accel_ <= 0.0) {
      throw std::invalid_argument("command_ramp acceleration must be positive and finite");
    }
    // 위치 유지(station keeping): 명령 0 유지 중 측정 CoM의 앵커 이탈을 속도 명령으로
    // 환원한다. 최적화에 몸통 위치 추종 항이 없어 생기는 제자리 random walk를 막는다.
    station_keeping_enabled_ = declare_parameter<bool>("station_keeping.enabled", false);
    station_keeping_gain_ = declare_parameter<double>("station_keeping.gain", 0.5);
    station_keeping_max_velocity_ =
        declare_parameter<double>("station_keeping.max_velocity", 0.06);
    if (!std::isfinite(station_keeping_gain_) || station_keeping_gain_ < 0.0 ||
        !std::isfinite(station_keeping_max_velocity_) ||
        station_keeping_max_velocity_ < 0.0) {
      throw std::invalid_argument(
          "station_keeping gain/max_velocity must be non-negative and finite");
    }
    effective_command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "~/effective_command", rclcpp::QoS(1));
    if (!command_topic_.empty()) {
      command_.setZero();
      command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          command_topic_, rclcpp::QoS(1).reliable(),
          [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            OnCommand(*msg);
          });
    }

    InitRobot();
    ConfigureSettings();

    state_sub_ = create_subscription<b2_terrain_planner::msg::LocomotionState>(
        state_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](b2_terrain_planner::msg::LocomotionState::SharedPtr msg) { OnState(*msg); });
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        map_topic_, rclcpp::QoS(1).best_effort(),
        [this](grid_map_msgs::msg::GridMap::SharedPtr msg) { OnMap(*msg); });
    corrected_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "/terrain_planner/elevation_map_corrected",
        rclcpp::QoS(1).reliable().transient_local());
    plan_pub_ = create_publisher<b2_terrain_planner::msg::LocomotionPlan>(plan_topic_, 1);
    solve_status_pub_ = create_publisher<std_msgs::msg::String>("~/solve_status", 10);
    pipeline_status_pub_ = create_publisher<std_msgs::msg::String>("~/pipeline_status", 100);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("marker_topic", "/terrain_planner/markers"), 1);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, plan_rate_hz_)),
        [this] { OnTimer(); });
    completion_timer_ = create_wall_timer(
        std::chrono::milliseconds(2), [this] { CompleteSolve(); });
    RCLCPP_INFO(get_logger(), "terrain_planner_node: state=%s map=%s plan=%s rate=%.1fHz",
                state_topic_.c_str(), map_topic_.c_str(), plan_topic_.c_str(), plan_rate_hz_);
  }

 private:
  friend struct TerrainPlannerRetryTest;
  using State = b2_terrain_planner::msg::LocomotionState;
  struct PendingSolve {
    Input input;
    State state;
    builtin_interfaces::msg::Time map_stamp;
    std::uint64_t generation = 0;
    bool touchdown_stop = false;
    std::optional<HeightMap> swing_terrain;
    double diagnostic_datum_offset = 0.0;
  };

  struct PublishedPlan {
    std::int64_t stamp_ns = 0;
    Result body;
    std::array<SwingReference, 4> swings;
  };

  const PublishedPlan* ActivePlan() const {
    if (!state_) return nullptr;
    for (const auto& plan : published_plans_) {
      if (plan.stamp_ns == state_->active_plan_stamp_ns) return &plan;
    }
    return nullptr;
  }

  static double StateBodyTime(const State& state) {
    if (state.body_clock_valid) return state.body_time;
    return state.gait_time;
  }

  void UpdateCommandRamp(const State& state) {
    double time = rclcpp::Time(state.header.stamp).seconds();
    if (state.body_clock_valid) time = state.body_time;
    if (!command_ramp_time_) {
      command_ramp_time_ = time;
      return;
    }
    const double dt = time - *command_ramp_time_;
    if (dt <= 0.0) return;
    command_ramp_time_ = time;
    // 착지 미확인의 즉시 0 목표는 유지하고, 실제 보행 정지 중에는 램프도 감속한다.
    Eigen::Vector3d target = EffectiveCommand();
    if (state.gait_clock_valid && state.gait_paused) target.setZero();
    const Eigen::Vector2d delta = target.head<2>() - applied_command_.head<2>();
    const double distance = delta.norm();
    const double step = command_linear_accel_ * dt;
    if (distance > step) {
      applied_command_.head<2>() += delta * (step / distance);
    } else {
      applied_command_.head<2>() = target.head<2>();
    }
    const double yaw_step = command_angular_accel_ * dt;
    applied_command_.z() += std::clamp(target.z() - applied_command_.z(),
                                     -yaw_step, yaw_step);
  }
  // 예정 착지가 확인되지 않으면 기존 TAMOLS 속도 목표만 0으로 낮춘다.
  bool UpdateTouchdownStop(const State& state) {
    const bool was_stopping = touchdown_stop_;
    touchdown_stop_ = false;
    if (state.gait_clock_valid && state.body_clock_valid) {
      const auto phases = MakeHorizon(state.gait_time, robot_.period, robot_.duty);
      for (int leg = 0; leg < 4; ++leg) {
        if (phases.front().contacts[leg] && !state.contact_confirmed[leg]) {
          touchdown_stop_ = true;
        }
      }
    }
    return was_stopping != touchdown_stop_;
  }

  Eigen::Vector3d PlanningCommand() const {
    if (touchdown_stop_) return Eigen::Vector3d::Zero();
    return applied_command_;
  }

  // 명령 0 유지 중 측정 CoM을 앵커 위치에 붙드는 보정 속도(heading 프레임 XY).
  // 사용자 XY 명령이 켜지면 앵커를 해제하고, 다시 0이 되면 그 자리에서 재설정한다.
  // 착지 대기(touchdown_stop)의 즉시 0 목표는 유지하되 앵커는 보존한다.
  Eigen::Vector2d StationKeepingCorrection() {
    if (!station_keeping_enabled_ || !state_ ||
        applied_command_.head<2>().norm() > 1e-3) {
      hold_anchor_.reset();
      return Eigen::Vector2d::Zero();
    }
    const Eigen::Vector2d com(state_->com_position.x, state_->com_position.y);
    if (!com.allFinite()) return Eigen::Vector2d::Zero();
    if (!hold_anchor_) hold_anchor_ = com;
    if (touchdown_stop_) return Eigen::Vector2d::Zero();
    Eigen::Vector2d correction = station_keeping_gain_ * (*hold_anchor_ - com);
    const double norm = correction.norm();
    if (norm > station_keeping_max_velocity_) {
      correction *= station_keeping_max_velocity_ / norm;
    }
    const Eigen::Quaterniond q(state_->base_orientation.w, state_->base_orientation.x,
                               state_->base_orientation.y, state_->base_orientation.z);
    const double yaw = QuatToZyx(q.normalized())[0];
    return Eigen::Rotation2Dd(-yaw) * correction;
  }

  void DiscardRetrySeed() {
    retry_seed_.reset();
    retry_seed_failures_ = 0;
    retry_seed_violation_ = std::numeric_limits<double>::infinity();
  }

  // 단일 executor에서 관찰한 원래 판정 결과만 기록한다. 진단은 재계획을 유발하지 않는다.
  void PublishPipelineStatus(const char* event, const char* reason, int leg = -1,
                             std::int64_t received_stamp_ns = 0) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"event\":" << std::quoted(event)
        << ",\"reason\":" << std::quoted(reason)
        << ",\"event_seq\":" << ++pipeline_event_seq_
        << ",\"attempt_seq\":" << attempt_seq_
        << ",\"ros_ns\":" << now().nanoseconds()
        << ",\"received_stamp_ns\":" << received_stamp_ns
        << ",\"solve_state_stamp_ns\":" << solve_state_stamp_ns_
        << ",\"ramped_command\":[" << applied_command_.x() << ','
        << applied_command_.y() << ',' << applied_command_.z() << ']'
        << ",\"active_plan_stamp_ns\":" << (state_.value_or(State{}).active_plan_stamp_ns)
        << ",\"retry_requested\":" << std::boolalpha << retry_requested_
        << ",\"last_planned_ns\":" << last_planned_ns_
        << ",\"last_published_stamp_ns\":" << last_published_stamp_ns_
        << ",\"last_publish_ns\":" << last_publish_ns_
        << ",\"solve_start_ns\":" << solve_start_ns_
        << ",\"solve_end_ns\":" << solve_end_ns_
        << ",\"map_stamp_ns\":" << rclcpp::Time(map_stamp_).nanoseconds()
        << ",\"leg\":" << leg
        << ",\"touchdown_stop\":" << std::boolalpha << touchdown_stop_;
    const Eigen::Vector3d command = PlanningCommand();
    out << ",\"effective_command\":[" << command.x() << ',' << command.y() << ',' << command.z() << ']';
    if (state_) {
      out << ",\"state_stamp_ns\":" << rclcpp::Time(state_->header.stamp).nanoseconds()
          << ",\"gait_epoch_ns\":" << state_->gait_epoch_ns
          << ",\"gait_time\":";
      if (std::isfinite(state_->gait_time)) out << state_->gait_time;
      else out << "null";
      out << ",\"gait_paused\":" << std::boolalpha << state_->gait_paused
          << ",\"body_time\":" << StateBodyTime(*state_);
    }
    if (pending_solve_) {
      const auto& pending = *pending_solve_;
      out << ",\"solve_touchdown_stop\":" << pending.touchdown_stop
          << ",\"solve_command\":[" << pending.input.command.x() << ','
          << pending.input.command.y() << ',' << pending.input.command.z() << ']'
          << ",\"solve_contact_confirmed\":[";
      for (int i = 0; i < 4; ++i) {
        if (i > 0) out << ',';
        out << pending.state.contact_confirmed[i];
      }
      out << ']';
    }
    out << '}';
    std_msgs::msg::String message;
    message.data = out.str();
    pipeline_status_pub_->publish(message);
    // 결과 이벤트는 전부 저장한다. 반복 보류만 같은 이유/계산/부모에서 1초마다 요약한다.
    const std::string key = std::string(event) + ":" + reason + ":" +
        std::to_string(attempt_seq_) + ":" +
        std::to_string(state_.value_or(State{}).active_plan_stamp_ns);
    const auto log_time = std::chrono::steady_clock::now();
    const bool waiting = std::string(event) == "deferred" || std::string(event) == "state_rejected";
    if (!waiting || key != last_pipeline_log_key_ ||
        log_time - last_pipeline_log_time_ >= std::chrono::seconds(1)) {
      RCLCPP_DEBUG(get_logger(), "TAMOLS_PIPELINE %s", message.data.c_str());
      last_pipeline_log_key_ = key;
      last_pipeline_log_time_ = log_time;
    }
  }

  void PublishSolveStatus(const Result& result, const PendingSolve& pending,
                          bool published, const char* publication_reason) {
    const auto& input = pending.input;
    const auto& stamp = pending.state.header.stamp;
    const auto number = [](double value) {
      if (!std::isfinite(value)) return std::string("null");
      std::ostringstream stream;
      stream << std::setprecision(12) << value;
      return stream.str();
    };
    // solver 밖 예외 등 단계가 없는 결과는 미진입으로 단정하지 않는다.
    const char* optimization_started = "null";
    if (result.stage != "not_started") {
      optimization_started = "false";
      if (result.optimization_started) optimization_started = "true";
    }
    std::string worst;
    double worst_violation = 0;
    for (const auto& group : result.constraint_groups) {
      if (group.max_violation > worst_violation) {
        worst = group.name;
        worst_violation = group.max_violation;
      }
    }
    std::ostringstream out;
    out << std::boolalpha << "{\"status\":" << std::quoted(result.status)
        << ",\"success\":" << result.success << ",\"published\":" << published
        << ",\"stage\":" << std::quoted(result.stage)
        << ",\"optimization_started\":" << optimization_started
        << ",\"publication_reason\":" << std::quoted(publication_reason)
        << ",\"touchdown_stop\":" << pending.touchdown_stop
        << ",\"parent_plan_stamp_ns\":" << pending.state.active_plan_stamp_ns
        << ",\"prefix_duration\":" << number(input.prefix_duration)
        << ",\"attempt_seq\":" << attempt_seq_
        << ",\"state_stamp_ns\":" << rclcpp::Time(stamp).nanoseconds()
        << ",\"solve_start_ns\":" << solve_start_ns_
        << ",\"solve_end_ns\":" << solve_end_ns_
        << ",\"last_publish_ns\":" << last_publish_ns_
        << ",\"elapsed\":" << number(result.elapsed)
        << ",\"terrain_processing_seconds\":" << number(result.terrain_processing_seconds)
        << ",\"terrain_initialization_seconds\":" << number(result.terrain_initialization_seconds)
        << ",\"terrain_initialization_status\":" << std::quoted(result.terrain_initialization_status)
        << ",\"iterations\":" << result.iterations
        << ",\"violation\":" << number(result.violation)
        << ",\"worst_constraint\":" << std::quoted(worst)
        << ",\"worst_violation\":" << number(worst_violation)
        << ",\"warm_start\":" << std::quoted(result.warm_start.reason)
        << ",\"state_age\":" << number((now() - rclcpp::Time(stamp)).seconds())
        << ",\"gait_time\":" << number(input.gait_time)
        << ",\"body_time\":" << number(BodyTime(input))
        << ",\"gait_hold_duration\":" << number(input.gait_hold_duration)
        << ",\"command\":[" << number(input.command.x()) << ','
        << number(input.command.y()) << ',' << number(input.command.z()) << ']'
        << ",\"foot_z\":[";
    for (int leg = 0; leg < 4; ++leg) {
      if (leg > 0) out << ',';
      out << number(input.feet(leg, 2));
    }
    out << "],\"contact_confirmed\":[";
    for (int leg = 0; leg < 4; ++leg) {
      if (leg > 0) out << ',';
      out << pending.state.contact_confirmed[leg];
    }
    out << "],\"contact_estimated\":[";
    for (int leg = 0; leg < 4; ++leg) {
      if (leg > 0) out << ',';
      out << pending.state.contact_estimated[leg];
    }
    out << "],\"terrain_attempts\":[";
    for (std::size_t i = 0; i < result.terrain_attempts.size(); ++i) {
      if (i > 0) out << ',';
      const auto& attempt = result.terrain_attempts[i];
      out << "{\"leg\":" << attempt.leg << ",\"status\":" << std::quoted(attempt.status)
          << ",\"success\":" << attempt.success << ",\"elapsed\":" << number(attempt.elapsed) << '}';
    }
    out << "],\"foothold_diagnostics\":{\"schema_version\":1,\"enabled\":"
        << settings_.foothold_diagnostics << ",\"frame\":" << std::quoted(input.frame)
        << ",\"map_stamp_ns\":" << rclcpp::Time(pending.map_stamp).nanoseconds()
        << ",\"datum_offset\":" << number(pending.diagnostic_datum_offset)
        << ",\"capture_seconds\":" << number(result.foothold_diagnostic_seconds);
    if (input.terrain) {
      out << ",\"map_origin\":[" << number(input.terrain->origin.x()) << ','
          << number(input.terrain->origin.y()) << "],\"resolution\":"
          << number(input.terrain->resolution);
    }
    out << ",\"cost_names\":[";
    for (std::size_t k = 0; k < kFootholdCostNames.size(); ++k) {
      if (k > 0) out << ',';
      out << std::quoted(kFootholdCostNames[k]);
    }
    out << "],\"candidate_cost_names\":[\"nominal\",\"backward\",\"edge\","
           "\"occlusion\",\"curvature\",\"virtual_height\",\"separation\"],\"stages\":[";
    for (std::size_t k = 0; k < result.foothold_trace.size(); ++k) {
      if (k > 0) out << ',';
      const auto& trace = result.foothold_trace[k];
      out << "{\"stage\":" << std::quoted(trace.stage) << ",\"layer\":" << std::quoted(trace.layer)
          << ",\"attempt_leg\":" << trace.attempt_leg << ",\"iteration\":" << trace.iteration
          << ",\"crossing_leg\":" << trace.crossing_leg
          << ",\"sqp_valid\":" << trace.sqp_valid << ",\"sqp_cost\":";
      if (trace.sqp_valid) out << number(trace.total_cost);
      else out << "null";
      out << ",\"sqp_cost_terms\":[";
      for (std::size_t j = 0; j < trace.sqp_costs.size(); ++j) {
        if (j > 0) out << ',';
        if (trace.sqp_valid) out << number(trace.sqp_costs[j]);
        else out << "null";
      }
      out << "],\"violation\":";
      if (trace.sqp_valid) out << number(trace.violation);
      else out << "null";
      if (trace.crossing_leg >= 0) {
        out << ",\"merit\":" << number(trace.merit)
            << ",\"scaled_violation\":" << number(trace.scaled_violation)
            << ",\"alpha\":" << number(trace.alpha)
            << ",\"penalty\":" << number(trace.penalty);
      } else {
        out << ",\"merit\":null,\"scaled_violation\":null,\"alpha\":null,\"penalty\":null";
      }
      out << ",\"feet\":[";
      for (int leg = 0; leg < 4; ++leg) {
        if (leg > 0) out << ',';
        const auto& point = trace.terrain[leg];
        out << "{\"leg\":" << leg << ",\"source\":" << std::quoted(trace.sources[leg])
            << ",\"xyz\":[" << number(trace.feet(leg, 0)) << ',' << number(trace.feet(leg, 1))
            << ',' << number(trace.feet(leg, 2)) << "],\"map_valid\":" << point.valid
            << ",\"height\":";
        if (point.valid) out << number(point.height);
        else out << "null";
        out << ",\"occlusion\":";
        if (point.valid) out << number(point.occlusion);
        else out << "null";
        out << ",\"candidate_cost_valid\":" << trace.candidate_valid[leg]
            << ",\"candidate_costs\":[";
        for (int j = 0; j < 7; ++j) {
          if (j > 0) out << ',';
          if (trace.candidate_valid[leg]) out << number(trace.candidate_costs[leg][j]);
          else out << "null";
        }
        out << "],\"cells\":[";
        if (point.valid) {
          for (int j = 0; j < 4; ++j) {
            if (j > 0) out << ',';
            const int col = point.lower_cell.x() + j % 2;
            const int row = point.lower_cell.y() + j / 2;
            out << "{\"col\":" << col << ",\"row\":" << row
                << ",\"weight\":" << number(point.weights[j])
                << ",\"input_observed\":" << point.observed[j] << ",\"input_height\":";
            if (point.observed[j]) out << number(point.input_heights[j]);
            else out << "null";
            out << ",\"processed_height\":" << number(point.processed_heights[j])
                << ",\"sensor_observed\":";
            bool sensor_available = false;
            bool sensor_observed = false;
            if (pending.swing_terrain) {
              const auto& raw = *pending.swing_terrain;
              sensor_available = col >= 0 && row >= 0 && col < raw.heights.cols() &&
                  row < raw.heights.rows();
              if (sensor_available) {
                sensor_observed = raw.valid(row, col) && std::isfinite(raw.heights(row, col));
              }
            }
            if (sensor_available) out << sensor_observed;
            else out << "null";
            out << ",\"sensor_height\":";
            if (sensor_observed) {
              out << number(pending.swing_terrain->heights(row, col) - pending.diagnostic_datum_offset);
            } else out << "null";
            out << '}';
          }
        }
        out << "]}";
      }
      out << "]}";
    }
    out << "]}}";
    std_msgs::msg::String message;
    message.data = out.str();
    solve_status_pub_->publish(message);
    RCLCPP_DEBUG(get_logger(), "TAMOLS_SOLVE %s", message.data.c_str());
  }

  void OnCommand(const geometry_msgs::msg::TwistStamped& message) {
    const auto& v = message.twist;
    if (message.header.frame_id != "base_heading" ||
        !Fresh(message.header.stamp, command_timeout_) ||
        !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) ||
        !std::isfinite(v.linear.z) || !std::isfinite(v.angular.x) ||
        !std::isfinite(v.angular.y) || !std::isfinite(v.angular.z)) {
      command_.setZero();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Rejected invalid, stale, future or wrong-frame locomotion command");
      return;
    }
    if (command_stamp_ && rclcpp::Time(message.header.stamp) <=
                          rclcpp::Time(*command_stamp_)) return;
    command_stamp_ = message.header.stamp;
    command_ << v.linear.x, v.linear.y, v.angular.z;
  }

  Eigen::Vector3d EffectiveCommand() const {
    if (!command_topic_.empty() &&
        (!command_stamp_ || !Fresh(*command_stamp_, command_timeout_))) {
      return Eigen::Vector3d::Zero();
    }
    return command_;
  }

  void InitRobot() {
    auto model = std::make_shared<RobotModel>(DefaultModelPath(B2_BUILD_MODEL_PATH));
    ModelState state;
    for (int leg = 0; leg < 4; ++leg) {
      state.joints.segment<3>(3 * leg) << 0, 44.30 * M_PI / 180, -85.94 * M_PI / 180;
    }
    state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
    model->PopulatePlanner(state, &robot_, &nominal_input_);
    // CoM 기준 중립 발 배치의 XY 중심을 맞춰 명목 자세의 편심을 물려주지 않는다.
    // 실제 초기 발 위치와 운동학 기준 다리, 발 높이는 모델 값을 유지한다.
    const Eigen::RowVector2d neutral_center =
        robot_.nominal_offsets.leftCols<2>().colwise().mean();
    robot_.nominal_offsets.leftCols<2>().rowwise() -= neutral_center;
  }

  void ConfigureSettings() {
    // solve_main --fast-replanning 프리셋 + generate_stairs에서 검증된 실시간 QP 값.
    settings_.qp_reuse = QpReuseMode::kWarmStart;
    settings_.qp_working_set = true;
    settings_.structured_equalities = true;
    settings_.block_assembly = true;
    settings_.polynomial_tail = true;
    settings_.lapack_svd = true;
    settings_.equality_rank_tolerance = 1e-10;
    settings_.projection_rank_tolerance = 1e-9;
    settings_.projection_residual_tolerance = 5e-7;
    settings_.qp_regularization = 0.3;
    settings_.realtime_iterations = 2;
    settings_.sample_intervals = 6;
    settings_.qp_eps_abs = 1e-3;
    settings_.qp_eps_rel = 1e-3;
    settings_.qp_max_iterations = 4000;
    settings_.time_limit = declare_parameter<double>("time_limit", 0.1);
    settings_.foothold_diagnostics = declare_parameter<bool>("foothold_diagnostics", true);
    settings_.orientation_limit =
        declare_parameter<double>("orientation_limit_deg", 60.0) * M_PI / 180.0;
    settings_.pitch_weight = declare_parameter<double>("pitch_weight", 10.0);
    settings_.nominal_kinematics_weight =
        declare_parameter<double>("nominal_kinematics_weight", 7.0);
    settings_.foothold_backward_weight =
        declare_parameter<double>("foothold_backward_weight", 1.4);
    settings_.previous_foothold_weight =
        declare_parameter<double>("previous_foothold_weight", 0.01);
    settings_.vz_weight = declare_parameter<double>("vz_weight", 0.0);
    settings_.graduated_initialization = declare_parameter<bool>("terrain.graduated_initialization", true);
    settings_.graduated_budget_fraction = declare_parameter<double>("terrain.graduated_budget_fraction", 0.25);
    auto& terrain = settings_.terrain;
    terrain.pipeline_enabled = declare_parameter<bool>("terrain.pipeline_enabled", true);
    terrain.median_iterations = declare_parameter<int>("terrain.median_iterations", 1);
    terrain.smoothing_sigma = declare_parameter<double>("terrain.smoothing_sigma", 0.05);
    terrain.virtual_floor_sigma = declare_parameter<double>("terrain.virtual_floor_sigma", 0.2);
    terrain.virtual_floor_radius = declare_parameter<double>("terrain.virtual_floor_radius", 0.15);
    terrain.occlusion_weight = declare_parameter<double>("terrain.occlusion_weight", 10.0);
    terrain.curvature_weight = declare_parameter<double>("terrain.curvature_weight", 0.001);
    terrain.virtual_height_weight = declare_parameter<double>("terrain.virtual_height_weight", 100.0);
    terrain.body_alignment_weight = declare_parameter<double>("terrain.body_alignment_weight", 100.0);
    terrain.foot_separation_weight = declare_parameter<double>("terrain.foot_separation_weight", 0.0);
    terrain.foot_separation_distance_y = declare_parameter<double>("terrain.foot_separation_distance_y", 0.08);
    terrain.foot_separation_scale = declare_parameter<double>("terrain.foot_separation_scale", 0.01);

  }

  // grid_map_msgs를 직접 디코딩한다(grid_map_core의 전역 Eigen 플러그인 오염 회피).
  // 규약: grid_map 논리 index (i=row→-x, j=col→-y). 버퍼는 (outer,inner) start로 순환.
  static int LayerIndex(const grid_map_msgs::msg::GridMap& m, const std::string& name) {
    for (size_t k = 0; k < m.layers.size(); ++k) if (m.layers[k] == name) return static_cast<int>(k);
    return -1;
  }

  // xy(odom)에서 관측된 지도 높이를 최근접 셀로 읽는다. 미관측/범위 밖이면 false.
  // datum 재정착 오프셋 추정 전용(보간은 불필요, 관측 셀 하나면 충분).
  static bool SampleMapHeight(const HeightMap& hm, const Eigen::Vector2d& xy, double* out) {
    const Eigen::Vector2d idx = (xy - hm.origin) / hm.resolution;
    const int col = static_cast<int>(std::lround(idx.x()));
    const int row = static_cast<int>(std::lround(idx.y()));
    if (row < 0 || col < 0 || row >= hm.heights.rows() || col >= hm.heights.cols()) return false;
    if (hm.valid(row, col) == 0 || !std::isfinite(hm.heights(row, col))) return false;
    *out = hm.heights(row, col);
    return true;
  }

  // 확인 지지발(confirmed∧estimated)에서 median(foot_z − 지도높이 − 접촉 오프셋)을
  // 변화율 제한으로 추적한다. 지도 갱신 틱(OnMap)에만 호출해 solve 사이에는 오프셋이
  // 상수가 되게 한다. 적용은 solve 입력 복사본에만 하므로 저장 지도는 오염되지 않는다.
  void UpdateMapDatum() {
    if (!map_datum_reanchor_enabled_ || !terrain_ || !state_ ||
        !Fresh(state_->header.stamp, state_max_age_)) {
      return;
    }
    std::vector<double> diffs;
    for (int leg = 0; leg < 4; ++leg) {
      if (!(state_->contact_confirmed[leg] && state_->contact_estimated[leg])) continue;
      const auto& foot = state_->foot_position[leg];
      double mh;
      if (SampleMapHeight(*terrain_, Eigen::Vector2d(foot.x, foot.y), &mh)) {
        diffs.push_back(FootCenterToGroundZ(foot.z, foot_radius_) - mh);
      }
    }
    const std::int64_t stamp_ns = rclcpp::Time(map_stamp_).nanoseconds();
    double dt = 0.1;
    if (map_datum_stamp_ns_ != 0 && stamp_ns > map_datum_stamp_ns_) {
      dt = std::min(0.5, (stamp_ns - map_datum_stamp_ns_) * 1e-9);
    }
    map_datum_stamp_ns_ = stamp_ns;
    UpdateDatumOffset(&map_datum_, diffs, dt, map_datum_max_rate_);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Map datum re-anchor: offset %.4f m (planner input only)",
                        map_datum_.offset);
  }

  void RememberSupportContacts() {
    if (!support_patch_enabled_ || !terrain_ || !state_ ||
        !Fresh(state_->header.stamp, state_max_age_)) return;
    std::array<Eigen::Vector3d, 4> feet;
    std::array<bool, 4> contacts{};
    for (int leg = 0; leg < 4; ++leg) {
      const auto& foot = state_->foot_position[leg];
      feet[leg] << foot.x, foot.y, foot.z;
      contacts[leg] = state_->contact_confirmed[leg] && state_->contact_estimated[leg];
    }
    support_patch_memory_.Observe(feet, contacts, support_patch_half_width_,
                                  rclcpp::Time(state_->header.stamp).seconds(), foot_radius_);
  }

  void OnMap(const grid_map_msgs::msg::GridMap& msg) {
    if (msg.header.frame_id != frame_ || !Fresh(msg.header.stamp, map_max_age_)) return;
    const int ev = LayerIndex(msg, elevation_layer_);
    if (ev < 0 || static_cast<size_t>(ev) >= msg.data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "elevation layer '%s' missing", elevation_layer_.c_str());
      return;
    }
    const double res = msg.info.resolution;
    if (!std::isfinite(res) || !(res > 0.0)) return;
    const int gm_rows = static_cast<int>(std::round(msg.info.length_x / res));  // x축 셀 수
    const int gm_cols = static_cast<int>(std::round(msg.info.length_y / res));  // y축 셀 수
    const auto& fa = msg.data[ev];
    if (gm_rows < 3 || gm_cols < 3 ||
        static_cast<int>(fa.data.size()) != gm_rows * gm_cols) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unexpected grid_map layout");
      return;
    }
    const int var = LayerIndex(msg, "variance");
    if (var >= 0 && (static_cast<size_t>(var) >= msg.data.size() ||
                    msg.data[var].data.size() != fa.data.size())) return;
    const double cx = msg.info.pose.position.x, cy = msg.info.pose.position.y;
    const int os = msg.outer_start_index, is = msg.inner_start_index;

    HeightMap hm;
    hm.resolution = res;
    hm.frame = msg.header.frame_id;
    if (hm.frame.empty()) hm.frame = frame_;
    // 내 격자: col↔x(증가), row↔y(증가), origin=(0,0) 셀 중심.
    hm.origin = Eigen::Vector2d(cx - msg.info.length_x / 2.0 + 0.5 * res,
                               cy - msg.info.length_y / 2.0 + 0.5 * res);
    hm.heights = Eigen::MatrixXd::Zero(gm_cols, gm_rows);  // (Y=gm_cols) × (X=gm_rows)
    hm.variance = Eigen::MatrixXd::Zero(gm_cols, gm_rows);
    hm.valid = Eigen::ArrayXXi::Zero(gm_cols, gm_rows);
    for (int li = 0; li < gm_rows; ++li) {       // 논리 row → -x
      const int bi = (li + os) % gm_rows;        // 버퍼 row(순환)
      const int my_col = gm_rows - 1 - li;       // x 증가 방향으로 뒤집기
      for (int lj = 0; lj < gm_cols; ++lj) {     // 논리 col → -y
        const int bj = (lj + is) % gm_cols;      // 버퍼 col(순환)
        const int my_row = gm_cols - 1 - lj;
        const float h = fa.data[static_cast<size_t>(bi) + static_cast<size_t>(bj) * gm_rows];
        if (std::isfinite(h)) {
          hm.heights(my_row, my_col) = h;
          hm.valid(my_row, my_col) = 1;
          if (var >= 0) {
            const float v = msg.data[var].data[static_cast<size_t>(bi) +
                                               static_cast<size_t>(bj) * gm_rows];
            if (std::isfinite(v)) hm.variance(my_row, my_col) = v;
          }
        }
      }
    }
    terrain_ = std::move(hm);
    if (support_patch_enabled_) {
      support_patch_memory_.SyncLayout(*terrain_);
      RememberSupportContacts();
    }
    map_stamp_ = msg.header.stamp;
    UpdateMapDatum();
  }

  // 실제 SQP 입력을 표시한다. 원본 시각을 보존하고 미관측 셀은 NaN으로 유지한다.
  // 진단 발행만 최대 10 Hz로 제한하며 계산·계획 발행 조건에는 관여하지 않는다.
  void PublishCorrectedMap(const HeightMap& map,
                           const builtin_interfaces::msg::Time& stamp,
                           const TerrainModel* terrain = nullptr) {
    const auto wall_now = std::chrono::steady_clock::now();
    if (last_corrected_map_publish_ &&
        wall_now - *last_corrected_map_publish_ < std::chrono::milliseconds(100)) return;
    last_corrected_map_publish_ = wall_now;
    grid_map_msgs::msg::GridMap msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map.frame;
    const int rows = map.heights.cols();  // grid_map row는 -X 방향이다.
    const int cols = map.heights.rows();  // grid_map col은 -Y 방향이다.
    msg.info.resolution = map.resolution;
    msg.info.length_x = rows * map.resolution;
    msg.info.length_y = cols * map.resolution;
    msg.info.pose.position.x = map.origin.x() + (rows - 1) * map.resolution / 2.0;
    msg.info.pose.position.y = map.origin.y() + (cols - 1) * map.resolution / 2.0;
    msg.info.pose.orientation.w = 1.0;
    msg.layers = {"elevation", "variance"};
    if (terrain) msg.layers = {"elevation", "variance", "h", "h_s1", "h_s2", "observed"};
    msg.basic_layers = {"elevation"};
    msg.data.resize(msg.layers.size());
    for (auto& layer : msg.data) {
      layer.layout.dim.resize(2);
      layer.layout.dim[0].label = "column_index";
      layer.layout.dim[0].size = cols;
      layer.layout.dim[0].stride = rows * cols;
      layer.layout.dim[1].label = "row_index";
      layer.layout.dim[1].size = rows;
      layer.layout.dim[1].stride = rows;
      layer.data.resize(rows * cols, std::numeric_limits<float>::quiet_NaN());
    }
    for (int col = 0; col < cols; ++col) {
      for (int row = 0; row < rows; ++row) {
        const int x = rows - 1 - row;
        const int y = cols - 1 - col;
        const int index = row + col * rows;
        if (map.valid(y, x)) {
          msg.data[0].data[index] = map.heights(y, x);
          msg.data[1].data[index] = map.variance(y, x);
        }
        if (terrain) {
          msg.data[5].data[index] = map.valid(y, x);
          if (terrain->processed_map().valid(y, x)) {
            msg.data[2].data[index] = terrain->processed_map().heights(y, x);
            msg.data[3].data[index] = terrain->smooth_heights()(y, x);
            msg.data[4].data[index] = terrain->virtual_map().heights(y, x);
          }
        }
      }
    }
    corrected_map_pub_->publish(msg);
  }

  // leg의 time t 발끝 위치(odom)와 접지 여부. 스탠스=정착 위치, 스윙=포물선 arc.
  std::pair<Eigen::Vector3d, bool> FootAt(const Result& r, const Feet& feet0,
                                          int leg, double t) const {
    Eigen::Vector3d planted = feet0.row(leg).transpose();  // 정착(착지 전) 위치
    const Eigen::Vector3d target = r.footholds.row(leg).transpose();
    int phase_idx = static_cast<int>(r.phases.size()) - 1;
    for (int k = 0; k < static_cast<int>(r.phases.size()); ++k) {
      if (t < r.phases[k].start + r.phases[k].duration - 1e-9) { phase_idx = k; break; }
    }
    // 착지 이전이면 planted, 이미 착지했으면 target.
    for (int k = 0; k <= phase_idx; ++k) {
      if (r.phases[k].landed[leg]) planted = target;
    }
    const auto& phase = r.phases[phase_idx];
    if (phase.contacts[leg]) return {planted, true};
    return {swing_references_[leg].Sample(r.seed_context.gait_time + t).first, false};
  }

  void UpdateSwingReferences(const Result& r, const Feet& feet0,
                             const b2_terrain_planner::msg::LocomotionState& state,
                             const HeightMap* swing_terrain = nullptr) {
    const double now_phase = r.seed_context.gait_time;
    for (int leg = 0; leg < 4; ++leg) {
      for (std::size_t k = 0; k < r.phases.size(); ++k) {
        const auto& phase = r.phases[k];
        if (phase.contacts[leg]) continue;
        SwingReference next;
        next.start = now_phase + phase.start;
        next.end = next.start + phase.duration;
        // 몸통 연결 때문에 분할된 같은 접촉 위상은 하나의 스윙으로 취급한다.
        for (std::size_t j = k + 1; j < r.phases.size() && !r.phases[j].contacts[leg]; ++j) {
          next.end = now_phase + r.phases[j].start + r.phases[j].duration;
        }
        next.apex_time = next.end - robot_.period * (1.0 - robot_.duty) * 0.5;
        next.position = feet0.row(leg).transpose();
        next.target = r.footholds.row(leg).transpose();
        next.apex_z = next.target.z() + swing_height_;
        const auto& old = swing_references_[leg];
        if (phase.start < 1e-9 && old.ActiveAt(now_phase) &&
            std::abs(old.end - next.end) < 1e-6) {
          // 이륙한 발은 남은 곡선 전체를 유지한다. Solve에도 같은 착지점을 고정했다.
          break;
        }
        // 이륙 전 후보는 같은 착지 시각이어도 현재 측정 위치에서 다시 만든다.
        // 이전 곡선의 시작점을 이어받으면 지지 중 실제 발 이동을 다음 스윙까지 숨긴다.
        if (phase.start < 1e-9) {
          const auto& v = state.foot_velocity[leg];
          next.velocity = Eigen::Vector3d(v.x, v.y, v.z);
        } else {
          next.apex_z = std::max(next.position.z(), next.target.z()) + swing_height_;
        }
        if (swing_terrain) {
          // 진행 중인 발의 첫 수신에서는 새 후퇴 동작을 시작하지 않는다.
          if (phase.start >= 1e-9 || state.contact_confirmed[leg]) {
            next.SelectStepShape(*swing_terrain, step_swing_, foot_radius_);
          }
          next.ShapeOverTerrain(*swing_terrain, swing_terrain_clearance_,
                                foot_radius_);
        }
        swing_references_[leg] = next;
        break;
      }
    }
  }

  // 상태 발행자는 이미 센서 기반 전신 운동학을 수행한다. base/명목 자세로 재근사하지 않는다.
  bool Fresh(const builtin_interfaces::msg::Time& stamp, double max_age) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) return false;
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age <= max_age;
  }

  void OnState(const b2_terrain_planner::msg::LocomotionState& msg) {
    auto finite = [](const auto& v) {
      return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    const auto& q = msg.base_orientation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    bool valid = msg.header.frame_id == state_frame_ && Fresh(msg.header.stamp, state_max_age_) &&
        finite(msg.com_position) && finite(msg.com_velocity) &&
        finite(msg.base_angular_velocity) && std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
    for (int leg = 0; leg < 4; ++leg) {
      valid = valid && finite(msg.foot_position[leg]) && finite(msg.foot_velocity[leg]);
    }
    if (msg.gait_clock_valid) {
      valid = valid && msg.gait_epoch_ns > 0 && std::isfinite(msg.gait_time) && msg.gait_time >= 0.0;
    }
    if (msg.body_clock_valid) {
      valid = valid && msg.gait_clock_valid && std::isfinite(msg.body_time) &&
          msg.body_time >= 0;
    }
    if (!valid) {
      const auto stamp_ns = static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000 +
          msg.header.stamp.nanosec;
      PublishPipelineStatus("state_rejected", "invalid_stale_future_or_frame", -1, stamp_ns);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Rejected invalid, stale, future or wrong-frame locomotion state");
      return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (state_ && stamp <= rclcpp::Time(state_->header.stamp).nanoseconds()) {
      PublishPipelineStatus("state_rejected", "duplicate_or_out_of_order", -1, stamp);
      return;
    }
    if (state_ && (msg.body_clock_valid != state_->body_clock_valid ||
        (msg.body_clock_valid && msg.body_time < state_->body_time) ||
        msg.gait_clock_valid != state_->gait_clock_valid ||
        (msg.gait_clock_valid && (msg.gait_epoch_ns != state_->gait_epoch_ns ||
                                 msg.gait_time < state_->gait_time)))) {
      ++generation_;
      retry_requested_ = true;
      previous_.reset();
      DiscardRetrySeed();
      published_plans_.clear();
      command_ramp_time_.reset();
      applied_command_.setZero();
      hold_anchor_.reset();
      touchdown_stop_ = false;
      swing_references_ = {};
    }
    bool replan = state_ && state_->gait_paused != msg.gait_paused;
    if (state_ && msg.plan_adoption_status > 1 &&
        (msg.candidate_plan_stamp_ns != state_->candidate_plan_stamp_ns ||
         msg.plan_adoption_status != state_->plan_adoption_status)) replan = true;
    if (state_ && msg.gait_clock_valid && msg.gait_epoch_ns == state_->gait_epoch_ns) {
      for (int leg = 0; leg < 4; ++leg) {
        if (!state_->contact_confirmed[leg] && msg.contact_confirmed[leg]) replan = true;
      }
    }
    state_ = msg;
    RememberSupportContacts();
    if (UpdateTouchdownStop(msg)) {
      replan = true;
      const char* event = "touchdown_stop_released";
      if (touchdown_stop_) event = "touchdown_stop_entered";
      PublishPipelineStatus(event, "landing_confirmation", -1, stamp);
    }
    UpdateCommandRamp(msg);
    // 후보를 발행했다고 실행된 것으로 간주하지 않는다. WBC가 확인한 스윙만 고정한다.
    if (const auto* active = ActivePlan()) swing_references_ = active->swings;
    // 위상을 멈추기 전에 착지해도 즉시 재계획한다. 미확인 지지는 OnTimer가 계획에서 제외한다.
    if (replan) retry_requested_ = true;
    if (retry_requested_) OnTimer();
  }

  void OnTimer() {
    const auto current_time = now().nanoseconds();
    if (last_clock_ns_ > current_time) {
      PublishPipelineStatus("reset", "clock_reversed");
      ++generation_;
      retry_requested_ = false;
      same_state_retry_used_ = false;
      swing_references_ = {};
      state_.reset();
      terrain_.reset();
      support_patch_memory_.Reset();
      previous_.reset();
      DiscardRetrySeed();
      published_plans_.clear();
      command_ramp_time_.reset();
      applied_command_.setZero();
      hold_anchor_.reset();
      touchdown_stop_ = false;
      epoch_ns_.reset();
      last_planned_ns_ = -1;
      command_stamp_.reset();
      if (!command_topic_.empty()) command_.setZero();
    }
    last_clock_ns_ = current_time;
    if (state_) {
      UpdateTouchdownStop(*state_);
      UpdateCommandRamp(*state_);
    }
    // 램프한 사용자 목표를 사용하며 착지 대기는 속도 목표만 즉시 0으로 낮춘다.
    Eigen::Vector3d effective_command = PlanningCommand();
    effective_command.head<2>() += StationKeepingCorrection();
    geometry_msgs::msg::TwistStamped command_message;
    command_message.header.stamp = now();
    command_message.header.frame_id = "base_heading";
    command_message.twist.linear.x = effective_command.x();
    command_message.twist.linear.y = effective_command.y();
    command_message.twist.angular.z = effective_command.z();
    effective_command_pub_->publish(command_message);
    // worker는 입력 복사본만 사용한다. 수신 callback과 결과 채택은 이 executor에서 직렬 실행한다.
    if (solve_future_.valid()) return;
    if (!state_) {
      PublishPipelineStatus("deferred", "no_state");
      return;
    }
    if (!Fresh(state_->header.stamp, state_max_age_)) {
      PublishPipelineStatus("deferred", "state_not_fresh");
      return;
    }
    if (terrain_ && !Fresh(map_stamp_, map_max_age_)) {
      PublishPipelineStatus("deferred", "map_not_fresh");
      return;
    }
    if (require_map_ && !terrain_) {
      PublishPipelineStatus("deferred", "no_map");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for elevation map");
      return;
    }
    const auto& state = *state_;
    // 일괄 pause 차단은 하지 않는다. 아래에서 미확인 예정 지지를 차단하되,
    // 몸통 시각을 제공하는 경우 현재 지지를 유지하는 유한 horizon을 다시 계산한다.
    const auto stamp_ns = rclcpp::Time(state.header.stamp).nanoseconds();
    if (stamp_ns < last_planned_ns_ ||
        (stamp_ns == last_planned_ns_ && (!retry_requested_ || same_state_retry_used_))) {
      PublishPipelineStatus("deferred", "state_already_planned");
      return;
    }
    if (!epoch_ns_) epoch_ns_ = stamp_ns;
    const Eigen::Quaterniond q(state.base_orientation.w, state.base_orientation.x,
                               state.base_orientation.y, state.base_orientation.z);
    const Eigen::Vector3d ypr = QuatToZyx(q);
    Input input = nominal_input_;
    input.frame = frame_;
    input.terrain = terrain_;
    input.command = effective_command;
    if (state.body_clock_valid) input.body_time = state.body_time;
    input.gait_time = static_cast<double>(stamp_ns - *epoch_ns_) * 1e-9;
    if (state.gait_clock_valid) {
      input.gait_time = state.gait_time;
      const auto nominal = MakeHorizon(input.gait_time, robot_.period, robot_.duty);
      // WBC stance를 지지로 유지하고, 아직 stance가 아닌 예정 지지만 기다린다.
      // 추정 접촉이 OFF로 바뀌어도 이미 확인된 stance를 취소하지 않는다.
      bool missing_support = false;
      std::array<bool, 4> supports{};
      for (int leg = 0; leg < 4; ++leg) {
        supports[leg] = state.contact_confirmed[leg];
        if (nominal.front().contacts[leg] && !supports[leg]) missing_support = true;
      }
      if (missing_support) {
        if (!state.body_clock_valid) {
          PublishPipelineStatus("deferred", "support_unconfirmed");
          return;
        }
        if (std::count(supports.begin(), supports.end(), true) < 2) {
          PublishPipelineStatus("deferred", "fewer_than_two_confirmed_supports");
          return;
        }
        // 착지 시각을 예측해 지지로 가정하지 않는다. 이번 0.8초 계획은 현재
        // 지지를 유지하며, 실제 착지가 확인되면 즉시 다음 스윙 계획으로 갱신한다.
        input.gait_hold_duration = robot_.period;
        input.hold_contacts = supports;
      }
    }
    if (state.gait_clock_valid && state.active_plan_stamp_ns >= state.gait_epoch_ns &&
        !published_plans_.empty()) {
      const auto* active = ActivePlan();
      if (!active) {
        PublishPipelineStatus("deferred", "active_plan_not_in_history");
        return;
      }
      // 몸통 궤적은 측정 상태에서 다시 시작한다(폐루프). 이전 계획에 하드 고정하던
      // reference_prefix는 추적 오차를 영원히 누적시켜 제거했다. prefix_duration은
      // 이제 핸드오버 마감시각과 착지 대기 재개 유예로만 쓰인다.
      input.prefix_duration = settings_.time_limit + 2 * sample_dt_;
      if (handover_grace_ > 0) input.prefix_duration = handover_grace_;
      // legacy는 정지한 기준점에 연결한다. 별도 몸통 시각은 계산 중에도 진행한다.
      if (state.gait_paused && !state.body_clock_valid) input.prefix_duration = 0;
      if (state.body_clock_valid && state.gait_paused &&
          input.gait_hold_duration == 0) {
        input.gait_hold_duration = input.prefix_duration;
        input.hold_contacts = state.contact_confirmed;
        const auto& context = active->body.seed_context;
        // 이미 채택한 재개 계획의 시각을 유지한다. 매 재계획마다 prefix만큼
        // 재개를 연기하면 발이 영원히 멈추므로 몸통 인계 시각과 구분한다.
        if (context.body_time && context.gait_hold_duration > 0 &&
            context.gait_hold_duration < robot_.period &&
            context.hold_contacts == input.hold_contacts) {
          input.gait_hold_duration = std::max(0.0,
              BodyTime(context) + context.gait_hold_duration - BodyTime(input));
        }
      }
      // prefix 고정이 없으므로 이전 계획 horizon이 짧아도 재계획을 막지 않는다.
    }
    input.contact_estimated = state.contact_estimated;
    input.position << state.com_position.x, state.com_position.y, state.com_position.z,
                      ypr[0], ypr[1], ypr[2];
    // wire는 world 각속도, 최적화 변수는 ZYX Euler 변화율이다.
    const Eigen::Vector3d omega_body = q.normalized().conjugate() * Eigen::Vector3d(
        state.base_angular_velocity.x, state.base_angular_velocity.y, state.base_angular_velocity.z);
    const double pitch = ypr[1], roll = ypr[2];
    if (std::abs(std::cos(pitch)) < 0.1) {
      PublishPipelineStatus("deferred", "euler_singularity");
      return;
    }
    const double yaw_rate = (std::sin(roll) * omega_body.y() +
                             std::cos(roll) * omega_body.z()) / std::cos(pitch);
    const double pitch_rate = std::cos(roll) * omega_body.y() - std::sin(roll) * omega_body.z();
    const double roll_rate = omega_body.x() + std::sin(pitch) * yaw_rate;
    input.velocity << state.com_velocity.x, state.com_velocity.y, state.com_velocity.z,
                      yaw_rate, pitch_rate, roll_rate;
    for (int leg = 0; leg < 4; ++leg) {
      const auto& foot = state.foot_position[leg];
      input.feet.row(leg) << foot.x, foot.y, foot.z;
    }
    // 지도 틱에서 추적한 datum 오프셋을 solve 입력 복사본에만 적용한다(지형 모양 보존).
    if (map_datum_reanchor_enabled_ && input.terrain) {
      ApplyDatumOffset(&*input.terrain, map_datum_.offset);
    }
    const auto swing_terrain = input.terrain;
    if (support_patch_enabled_ && input.terrain) {
      // 수명 초과 패치는 옛 KF datum의 화석이라 적용하지 않는다(가짜 둔덕 방지).
      const int count = support_patch_memory_.ApplyTo(
          &*input.terrain, rclcpp::Time(state.header.stamp).seconds(),
          support_patch_max_age_sec_);
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Support contact memory: %d cells, half-width %.3f m (planner input only)",
                          count, support_patch_half_width_);
    }
    if (state.gait_clock_valid && !CommitActiveSwingFootholds(robot_, swing_references_, &input)) {
      PublishPipelineStatus("deferred", "active_swing_reference_missing");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "waiting: no matching committed reference for active swing");
      return;
    }
    // 같은 입력은 실패 후 한 번만 즉시 재시도한다. 새 상태가 오면 다시 시도할 수 있다.
    same_state_retry_used_ = stamp_ns == last_planned_ns_;
    last_planned_ns_ = stamp_ns;
    retry_requested_ = false;
    pending_solve_ = PendingSolve{input, state, map_stamp_, generation_,
                                  touchdown_stop_, swing_terrain};
    if (map_datum_reanchor_enabled_) pending_solve_->diagnostic_datum_offset = map_datum_.offset;
    ++attempt_seq_;
    solve_state_stamp_ns_ = stamp_ns;
    solve_start_ns_ = now().nanoseconds();
    solve_end_ns_ = 0;
    PublishPipelineStatus("solve_started", "none");
    std::optional<Result> seed = previous_;
    if (retry_seed_ && retry_seed_parent_ns_ == state.active_plan_stamp_ns) seed = retry_seed_;
    solve_future_ = std::async(std::launch::async,
        [robot = robot_, input = std::move(input), settings = settings_,
         previous = std::move(seed), solver = solver_, clock = get_clock()] {
          Result result;
          try {
            const Result* seed = nullptr;
            if (previous) seed = &*previous;
            result = solver(robot, input, settings, seed);
          } catch (const std::exception& error) {
            result.status = std::string("solve_exception: ") + error.what();
          }
          return std::make_pair(std::move(result), clock->now().nanoseconds());
        });
  }

  void CompleteSolve() {
    if (!solve_future_.valid() ||
        solve_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    // 시각 역행은 완료 직전에도 검사해 이전 실행의 결과를 채택하지 않는다.
    if (last_clock_ns_ > now().nanoseconds()) OnTimer();
    auto completed = solve_future_.get();
    Result& result = completed.first;
    solve_end_ns_ = completed.second;
    auto pending = std::move(*pending_solve_);
    pending_solve_.reset();
    const auto& state = pending.state;
    const auto& input = pending.input;
    if (input.terrain && pending.generation == generation_) {
      PublishCorrectedMap(*input.terrain, pending.map_stamp, result.terrain_snapshot.get());
    }
    bool publishable = result.success;
    const char* reason = "solve_failed";
    if (publishable) {
      publishable = pending.generation == generation_ && state_.has_value();
      reason = "state_reset_during_solve";
    }
    if (publishable) {
      publishable = Fresh(state.header.stamp, state_max_age_) &&
                    Fresh(state_->header.stamp, state_max_age_);
      reason = "state_not_fresh_after_solve";
    }
    if (publishable) {
      publishable = pending.touchdown_stop == touchdown_stop_;
      reason = "touchdown_stop_changed_after_solve";
    }
    if (publishable && state.gait_clock_valid) {
      publishable = state.active_plan_stamp_ns == state_->active_plan_stamp_ns;
      reason = "active_plan_changed_after_solve";
      if (publishable) {
        const auto before = MakeHorizon(state.gait_time, robot_.period, robot_.duty);
        const auto after = MakeHorizon(state_->gait_time, robot_.period, robot_.duty);
        // 추정 접촉 변동만으로 WBC stance가 유지된 계획을 버리지 않는다.
        publishable = before.front().contacts == after.front().contacts &&
                      state.contact_confirmed == state_->contact_confirmed &&
                      (!state.body_clock_valid || state.gait_paused == state_->gait_paused);
        reason = "contact_phase_changed_after_solve";
      }
    }
    if (publishable) {
      publishable = !input.terrain || (Fresh(pending.map_stamp, map_max_age_) &&
                                      terrain_ && Fresh(map_stamp_, map_max_age_));
      reason = "map_not_fresh_after_solve";
    }
    if (publishable && input.reference_prefix) {
      publishable = StateBodyTime(*state_) < BodyTime(input) + input.prefix_duration;
      if (input.prefix_duration == 0) {
        publishable = state_->gait_paused && state_->gait_time == input.gait_time;
      }
      reason = "body_handover_missed";
    }
    // 명령은 계산 입력의 snapshot이다. 새 명령 수신은 성공해 발행을 취소하지 않는다.
    if (!result.success) {
      // 실패 반복값은 성공 계획과 별도 보관한다. 부모/세션이 같은 계산에서만 재사용한다.
      // 시드 체인이 발산하거나 소진되면 폐기해 다음 계산을 마지막 발행 계획에서 시작한다.
      // 발행 정체 중에는 부모가 바뀌지 않아 오염된 시드가 영원히 유효 판정을 받기 때문이다.
      bool store_seed = result.seed_available && !result.phases.empty() &&
          result.coefficients.size() == result.phases.size() &&
          result.margins.size() == result.phases.size() &&
          pending.generation == generation_ && state_ &&
          state.active_plan_stamp_ns == state_->active_plan_stamp_ns;
      const bool used_retry_seed =
          result.warm_start.reason.rfind("shifted_retry_iterate", 0) == 0;
      if (used_retry_seed) {
        const bool diverged =
            result.violation > 2.0 * retry_seed_violation_ + 1e-9;
        if (++retry_seed_failures_ >= 3 || diverged) {
          store_seed = false;
          const char* reason = "chain_exhausted";
          if (diverged) reason = "violation_diverged";
          PublishPipelineStatus("retry_seed_discarded", reason);
        }
      }
      if (store_seed) {
        retry_seed_ = result;
        if (input.reference_prefix) {
          retry_seed_ = SpliceBody(*input.reference_prefix,
              BodyTime(input) - BodyTime(input.reference_prefix->seed_context),
              result, input.prefix_duration);
        }
        retry_seed_parent_ns_ = state.active_plan_stamp_ns;
        if (!used_retry_seed) retry_seed_failures_ = 1;
        retry_seed_violation_ = result.violation;
      } else {
        DiscardRetrySeed();
      }
      PublishSolveStatus(result, pending, false, "solve_failed");
      PublishPipelineStatus("solve_failed", result.status.c_str());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "solve failed: %s", result.status.c_str());
      retry_requested_ = true;
    } else if (!publishable) {
      PublishSolveStatus(result, pending, false, reason);
      PublishPipelineStatus("publish_deferred", reason);
      retry_requested_ = true;
    } else {
      active_frame_ = state.header.frame_id;
      const HeightMap* swing_terrain = nullptr;
      if (pending.swing_terrain) {
        swing_terrain = &*pending.swing_terrain;
      }
      Publish(result, input, state, swing_terrain);
      // 실제 발행한 몸통 곡선을 다음 계산의 초기값으로 보관한다.
      previous_ = published_plans_.back().body;
      DiscardRetrySeed();
      PublishSolveStatus(result, pending, true, "published");
    }
    // 실패해도 마지막 성공해는 보존한다. Solve 내부가 시간·모델·지형 호환성을 재검사한다.
    // 계산 중 수신한 최신 상태 또는 같은 상태의 제한된 재시도를 즉시 시작한다.
    if (retry_requested_) OnTimer();
  }

  void Publish(const Result& solved, const Input& input,
               const b2_terrain_planner::msg::LocomotionState& state,
               const HeightMap* swing_terrain = nullptr) {
    const rclcpp::Time stamp(state.header.stamp);
    const Feet& feet0 = input.feet;
    // 발 접촉 스케줄은 원래 solver 결과로 만들고 몸통만 정확히 이어 붙인다.
    Result foot_plan = solved;
    if (state.body_clock_valid) {
      foot_plan.phases = MakeHorizon(input.gait_time, robot_.period, robot_.duty);
    }
    const bool hold_gait = input.gait_hold_duration >= robot_.period;
    if (!hold_gait) UpdateSwingReferences(foot_plan, feet0, state, swing_terrain);
    Result r = solved;
    if (input.reference_prefix) {
      r = SpliceBody(*input.reference_prefix,
          BodyTime(input) - BodyTime(input.reference_prefix->seed_context),
          solved, input.prefix_duration);
    }
    double horizon = 0.0;
    for (const auto& p : r.phases) horizon += p.duration;
    const int n = std::max(1, static_cast<int>(std::round(horizon / sample_dt_)) + 1);

    b2_terrain_planner::msg::LocomotionPlan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = active_frame_;
    // WBC가 활성 계획을 들고 있으면 거기에 체이닝한다. prefix 고정과는 무관하다.
    const bool has_parent = state.gait_clock_valid &&
        state.active_plan_stamp_ns >= state.gait_epoch_ns && state.active_plan_stamp_ns > 0;
    msg.parent_plan_stamp_ns = 0;
    if (has_parent) msg.parent_plan_stamp_ns = state.active_plan_stamp_ns;
    msg.handover_gait_time = input.gait_time;
    if (has_parent && !state.body_clock_valid) {
      msg.handover_gait_time += input.prefix_duration;
    }
    msg.body_clock_valid = state.body_clock_valid;
    msg.body_time = BodyTime(input);
    msg.handover_body_time = BodyTime(input) + input.prefix_duration;
    msg.gait_resume_body_time = BodyTime(input) + input.gait_hold_duration;
    msg.gait_duration = robot_.period - input.gait_hold_duration;
    msg.hold_gait = hold_gait;
    for (std::size_t k = 0; k < r.phases.size(); ++k) {
      msg.body_breaks.push_back(r.phases[k].start);
      for (int axis = 0; axis < 6; ++axis) {
        for (int power = 0; power < 5; ++power) {
          msg.body_coefficients.push_back(r.coefficients[k](axis, power));
        }
      }
    }
    msg.body_breaks.push_back(horizon);
    msg.dt = sample_dt_;
    msg.gait_clock_valid = state.gait_clock_valid;
    msg.gait_epoch_ns = state.gait_epoch_ns;
    msg.gait_time = r.seed_context.gait_time;
    msg.gait_period = robot_.period;
    msg.gait_duty = robot_.duty;
    msg.num_samples = n;
    msg.base_pose.resize(n);
    msg.base_twist.resize(n);
    msg.foot_position.resize(static_cast<size_t>(4) * n);
    msg.foot_contact.resize(static_cast<size_t>(4) * n);
    for (int i = 0; i < n; ++i) {
      const double t = std::min(static_cast<double>(i) * sample_dt_, horizon);
      const Vector6 pose = Evaluate(r, t, 0);
      const Vector6 vel = Evaluate(r, t, 1);
      auto& bp = msg.base_pose[i];
      bp.position.x = pose[0]; bp.position.y = pose[1]; bp.position.z = pose[2];
      const Eigen::Quaterniond bq = ZyxToQuat(pose[3], pose[4], pose[5]);
      bp.orientation.x = bq.x(); bp.orientation.y = bq.y();
      bp.orientation.z = bq.z(); bp.orientation.w = bq.w();
      auto& bt = msg.base_twist[i];
      bt.linear.x = vel[0]; bt.linear.y = vel[1]; bt.linear.z = vel[2];
      bt.angular.x = vel[5]; bt.angular.y = vel[4]; bt.angular.z = vel[3];  // euler rate (roll,pitch,yaw)
      for (int leg = 0; leg < 4; ++leg) {
        double foot_time = t;
        if (state.body_clock_valid) foot_time = std::min(t, msg.gait_duration);
        Eigen::Vector3d foot = feet0.row(leg).transpose();
        bool contact = state.contact_confirmed[leg];
        if (!hold_gait) {
          const auto sample = FootAt(foot_plan, feet0, leg, foot_time);
          foot = sample.first;
          contact = sample.second;
        }
        const size_t fi = static_cast<size_t>(i) * 4 + leg;
        msg.foot_position[fi].x = foot.x();
        msg.foot_position[fi].y = foot.y();
        msg.foot_position[fi].z = foot.z();
        msg.foot_contact[fi] = contact;
      }
    }
    published_plans_.push_back(PublishedPlan{stamp.nanoseconds(), r, swing_references_});
    if (published_plans_.size() > 32) published_plans_.pop_front();
    plan_pub_->publish(msg);
    last_publish_ns_ = now().nanoseconds();
    last_published_stamp_ns_ = stamp.nanoseconds();
    PublishPipelineStatus("published", "none");
    if (publish_markers_) PublishMarkers(msg, r, stamp);
  }

  static std_msgs::msg::ColorRGBA Color(double r, double g, double b, double a = 1.0) {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
  }

  // RViz 마커: 몸통 궤적(라인), 네 발 궤적(라인), footholds(구). odom 프레임으로 elevation map 위에 겹침.
  void PublishMarkers(const b2_terrain_planner::msg::LocomotionPlan& plan,
                      const Result& r, const rclcpp::Time& stamp) {
    static const std::array<std_msgs::msg::ColorRGBA, 4> leg_color{
        Color(0.2, 0.4, 1.0), Color(0.2, 0.85, 1.0),
        Color(0.2, 0.9, 0.3), Color(1.0, 0.35, 0.2)};
    static const std::array<std::string, 4> leg_name{"FL", "FR", "RL", "RR"};
    const int n = static_cast<int>(plan.base_pose.size());
    visualization_msgs::msg::MarkerArray arr;

    auto make = [&](const std::string& ns, int id, int type) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = active_frame_;
      m.header.stamp = stamp;
      m.ns = ns; m.id = id; m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.lifetime = rclcpp::Duration::from_seconds(2.0 / std::max(1.0, plan_rate_hz_));
      return m;
    };

    // 몸통 궤적.
    auto body = make("body_path", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    body.scale.x = 0.012;
    body.color = Color(1.0, 0.95, 0.1);
    for (int i = 0; i < n; ++i) body.points.push_back(plan.base_pose[i].position);
    arr.markers.push_back(body);

    // 네 발 궤적(스윙 arc 포함).
    for (int leg = 0; leg < 4; ++leg) {
      auto path = make("foot_path_" + leg_name[leg], leg,
                       visualization_msgs::msg::Marker::LINE_STRIP);
      path.scale.x = 0.008;
      path.color = leg_color[leg];
      for (int i = 0; i < n; ++i)
        path.points.push_back(plan.foot_position[static_cast<size_t>(i) * 4 + leg]);
      arr.markers.push_back(path);
    }

    // footholds(최종 착지점) 구.
    auto fh = make("footholds", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
    fh.scale.x = fh.scale.y = fh.scale.z = 0.05;
    fh.color = Color(1.0, 0.5, 0.0);
    for (int leg = 0; leg < 4; ++leg) {
      geometry_msgs::msg::Point p;
      p.x = r.footholds(leg, 0); p.y = r.footholds(leg, 1); p.z = r.footholds(leg, 2);
      fh.points.push_back(p);
      fh.colors.push_back(leg_color[leg]);
    }
    arr.markers.push_back(fh);

    marker_pub_->publish(arr);
  }

  std::string last_pipeline_log_key_;
  std::chrono::steady_clock::time_point last_pipeline_log_time_;

  // 상태.
  Robot robot_;
  Input nominal_input_;
  Settings settings_;
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d applied_command_ = Eigen::Vector3d::Zero();
  bool touchdown_stop_ = false;
  std::optional<double> command_ramp_time_;
  double command_linear_accel_ = 0.5;
  double command_angular_accel_ = 1.0;
  bool station_keeping_enabled_ = false;
  double station_keeping_gain_ = 0.5;
  double station_keeping_max_velocity_ = 0.06;
  std::optional<Eigen::Vector2d> hold_anchor_;
  std::deque<PublishedPlan> published_plans_;
  std::string command_topic_;
  double command_timeout_ = 0.25;
  std::optional<builtin_interfaces::msg::Time> command_stamp_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr effective_command_pub_;
  std::optional<HeightMap> terrain_;
  std::optional<b2_terrain_planner::msg::LocomotionState> state_;
  builtin_interfaces::msg::Time map_stamp_;
  std::optional<int64_t> epoch_ns_;
  int64_t last_clock_ns_ = 0, last_planned_ns_ = -1;
  std::optional<Result> previous_;
  std::optional<Result> retry_seed_;
  std::int64_t retry_seed_parent_ns_ = 0;
  int retry_seed_failures_ = 0;
  double retry_seed_violation_ = std::numeric_limits<double>::infinity();
  std::function<Result(const Robot&, const Input&, const Settings&, const Result*)> solver_ = Solve;
  std::future<std::pair<Result, std::int64_t>> solve_future_;
  std::optional<PendingSolve> pending_solve_;
  std::uint64_t generation_ = 0;
  bool retry_requested_ = false, same_state_retry_used_ = false;
  std::int64_t solve_state_stamp_ns_ = 0;

  // 파라미터.
  std::string state_topic_, state_frame_, map_topic_, elevation_layer_, plan_topic_, frame_;
  std::string active_frame_ = "odom";
  StepSwingSettings step_swing_;
  double swing_terrain_clearance_ = 0.03;
  double foot_radius_ = kDefaultFootRadius;
  double plan_rate_hz_ = 10.0, sample_dt_ = 0.02, swing_height_ = 0.10;
  double handover_grace_ = -1.0;
  bool require_map_ = true;
  bool support_patch_enabled_ = false;
  double support_patch_half_width_ = 0.025;
  double support_patch_max_age_sec_ = 2.0;
  SupportPatchMemory support_patch_memory_;
  bool map_datum_reanchor_enabled_ = false;
  double map_datum_max_rate_ = 0.01;
  DatumOffsetState map_datum_;
  std::int64_t map_datum_stamp_ns_ = 0;
  double state_max_age_ = 0.25, map_max_age_ = 1.0;

  rclcpp::Subscription<b2_terrain_planner::msg::LocomotionState>::SharedPtr state_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr corrected_map_pub_;
  std::optional<std::chrono::steady_clock::time_point> last_corrected_map_publish_;
  rclcpp::Publisher<b2_terrain_planner::msg::LocomotionPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr solve_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_status_pub_;
  std::uint64_t pipeline_event_seq_ = 0, attempt_seq_ = 0;
  std::int64_t solve_start_ns_ = 0, solve_end_ns_ = 0;
  std::int64_t last_publish_ns_ = 0, last_published_stamp_ns_ = 0;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  bool publish_markers_ = true;
  rclcpp::TimerBase::SharedPtr timer_, completion_timer_;
  std::array<SwingReference, 4> swing_references_{};

};

#ifndef B2_TERRAIN_PLANNER_NO_MAIN
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
