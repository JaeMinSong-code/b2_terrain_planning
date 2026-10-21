/**
 * @file terrain_planner_node.cpp
 * @brief elevation map + WBC 상태를 구독해 TAMOLS 계획을 LocomotionPlan으로 발행하는 ROS2 노드.
 * @version 0.0.14
 * @update 2026-10-21: 착지 대기 중 몸통 시간 진행과 유한 재계획·보행 재개 연속성을 반영함.
 * @update 2026-10-19: 명령 확정·실행 계획 확인·착지 인계·중단 계산 재시도를 구현함.
 *
 * 모든 계산은 odom 프레임에서 수행한다. 발행 궤적(몸통/네 발)은 WBC 추종 모드가 소비한다.
 * 공통 센서 기반 CoM/발 상태를 초기 조건으로 사용하고 입력 만료를 검사한다.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"
#include "b2_terrain_planner/support_patch.h"
#include "b2_terrain_planner/swing_reference.h"
#include "b2_terrain_planner/committed_swing.h"

#include <array>
#include <deque>
#include <cmath>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
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

std::string DefaultModelPath() {
  const auto exe = std::filesystem::canonical("/proc/self/exe");
  const auto installed = exe.parent_path().parent_path().parent_path() /
      "share/b2_terrain_planner/model/unitree_b2/b2.urdf";
  if (std::filesystem::exists(installed)) return installed.string();
  return B2_BUILD_MODEL_PATH;
}

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
    swing_height_ = declare_parameter<double>("swing_height", 0.10);
    require_map_ = declare_parameter<bool>("require_map", true);
    support_patch_enabled_ = declare_parameter<bool>("support_patch.enabled", false);
    support_patch_half_width_ = declare_parameter<double>("support_patch.half_width", 0.30);
    if (!std::isfinite(support_patch_half_width_) || support_patch_half_width_ <= 0) {
      throw std::invalid_argument("support_patch.half_width must be positive and finite");
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

  void LatchCommand(const State& state) {
    if (!state.gait_clock_valid) {
      applied_command_ = EffectiveCommand();
      return;
    }
    const auto nominal = MakeHorizon(state.gait_time, robot_.period, robot_.duty);
    const auto slot = static_cast<std::int64_t>(std::floor(
        2.0 * state.gait_time / robot_.period + 1e-9));
    if (slot == command_slot_) return;
    for (int leg = 0; leg < 4; ++leg) {
      if (!nominal.front().contacts[leg] || !state.contact_confirmed[leg]) return;
    }
    applied_command_ = EffectiveCommand();
    command_slot_ = slot;
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
        << ",\"command_slot\":" << command_slot_
        << ",\"active_plan_stamp_ns\":" << (state_.value_or(State{}).active_plan_stamp_ns)
        << ",\"retry_requested\":" << std::boolalpha << retry_requested_
        << ",\"last_planned_ns\":" << last_planned_ns_
        << ",\"last_published_stamp_ns\":" << last_published_stamp_ns_
        << ",\"last_publish_ns\":" << last_publish_ns_
        << ",\"solve_start_ns\":" << solve_start_ns_
        << ",\"solve_end_ns\":" << solve_end_ns_
        << ",\"map_stamp_ns\":" << rclcpp::Time(map_stamp_).nanoseconds()
        << ",\"leg\":" << leg;
    if (state_) {
      out << ",\"state_stamp_ns\":" << rclcpp::Time(state_->header.stamp).nanoseconds()
          << ",\"gait_epoch_ns\":" << state_->gait_epoch_ns
          << ",\"gait_time\":";
      if (std::isfinite(state_->gait_time)) out << state_->gait_time;
      else out << "null";
      out << ",\"gait_paused\":" << std::boolalpha << state_->gait_paused
          << ",\"body_time\":" << StateBodyTime(*state_);
    }
    out << '}';
    std_msgs::msg::String message;
    message.data = out.str();
    pipeline_status_pub_->publish(message);
  }

  void PublishSolveStatus(const Result& result, const Input& input,
                          const builtin_interfaces::msg::Time& stamp, bool published) {
    const auto number = [](double value) {
      if (!std::isfinite(value)) return std::string("null");
      std::ostringstream stream;
      stream << std::setprecision(12) << value;
      return stream.str();
    };
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
        << ",\"attempt_seq\":" << attempt_seq_
        << ",\"state_stamp_ns\":" << rclcpp::Time(stamp).nanoseconds()
        << ",\"solve_start_ns\":" << solve_start_ns_
        << ",\"solve_end_ns\":" << solve_end_ns_
        << ",\"last_publish_ns\":" << last_publish_ns_
        << ",\"elapsed\":" << number(result.elapsed)
        << ",\"iterations\":" << result.iterations
        << ",\"violation\":" << number(result.violation)
        << ",\"worst_constraint\":" << std::quoted(worst)
        << ",\"worst_violation\":" << number(worst_violation)
        << ",\"warm_start\":" << std::quoted(result.warm_start.reason)
        << ",\"state_age\":" << number((now() - rclcpp::Time(stamp)).seconds())
        << ",\"ik_checks\":" << result.ik_checks
        << ",\"max_ik_error\":" << number(result.max_ik_error)
        << ",\"ik_failed_leg\":" << result.ik_failed_leg.value_or(-1)
        << ",\"ik_failed_time\":" << number(result.ik_failed_time.value_or(-1))
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
    out << "]}";
    std_msgs::msg::String message;
    message.data = out.str();
    solve_status_pub_->publish(message);
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
    auto model = std::make_shared<RobotModel>(DefaultModelPath());
    ModelState state;
    for (int leg = 0; leg < 4; ++leg) {
      state.joints.segment<3>(3 * leg) << 0, 44.30 * M_PI / 180, -85.94 * M_PI / 180;
    }
    state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
    model->PopulatePlanner(state, &robot_, &nominal_input_);
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
    settings_.realtime_iterations = 5;
    settings_.sample_intervals = 12;
    settings_.qp_eps_abs = 1e-3;
    settings_.qp_eps_rel = 1e-3;
    settings_.qp_max_iterations = 4000;
    settings_.time_limit = declare_parameter<double>("time_limit", 0.1);
    settings_.orientation_limit =
        declare_parameter<double>("orientation_limit_deg", 60.0) * M_PI / 180.0;
    settings_.pitch_weight = declare_parameter<double>("pitch_weight", 10.0);
    settings_.leg_vertical_weight = declare_parameter<double>("leg_vertical_weight", 30.0);
    settings_.foothold_backward_weight =
        declare_parameter<double>("foothold_backward_weight", 1.4);
    settings_.vz_weight = declare_parameter<double>("vz_weight", 0.0);
  }

  // grid_map_msgs를 직접 디코딩한다(grid_map_core의 전역 Eigen 플러그인 오염 회피).
  // 규약: grid_map 논리 index (i=row→-x, j=col→-y). 버퍼는 (outer,inner) start로 순환.
  static int LayerIndex(const grid_map_msgs::msg::GridMap& m, const std::string& name) {
    for (size_t k = 0; k < m.layers.size(); ++k) if (m.layers[k] == name) return static_cast<int>(k);
    return -1;
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
    hm.frame = msg.header.frame_id.empty() ? frame_ : msg.header.frame_id;
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
    map_stamp_ = msg.header.stamp;
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
                             const b2_terrain_planner::msg::LocomotionState& state) {
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
        if (old.end > old.start && std::abs(old.end - next.end) < 1e-6 &&
            next.start >= old.start && next.start < old.end) {
          const auto sample = old.Sample(next.start);
          next.position = sample.first;
          next.velocity = sample.second;
          next.apex_z = old.apex_z;
        } else if (phase.start < 1e-9) {
          const auto& v = state.foot_velocity[leg];
          next.velocity = Eigen::Vector3d(v.x, v.y, v.z);
        } else {
          next.apex_z = std::max(next.position.z(), next.target.z()) + swing_height_;
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
      retry_seed_.reset();
      published_plans_.clear();
      command_slot_ = -1;
      applied_command_.setZero();
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
    LatchCommand(msg);
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
      previous_.reset();
      retry_seed_.reset();
      published_plans_.clear();
      command_slot_ = -1;
      applied_command_.setZero();
      epoch_ns_.reset();
      last_planned_ns_ = -1;
      command_stamp_.reset();
      if (!command_topic_.empty()) command_.setZero();
    }
    last_clock_ns_ = current_time;
    if (state_) LatchCommand(*state_);
    const Eigen::Vector3d effective_command = applied_command_;
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
      // 위상 경계에서 예정 지지가 아직 확인되지 않은 발로 정착점을 초기화하지 않는다.
      bool missing_support = false;
      std::array<bool, 4> supports{};
      for (int leg = 0; leg < 4; ++leg) {
        supports[leg] = state.contact_confirmed[leg] && state.contact_estimated[leg];
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
      input.reference_prefix = std::make_shared<Result>(active->body);
      input.prefix_duration = settings_.time_limit + 2 * sample_dt_;
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
      const double old_end = BodyTime(active->body.seed_context) + robot_.period;
      if (BodyTime(input) + input.prefix_duration > old_end) {
        PublishPipelineStatus("deferred", "active_body_horizon_too_short");
        return;
      }
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
    if (support_patch_enabled_ && input.terrain) {
      std::array<Eigen::Vector3d, 4> feet;
      for (int leg = 0; leg < 4; ++leg) feet[leg] = input.feet.row(leg).transpose();
      const double half_width = SupportPatchHalfWidth(support_patch_half_width_,
          input.terrain->resolution, settings_.terrain.smoothing_sigma);
      const int count = ApplySupportPatches(&*input.terrain, feet,
                                           state.contact_estimated, half_width);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Support patch: %d cells, half-width %.3f m (planner input only)",
                          count, half_width);
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
    pending_solve_ = PendingSolve{input, state, map_stamp_, generation_};
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
    if (publishable && state.gait_clock_valid) {
      publishable = state.active_plan_stamp_ns == state_->active_plan_stamp_ns;
      reason = "active_plan_changed_after_solve";
      if (publishable) {
        const auto before = MakeHorizon(state.gait_time, robot_.period, robot_.duty);
        const auto after = MakeHorizon(state_->gait_time, robot_.period, robot_.duty);
        publishable = before.front().contacts == after.front().contacts &&
                      state.contact_confirmed == state_->contact_confirmed &&
                      (!state.body_clock_valid ||
                       (state.gait_paused == state_->gait_paused &&
                        (!input.hold_contacts || state.contact_estimated == state_->contact_estimated)));
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
      if (result.seed_available && !result.phases.empty() &&
          result.coefficients.size() == result.phases.size() &&
          result.margins.size() == result.phases.size() &&
          pending.generation == generation_ && state_ &&
          state.active_plan_stamp_ns == state_->active_plan_stamp_ns) {
        retry_seed_ = result;
        if (input.reference_prefix) {
          retry_seed_ = SpliceBody(*input.reference_prefix,
              BodyTime(input) - BodyTime(input.reference_prefix->seed_context),
              result, input.prefix_duration);
        }
        retry_seed_parent_ns_ = state.active_plan_stamp_ns;
      }
      PublishSolveStatus(result, input, state.header.stamp, false);
      PublishPipelineStatus("solve_failed", result.status.c_str());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "solve failed: %s", result.status.c_str());
      retry_requested_ = true;
    } else if (!publishable) {
      PublishSolveStatus(result, input, state.header.stamp, false);
      PublishPipelineStatus("publish_deferred", reason);
      retry_requested_ = true;
    } else {
      active_frame_ = state.header.frame_id;
      Publish(result, input, state);
      // 실제 발행한 몸통 곡선을 다음 계산의 초기값으로 보관한다.
      previous_ = published_plans_.back().body;
      retry_seed_.reset();
      PublishSolveStatus(result, input, state.header.stamp, true);
    }
    // 실패해도 마지막 성공해는 보존한다. Solve 내부가 시간·모델·지형 호환성을 재검사한다.
    // 계산 중 수신한 최신 상태 또는 같은 상태의 제한된 재시도를 즉시 시작한다.
    if (retry_requested_) OnTimer();
  }

  void Publish(const Result& solved, const Input& input,
               const b2_terrain_planner::msg::LocomotionState& state) {
    const rclcpp::Time stamp(state.header.stamp);
    const Feet& feet0 = input.feet;
    // 발 접촉 스케줄은 원래 solver 결과로 만들고 몸통만 정확히 이어 붙인다.
    Result foot_plan = solved;
    if (state.body_clock_valid) {
      foot_plan.phases = MakeHorizon(input.gait_time, robot_.period, robot_.duty);
    }
    const bool hold_gait = input.gait_hold_duration >= robot_.period;
    if (!hold_gait) UpdateSwingReferences(foot_plan, feet0, state);
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
    msg.parent_plan_stamp_ns = 0;
    if (input.reference_prefix) msg.parent_plan_stamp_ns = state.active_plan_stamp_ns;
    msg.handover_gait_time = input.gait_time;
    if (input.reference_prefix && !state.body_clock_valid) {
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

  // 상태.
  Robot robot_;
  Input nominal_input_;
  Settings settings_;
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d applied_command_ = Eigen::Vector3d::Zero();
  std::int64_t command_slot_ = -1;
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
  std::function<Result(const Robot&, const Input&, const Settings&, const Result*)> solver_ = Solve;
  std::future<std::pair<Result, std::int64_t>> solve_future_;
  std::optional<PendingSolve> pending_solve_;
  std::uint64_t generation_ = 0;
  bool retry_requested_ = false, same_state_retry_used_ = false;
  std::int64_t solve_state_stamp_ns_ = 0;

  // 파라미터.
  std::string state_topic_, state_frame_, map_topic_, elevation_layer_, plan_topic_, frame_;
  std::string active_frame_ = "odom";
  double plan_rate_hz_ = 10.0, sample_dt_ = 0.02, swing_height_ = 0.10;
  bool require_map_ = true;
  bool support_patch_enabled_ = false;
  double support_patch_half_width_ = 0.30;
  double state_max_age_ = 0.25, map_max_age_ = 1.0;

  rclcpp::Subscription<b2_terrain_planner::msg::LocomotionState>::SharedPtr state_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
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
