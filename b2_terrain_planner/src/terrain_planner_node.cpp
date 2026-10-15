/**
 * @file terrain_planner_node.cpp
 * @brief elevation map + odom을 구독해 TAMOLS 계획을 LocomotionPlan으로 발행하는 ROS2 노드.
 * @version 0.0.1
 *
 * 모든 계산은 odom 프레임에서 수행한다. 발행 궤적(몸통/네 발)은 WBC 추종 모드가 소비한다.
 * v1: 매 주기 현재 odom 자세 + 명목 스탠스에서 cold/warm Solve 후 horizon을 dt로 샘플링한다.
 * 측정 발 위치가 없으므로 스탠스는 명목(hips 아래, 지형 높이), 스윙 발은 포물선 arc로 생성한다.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "b2_terrain_planner/msg/locomotion_plan.hpp"

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
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_topic_ = declare_parameter<std::string>(
        "map_topic", "/elevation_mapping_node/elevation_map_raw");
    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/terrain_planner/plan");
    frame_ = declare_parameter<std::string>("frame", "odom");
    plan_rate_hz_ = declare_parameter<double>("plan_rate_hz", 10.0);
    sample_dt_ = declare_parameter<double>("sample_dt", 0.02);
    swing_height_ = declare_parameter<double>("swing_height", 0.10);
    require_map_ = declare_parameter<bool>("require_map", true);
    const auto cmd = declare_parameter<std::vector<double>>(
        "command", std::vector<double>{0.1, 0.0, 0.0});
    command_ = Eigen::Vector3d(cmd.size() > 0 ? cmd[0] : 0.0,
                               cmd.size() > 1 ? cmd[1] : 0.0,
                               cmd.size() > 2 ? cmd[2] : 0.0);

    InitRobot();
    ConfigureSettings();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = *msg; });
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        map_topic_, rclcpp::QoS(1).best_effort(),
        [this](grid_map_msgs::msg::GridMap::SharedPtr msg) { OnMap(*msg); });
    plan_pub_ = create_publisher<b2_terrain_planner::msg::LocomotionPlan>(plan_topic_, 1);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("marker_topic", "/terrain_planner/markers"), 1);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, plan_rate_hz_)),
        [this] { OnTimer(); });
    RCLCPP_INFO(get_logger(), "terrain_planner_node: odom=%s map=%s plan=%s rate=%.1fHz",
                odom_topic_.c_str(), map_topic_.c_str(), plan_topic_.c_str(), plan_rate_hz_);
  }

 private:
  void InitRobot() {
    auto model = std::make_shared<RobotModel>(DefaultModelPath());
    ModelState state;
    for (int leg = 0; leg < 4; ++leg) {
      state.joints.segment<3>(3 * leg) << 0, 44.30 * M_PI / 180, -85.94 * M_PI / 180;
    }
    state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
    model->PopulatePlanner(state, &robot_, &nominal_input_);
    nominal_base_z_ = nominal_input_.position[2];
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
    const int ev = LayerIndex(msg, elevation_layer_);
    if (ev < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "elevation layer '%s' missing", elevation_layer_.c_str());
      return;
    }
    const double res = msg.info.resolution;
    if (!(res > 0.0)) return;
    const int gm_rows = static_cast<int>(std::round(msg.info.length_x / res));  // x축 셀 수
    const int gm_cols = static_cast<int>(std::round(msg.info.length_y / res));  // y축 셀 수
    const auto& fa = msg.data[ev];
    if (gm_rows < 3 || gm_cols < 3 ||
        static_cast<int>(fa.data.size()) != gm_rows * gm_cols) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unexpected grid_map layout");
      return;
    }
    const int var = LayerIndex(msg, "variance");
    const double cx = msg.info.pose.position.x, cy = msg.info.pose.position.y;
    const int os = msg.outer_start_index, is = msg.inner_start_index;

    HeightMap hm;
    hm.resolution = res;
    hm.frame = msg.header.frame_id.empty() ? frame_ : msg.header.frame_id;
    // 내 격자: col↔x(증가), row↔y(증가), origin=최소 모서리.
    hm.origin = Eigen::Vector2d(cx - msg.info.length_x / 2.0, cy - msg.info.length_y / 2.0);
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
    // 스윙: 직전 정착 위치 → target 보간 + lift.
    Eigen::Vector3d from = feet0.row(leg).transpose();
    const double s = phase.duration > 1e-9
        ? std::clamp((t - phase.start) / phase.duration, 0.0, 1.0) : 0.0;
    Eigen::Vector3d foot = (1 - s) * from + s * target;
    foot.z() += swing_height_ * std::sin(M_PI * s);
    return {foot, false};
  }

  void OnTimer() {
    if (!odom_.has_value()) return;
    if (require_map_ && !terrain_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for elevation map");
      return;
    }
    const auto& od = *odom_;
    const Eigen::Quaterniond q(od.pose.pose.orientation.w, od.pose.pose.orientation.x,
                               od.pose.pose.orientation.y, od.pose.pose.orientation.z);
    const Eigen::Vector3d ypr = QuatToZyx(q);
    const Eigen::Vector3d base_xyz(od.pose.pose.position.x, od.pose.pose.position.y,
                                   od.pose.pose.position.z);

    Input input = nominal_input_;
    input.frame = frame_;
    input.terrain = terrain_;
    input.command = command_;
    input.gait_time = gait_time_;
    input.position << base_xyz.x(), base_xyz.y(), base_xyz.z(), ypr[0], ypr[1], ypr[2];
    // 월드 선속도(odom twist는 base 프레임 → 회전), euler rate는 v1에서 0.
    const Eigen::Vector3d v_body(od.twist.twist.linear.x, od.twist.twist.linear.y,
                                 od.twist.twist.linear.z);
    const Eigen::Vector3d v_world = q.toRotationMatrix() * v_body;
    input.velocity << v_world.x(), v_world.y(), v_world.z(), 0.0, 0.0, 0.0;
    // 명목 스탠스: hips 아래, 지형 높이(없으면 명목).
    const Eigen::Matrix2d rz = Eigen::Rotation2Dd(ypr[0]).toRotationMatrix();
    Feet feet = Feet::Zero();
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector2d off = rz * robot_.hips.row(leg).head<2>().transpose();
      const double fx = base_xyz.x() + off.x();
      const double fy = base_xyz.y() + off.y();
      feet(leg, 0) = fx;
      feet(leg, 1) = fy;
      feet(leg, 2) = TerrainHeight(fx, fy, base_xyz.z() - nominal_base_z_);
    }
    input.feet = feet;

    Result result = Solve(robot_, input, settings_,
                          previous_.has_value() ? &*previous_ : nullptr);
    if (!result.success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "solve failed: %s", result.status.c_str());
      previous_.reset();
      return;
    }
    previous_ = result;
    gait_time_ += 1.0 / std::max(1.0, plan_rate_hz_);
    // 발행 프레임은 odom 메시지 프레임에 맞춘다(배포가 odom/elevation_odom 어느 쪽이든 일치).
    active_frame_ = od.header.frame_id.empty() ? frame_ : od.header.frame_id;
    Publish(result, input.feet, od.header.stamp);
  }

  double TerrainHeight(double x, double y, double fallback) const {
    if (!terrain_.has_value()) return fallback;
    const auto& t = *terrain_;
    const int col = static_cast<int>(std::round((x - t.origin.x()) / t.resolution));
    const int row = static_cast<int>(std::round((y - t.origin.y()) / t.resolution));
    if (row < 0 || col < 0 || row >= t.heights.rows() || col >= t.heights.cols())
      return fallback;
    if (t.valid(row, col) == 0) return fallback;
    return t.heights(row, col);
  }

  void Publish(const Result& r, const Feet& feet0, const rclcpp::Time& stamp) {
    double horizon = 0.0;
    for (const auto& p : r.phases) horizon += p.duration;
    const int n = std::max(1, static_cast<int>(std::round(horizon / sample_dt_)) + 1);

    b2_terrain_planner::msg::LocomotionPlan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = active_frame_;
    msg.dt = sample_dt_;
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
        const auto [foot, contact] = FootAt(r, feet0, leg, t);
        const size_t fi = static_cast<size_t>(i) * 4 + leg;
        msg.foot_position[fi].x = foot.x();
        msg.foot_position[fi].y = foot.y();
        msg.foot_position[fi].z = foot.z();
        msg.foot_contact[fi] = contact;
      }
    }
    plan_pub_->publish(msg);
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
  double nominal_base_z_ = 0.55;
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  double gait_time_ = 0.0;
  std::optional<HeightMap> terrain_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<Result> previous_;

  // 파라미터.
  std::string odom_topic_, map_topic_, elevation_layer_, plan_topic_, frame_;
  std::string active_frame_ = "odom";
  double plan_rate_hz_ = 10.0, sample_dt_ = 0.02, swing_height_ = 0.10;
  bool require_map_ = true;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Publisher<b2_terrain_planner::msg::LocomotionPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  bool publish_markers_ = true;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
