/**
 * @file planner_retry_test.cpp
 * @brief 지연·실패를 주입해 비동기 재계획과 성공해 보존 및 발행 차단을 검증한다.
 * @version 0.0.28
 * @update 2026-11-09: 착지 대기 solve 연속 실패 시 감속 유지 계획 fallback 발행을 검증함.
 * @update 2026-11-05: 선택 제약 목록만 재사용하는 기본값과 설정의 worker 전달을 검증함.
 */
#define B2_TERRAIN_PLANNER_NO_MAIN
#include "../../src/terrain_planner_node.cpp"

#include <atomic>
#include <iostream>
#include <thread>

struct TerrainPlannerRetryTest {
  struct Attempt {
    std::atomic<bool> started{false};
    bool had_previous = false;
    bool had_failed_seed = false;
    bool skip_projection = false;
    bool candidate_retries = true;
    bool qp_working_set = true;
    bool reuse_qp_working_set = false;
    bool reuse_qp_direction = false;
    double gait_time = -1;
    double body_time = -1;
    double hold_duration = -1;
    std::optional<std::array<bool, 4>> hold_contacts;
    std::optional<std::array<bool, 4>> contact_estimated;
    Eigen::Vector3d command = Eigen::Vector3d::Zero();
    std::promise<void> release;
    bool success = true;
    double violation = 0.0;
  };

  static void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  }

  template <class Predicate>
  static void Wait(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate()) {
      Check(std::chrono::steady_clock::now() < deadline, "test operation timed out");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // 수치 최적화 대신 명시적 gate로 종료 시점을 통제한다. 실제 Solve 검증은 기존 시험이 담당한다.
  static std::shared_ptr<Attempt> Install(TerrainPlannerNode& node, bool success = true) {
    auto attempt = std::make_shared<Attempt>();
    attempt->success = success;
    node.solver_ = [attempt](const Robot& robot, const Input& input, const Settings& settings,
                            const Result* previous) {
      attempt->skip_projection = settings.skip_line_search_projection;
      attempt->candidate_retries = settings.terrain_candidate_retries;
      attempt->qp_working_set = settings.qp_working_set;
      attempt->reuse_qp_working_set = settings.reuse_qp_working_set;
      attempt->reuse_qp_direction = settings.reuse_qp_direction;
      attempt->had_previous = previous != nullptr && previous->success;
      attempt->had_failed_seed = previous != nullptr && !previous->success;
      attempt->gait_time = input.gait_time;
      attempt->body_time = BodyTime(input);
      attempt->hold_duration = input.gait_hold_duration;
      attempt->hold_contacts = input.hold_contacts;
      attempt->contact_estimated = input.contact_estimated;
      attempt->command = input.command;
      auto release = attempt->release.get_future();
      attempt->started.store(true);
      release.wait();
      Result result;
      result.success = attempt->success;
      result.status = "time_limit";
      if (result.success) result.status = "test_success";
      result.violation = attempt->violation;
      // 실제 Solve의 warm start 사유 규약을 흉내 내 시드 체인 판정을 검증한다.
      if (attempt->had_previous) result.warm_start.reason = "shifted_previous_plan";
      result.seed_context.gait_time = input.gait_time;
      result.seed_context.body_time = input.body_time;
      result.seed_context.gait_hold_duration = input.gait_hold_duration;
      result.seed_context.hold_contacts = input.hold_contacts;
      result.phases = MakeHorizon(input.gait_time, robot.period, robot.duty,
          input.gait_hold_duration, input.hold_contacts);
      result.footholds = input.feet;
      for (const auto& phase : result.phases) {
        (void)phase;
        Eigen::Matrix<double, 6, 5> coefficients = Eigen::Matrix<double, 6, 5>::Zero();
        coefficients.col(0) = input.position;
        result.coefficients.push_back(coefficients);
        result.margins.push_back(0.0);
      }
      return result;
    };
    return attempt;
  }

  static void Started(const std::shared_ptr<Attempt>& attempt) {
    Wait([&] { return attempt->started.load(); });
  }

  static void Finish(TerrainPlannerNode& node, const std::shared_ptr<Attempt>& attempt) {
    attempt->release.set_value();
    Wait([&] { return node.solve_future_.wait_for(std::chrono::seconds(0)) ==
                      std::future_status::ready; });
    node.CompleteSolve();
  }

  static void CheckCommandRamp() {
    TerrainPlannerNode node;
    node.timer_->cancel();
    node.command_ << .3, .4, 1.0;
    TerrainPlannerNode::State state;
    state.gait_clock_valid = true;
    state.body_clock_valid = true;
    state.gait_time = .2;  // 같은 스윙 안에서도 램프가 진행돼야 한다.
    state.body_time = 10.0;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isZero(), "initial ramp jumped to request");
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(.03, .04, .1), 1e-10),
          "XY vector or yaw ramp rate is incorrect");
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(.03, .04, .1), 1e-10),
          "repeated state advanced ramp");
    node.command_ << -.3, -.4, -1.0;
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isZero(1e-10), "reversal jumped across zero");
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.03, -.04, -.1), 1e-10),
          "reversal did not continue smoothly");
    state.gait_time = .4;
    state.contact_confirmed.fill(true);
    state.contact_confirmed[2] = false;
    node.UpdateTouchdownStop(state);
    Check(node.PlanningCommand().isZero(), "touchdown stop lost priority");
    state.body_time += .5;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isZero() &&
          node.applied_command_.isApprox(Eigen::Vector3d(-.18, -.24, -.6), 1e-10),
          "touchdown wait stopped the user ramp or lost zero override");
    state.contact_confirmed.fill(true);
    node.UpdateTouchdownStop(state);
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.18, -.24, -.6), 1e-10),
          "resume did not use the preserved ramp");
    state.body_time += .02;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.186, -.248, -.62), 1e-10),
          "resume restarted the user ramp");
    state.body_time += 2.0;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(node.command_), "ramp overshot target");
    node.command_topic_ = "/test/command";
    node.command_stamp_.reset();
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.27, -.36, -.9), 1e-10),
          "expired command did not ramp toward zero");
    // 반복 대기마다 램프를 초기화하면 최대 속도에 영원히 도달하지 못하는 회귀를 재현한다.
    node.command_topic_.clear();
    node.command_ << .5, 0.0, .5;
    node.applied_command_.setZero();
    for (int tick = 1; tick <= 12; ++tick) {
      state.contact_confirmed[2] = tick % 2 == 0;
      node.UpdateTouchdownStop(state);
      state.body_time += .1;
      node.UpdateCommandRamp(state);
      Check(std::abs(node.applied_command_.x() - std::min(.5, .05 * tick)) < 1e-10,
            "repeated touchdown waits reset the user ramp");
      if (node.touchdown_stop_) {
        Check(node.PlanningCommand().isZero(), "wait lost zero override");
      } else {
        Check(node.PlanningCommand().isApprox(node.applied_command_),
              "confirmed support did not expose the ongoing ramp");
      }
    }
    Check(node.PlanningCommand().isApprox(node.command_),
          "repeated touchdown waits prevented full command speed");
    // 접촉이 모두 확인돼도 몸통 계획 인계로 보행이 멈춰 있으면 감속한다.
    state.gait_paused = true;
    state.contact_confirmed.fill(true);
    node.UpdateTouchdownStop(state);
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(.45, 0, .4), 1e-10),
          "paused gait did not ramp linear and yaw commands toward zero");
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(.45, 0, .4), 1e-10),
          "repeated state advanced paused ramp");
    state.body_time += 1.0;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isZero(1e-10), "paused ramp overshot zero");
    state.gait_paused = false;
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(.05, 0, .1), 1e-10),
          "gait resume jumped to the original command");
  }

  static void CheckTouchdownStop(TerrainPlannerNode& node) {
    node.touchdown_stop_ = false;
    node.applied_command_ << .5, 0.0, .5;
    TerrainPlannerNode::State state;
    state.gait_clock_valid = true;
    state.body_clock_valid = true;
    state.gait_time = .4;
    state.body_time = 10.0;
    state.contact_confirmed.fill(true);
    Check(!node.UpdateTouchdownStop(state) && !node.touchdown_stop_ &&
          node.PlanningCommand().x() == .5, "ordinary support triggered stopping");
    state.contact_confirmed[2] = false;
    Check(node.UpdateTouchdownStop(state) && node.PlanningCommand().isZero() &&
          node.touchdown_stop_,
          "missing landing did not immediately request zero velocity");
    for (double time : {10.0, 10.1, 10.5, 10.9}) {
      state.body_time = time;
      state.contact_estimated.fill(true);
      Check(!node.UpdateTouchdownStop(state) && node.touchdown_stop_ &&
            node.PlanningCommand().isZero(), "retry or raw contact released the zero-velocity command");
    }
    state.contact_confirmed.fill(true);
    Check(node.UpdateTouchdownStop(state) && !node.touchdown_stop_ &&
          node.PlanningCommand().x() == .5, "confirmed landing did not restore preserved command");
  }

  static void CheckSwingReferenceUpdates(TerrainPlannerNode& node) {
    node.swing_references_ = {};
    TerrainPlannerNode::State state;
    state.gait_clock_valid = true;
    state.body_clock_valid = true;
    state.contact_confirmed.fill(true);
    Feet measured = node.nominal_input_.feet;
    Result plan;
    plan.footholds = measured;
    plan.footholds.col(0).array() += .1;
    HeightMap swing_map;
    swing_map.resolution = 0.01;
    swing_map.origin = measured.row(1).head<2>().transpose() - Eigen::Vector2d(.2, .2);
    swing_map.heights = Eigen::MatrixXd::Constant(41, 61, measured(1, 2) - node.foot_radius_);
    swing_map.valid = Eigen::ArrayXXi::Ones(41, 61);
    const auto update = [&](double gait_time) {
      state.gait_time = gait_time;
      plan.seed_context.gait_time = gait_time;
      plan.phases = MakeHorizon(gait_time, node.robot_.period, node.robot_.duty);
      node.UpdateSwingReferences(plan, measured, state, &swing_map);
    };
    update(0.0);
    const auto original = node.swing_references_[1];
    Check(original.start > 0 && original.end > original.start,
          "test did not start before FR liftoff");
    // 최신 실패 로그의 약 40cm 수평 이동을 재현한다. 다음 이륙 시각은 그대로다.
    const Eigen::Vector3d slip(-.37, -.17, .005);
    measured.row(1) += slip.transpose();
    update(.04);
    const auto refreshed = node.swing_references_[1];
    Check(std::abs(refreshed.start - original.start) < 1e-12 &&
          std::abs(refreshed.end - original.end) < 1e-12,
          "pre-liftoff refresh changed trot timing");
    Check((refreshed.Sample(refreshed.start).first - measured.row(1).transpose()).norm() < 1e-12 &&
          (refreshed.position - original.position).norm() > .4,
          "pre-liftoff swing retained the stale start after foot movement");
    Check(refreshed.Sample(refreshed.start).second.isZero(1e-12),
          "future stance liftoff inherited an old swing velocity");
    // 보행 대기 중 몸통 시각만 진행해도 아직 이륙하지 않은 시작점·정점은 갱신한다.
    state.gait_paused = true;
    for (double height : {.04, .01}) {
      state.body_time += .2;
      measured(1, 2) = height;
      plan.footholds(1, 0) += .02;
      update(.04);
      const auto& future = node.swing_references_[1];
      Check((future.position - measured.row(1).transpose()).norm() < 1e-12 &&
            (future.target - plan.footholds.row(1).transpose()).norm() < 1e-12 &&
            std::abs(future.apex_z -
                     (std::max(height, future.target.z()) + node.swing_height_)) < 1e-12,
            "paused future swing retained stale start, target or apex");
    }
    state.gait_paused = false;
    // 최종 이륙 후보의 앞 6cm부터 15cm 상승 단차를 둔다.
    swing_map.origin = measured.row(1).head<2>().transpose() - Eigen::Vector2d(.2, .2);
    swing_map.heights.setConstant(measured(1, 2) - node.foot_radius_);
    swing_map.heights.rightCols(35).array() += .15;
    plan.footholds(1, 2) = measured(1, 2) + .15;
    update(.04);
    const auto committed = node.swing_references_[1];
    Check(committed.shape == SwingReference::Shape::kStepUp,
          "nearby riser did not select rounded swing through node");
    swing_map.heights.setZero();
    swing_map.valid.setZero();
    const bool enabled = node.step_swing_.enabled;
    node.step_swing_.enabled = false;
    state.contact_confirmed[1] = false;
    state.contact_confirmed[2] = false;
    // 이륙 경계와 진행 중에는 추정 위치·후보 착지점이 달라져도 실행 곡선을 보존한다.
    for (double time : {committed.start, committed.start + .04, committed.end - .02}) {
      measured.row(1).array() += .03;
      plan.footholds.row(1).array() += .05;
      update(time);
      const auto& active = node.swing_references_[1];
      Check(std::abs(active.start - committed.start) < 1e-12 &&
            std::abs(active.end - committed.end) < 1e-12 &&
            (active.target - committed.target).norm() < 1e-12,
            "active swing timing or touchdown changed");
      for (double sample_time : {time, (time + committed.end) * .5, committed.end}) {
        const auto before = committed.Sample(sample_time);
        const auto after = active.Sample(sample_time);
        Check((before.first - after.first).norm() < 1e-12 &&
              (before.second - after.second).norm() < 1e-12,
              "active swing position/velocity curve changed");
      }
      Input input;
      input.gait_time = time;
      Check(CommitActiveSwingFootholds(node.robot_, node.swing_references_, &input) &&
            input.committed_footholds[1].has_value() &&
            (*input.committed_footholds[1] - committed.target).norm() < 1e-12,
            "active swing target was not fixed in optimizer input");
    }
    // 착지 후 다음 주기의 이륙점은 다시 현재 발 위치를 사용한다.
    state.contact_confirmed.fill(true);
    measured.row(1).array() -= .02;
    update(committed.end + .01);
    Check((node.swing_references_[1].position - measured.row(1).transpose()).norm() < 1e-12 &&
          node.swing_references_[1].start > committed.end,
          "next swing inherited the completed swing start");
    node.step_swing_.enabled = enabled;
  }

  static void CheckSnapshotCompletion() {
    TerrainPlannerNode node;
    node.timer_->cancel();
    node.require_map_ = false;
    node.publish_markers_ = false;
    node.command_.setZero();
    node.state_max_age_ = .25;
    node.map_max_age_ = .25;
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.base_orientation.w = 1;
    state.com_position.x = node.nominal_input_.position.x();
    state.com_position.y = node.nominal_input_.position.y();
    state.com_position.z = node.nominal_input_.position.z();
    state.gait_clock_valid = true;
    state.gait_epoch_ns = 1;
    state.contact_confirmed.fill(true);
    state.contact_estimated.fill(true);
    for (int leg = 0; leg < 4; ++leg) {
      state.foot_position[leg].x = node.nominal_input_.feet(leg, 0);
      state.foot_position[leg].y = node.nominal_input_.feet(leg, 1);
      state.foot_position[leg].z = node.nominal_input_.feet(leg, 2);
    }
    auto baseline = Install(node);
    state.header.stamp = node.now();
    node.OnState(state);
    Started(baseline);
    Finish(node, baseline);

    HeightMap map;
    map.origin = Eigen::Vector2d::Constant(-1);
    map.heights = Eigen::MatrixXd::Zero(41, 41);
    map.variance = Eigen::MatrixXd::Zero(41, 41);
    map.valid = Eigen::ArrayXXi::Ones(41, 41);
    node.terrain_ = map;
    // 시각을 주입해 sleep 없이 오래된 시작 snapshot과 최신 입력을 분리한다.
    // 앞 두 경우는 0/양수 prefix 마감 경과, 뒤 두 경우는 최신 상태/지도 단절이다.
    for (int scenario = 0; scenario < 4; ++scenario) {
      state.gait_time = .01 + .02 * scenario;
      state.header.stamp = node.now();
      node.map_stamp_ = node.now();
      auto attempt = Install(node);
      node.OnState(state);
      Started(attempt);
      const auto old_stamp = node.now() - rclcpp::Duration::from_seconds(.7);
      node.pending_solve_->state.header.stamp = old_stamp;
      node.pending_solve_->map_stamp = old_stamp;
      node.pending_solve_->input.reference_prefix = std::make_shared<Result>(*node.previous_);
      node.pending_solve_->input.prefix_duration = 0;
      if (scenario == 1) node.pending_solve_->input.prefix_duration = .001;
      node.state_->gait_time += .005;
      Check(!node.Fresh(node.pending_solve_->state.header.stamp, node.state_max_age_) &&
            !node.Fresh(node.pending_solve_->map_stamp, node.map_max_age_),
            "test snapshot did not exceed the former age limits");
      if (scenario == 2) node.state_->header.stamp = old_stamp;
      if (scenario == 3) node.map_stamp_ = old_stamp;
      const auto before = node.last_published_stamp_ns_;
      Finish(node, attempt);
      Check(!node.solve_future_.valid(), "snapshot test unexpectedly started a second solve");
      if (scenario < 2) {
        Check(node.last_published_stamp_ns_ == old_stamp.nanoseconds(),
              "old start snapshot or prefix deadline blocked publication");
      } else {
        Check(node.last_published_stamp_ns_ == before,
              "stale latest state or map was allowed to publish");
      }
    }
  }

  // 착지 대기(pause) 중 solve 연속 실패로 발행이 정체되면 SQP 없이 감속 유지
  // 계획을 공급하는지 검증한다. 보행 중 실패에는 발동하지 않아야 한다.
  static void CheckBrakeFallback() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("brake_fallback.enabled", true),
                                 rclcpp::Parameter("brake_fallback.stall_sec", 0.0),
                                 rclcpp::Parameter("solver_diagnostic_directory", "")});
    TerrainPlannerNode node(options);
    node.timer_->cancel();
    node.require_map_ = false;
    node.publish_markers_ = false;
    node.state_max_age_ = 10;
    node.command_.setZero();
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.base_orientation.w = 1;
    state.com_position.x = node.nominal_input_.position.x();
    state.com_position.y = node.nominal_input_.position.y();
    state.com_position.z = node.nominal_input_.position.z();
    state.gait_clock_valid = true;
    state.body_clock_valid = true;
    state.gait_epoch_ns = 1;
    state.gait_time = .01;
    state.body_time = 10.0;
    state.contact_confirmed.fill(true);
    state.contact_estimated.fill(true);
    for (int leg = 0; leg < 4; ++leg) {
      state.foot_position[leg].x = node.nominal_input_.feet(leg, 0);
      state.foot_position[leg].y = node.nominal_input_.feet(leg, 1);
      state.foot_position[leg].z = node.nominal_input_.feet(leg, 2);
    }
    // 기준 발행: fallback의 stall 계산은 마지막 발행 stamp가 필요하다.
    auto baseline = Install(node);
    state.header.stamp = node.now();
    node.OnState(state);
    Started(baseline);
    Finish(node, baseline);
    const auto published = node.published_plans_.size();
    Check(published >= 1 && node.last_published_stamp_ns_ > 0, "baseline publish missing");
    // 보행 중(비pause) 실패: fallback이 발동하면 안 된다.
    auto walking_failure = Install(node, false);
    state.gait_time = .03;
    state.body_time = 10.1;
    state.header.stamp = node.now();
    node.OnState(state);
    Started(walking_failure);
    Finish(node, walking_failure);
    Check(node.published_plans_.size() == published, "brake fallback fired while walking");
    // pause + 실패 + 발행 정체: 감속 유지 계획이 발행돼야 한다.
    auto paused_failure = Install(node, false);
    state.gait_time = .05;
    state.body_time = 10.2;
    state.gait_paused = true;
    state.com_velocity.x = .6;
    state.header.stamp = node.now();
    node.OnState(state);
    Started(paused_failure);
    Finish(node, paused_failure);
    Check(node.published_plans_.size() == published + 1, "brake fallback plan not published");
    const Result& brake = node.published_plans_.back().body;
    Check(brake.status == "brake_fallback" && brake.phases.size() == 1 &&
              brake.phases.front().duration == node.robot_.period,
          "brake plan shape mismatch");
    Check(std::abs(Evaluate(brake, 0.0, 1)[0] - .6) < 1e-9,
          "brake plan start velocity mismatch");
    Check(std::abs(Evaluate(brake, node.robot_.period, 1)[0]) < 1e-9,
          "brake plan did not reach zero velocity at horizon");
    Check(std::abs(Evaluate(brake, 0.0, 0)[0] - state.com_position.x) < 1e-9,
          "brake plan start position mismatch");
    Check(node.previous_ && node.previous_->status != "brake_fallback",
          "warm start source replaced by the fallback plan");
    // 종료 뒤 자동 시작한 모의 worker가 대기한 채 남지 않게 입력을 비운다.
    node.state_.reset();
    if (node.solve_future_.valid()) node.solve_future_.wait();
    std::cout << "PASS brake fallback publication on paused solve failures\n";
  }

  static void CheckSolverExperiment(bool working_set, bool reuse_working_set) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("qp_working_set", working_set),
                                 rclcpp::Parameter("reuse_qp_working_set", reuse_working_set),
                                 rclcpp::Parameter("line_search_projection", false),
                                 rclcpp::Parameter("terrain.candidate_retries", false),
                                 rclcpp::Parameter("solver_diagnostic_directory", "")});
    TerrainPlannerNode node(options);
    node.timer_->cancel();
    Check(node.settings_.skip_line_search_projection && !node.settings_.terrain_candidate_retries,
          "ROS experiment options not applied");
    node.require_map_ = false;
    node.publish_markers_ = false;
    node.state_max_age_ = 10;
    node.command_.setZero();
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.base_orientation.w = 1;
    state.com_position.x = node.nominal_input_.position.x();
    state.com_position.y = node.nominal_input_.position.y();
    state.com_position.z = node.nominal_input_.position.z();
    state.gait_clock_valid = true;
    state.gait_epoch_ns = 1;
    state.contact_estimated.fill(true);
    state.contact_confirmed.fill(true);
    for (int leg = 0; leg < 4; ++leg) {
      state.foot_position[leg].x = node.nominal_input_.feet(leg, 0);
      state.foot_position[leg].y = node.nominal_input_.feet(leg, 1);
      state.foot_position[leg].z = node.nominal_input_.feet(leg, 2);
    }
    auto failed = Install(node, false);
    state.header.stamp = node.now();
    node.OnState(state);
    Started(failed);
    auto latest = Install(node, false);
    state.gait_time = .02;
    state.header.stamp = node.now();
    node.OnState(state);
    Finish(node, failed);
    Started(latest);
    const bool failed_options = failed->skip_projection && !failed->candidate_retries &&
                                failed->qp_working_set == working_set &&
                                failed->reuse_qp_working_set == reuse_working_set &&
                                !failed->reuse_qp_direction;
    const bool latest_options = latest->skip_projection && !latest->candidate_retries &&
                                latest->qp_working_set == working_set &&
                                latest->reuse_qp_working_set == reuse_working_set &&
                                !latest->reuse_qp_direction;
    const bool fresh_state = latest->gait_time == .02 && !latest->had_previous &&
                             !latest->had_failed_seed;
    // 종료 뒤 자동 시작할 입력을 비워 모의 worker가 대기한 채 남지 않게 한다.
    node.state_.reset();
    Finish(node, latest);
    Check(failed_options && latest_options, "worker lost experiment settings");
    Check(fresh_state, "experiment failure did not replan from latest state");
  }

  static void Run() {
    CheckSolverExperiment(false, false);
    CheckSolverExperiment(false, true);
    CheckSolverExperiment(true, false);
    CheckSolverExperiment(true, true);
    TerrainPlannerNode node;
    node.timer_->cancel();
    Check(node.settings_.qp_working_set, "online default lost QP constraint selection");
    Check(node.settings_.reuse_qp_working_set && !node.settings_.reuse_qp_direction,
          "online default must reuse only the selected constraint list");
    Check(node.settings_.time_limit == 0, "online default still has a time limit");
    Check(!node.settings_.skip_line_search_projection && node.settings_.terrain_candidate_retries,
          "online defaults changed without experiment options");
    node.require_map_ = false;
    node.publish_markers_ = false;
    node.state_max_age_ = 10;
    node.command_.setZero();
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.base_orientation.w = 1;
    state.com_position.x = node.nominal_input_.position.x();
    state.com_position.y = node.nominal_input_.position.y();
    state.com_position.z = node.nominal_input_.position.z();
    state.gait_clock_valid = true;
    state.gait_epoch_ns = 1;
    state.contact_estimated.fill(true);
    state.contact_confirmed.fill(true);
    for (int leg = 0; leg < 4; ++leg) {
      state.foot_position[leg].x = node.nominal_input_.feet(leg, 0);
      state.foot_position[leg].y = node.nominal_input_.feet(leg, 1);
      state.foot_position[leg].z = node.nominal_input_.feet(leg, 2);
    }
    const auto update = [&](double gait_time) {
      state.gait_time = gait_time;
      state.header.stamp = node.now();
      node.OnState(state);
    };

    auto cold_failure = Install(node, false);
    update(0);
    Started(cold_failure);
    auto first = Install(node);
    update(0);
    Finish(node, cold_failure);
    node.OnTimer();
    Started(first);
    Check(!first->had_failed_seed, "initial failure was reused as a warm start");
    Check(!first->had_previous, "first solve unexpectedly had a seed");
    // 재계획 타이머를 취소한 상태에서도 worker 완료 알림만으로 발행해야 한다.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    first->release.set_value();
    const auto completion_deadline = std::chrono::steady_clock::now() +
                                     std::chrono::seconds(2);
    while (node.solve_future_.valid() &&
           std::chrono::steady_clock::now() < completion_deadline) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(node.get_node_base_interface());
    Check(!node.solve_future_.valid(), "completion notification was lost");
    Check(node.previous_.has_value() && node.last_publish_ns_ > 0, "initial plan not published");
    Check(node.completion_dispatch_ns_ >= node.solve_end_ns_ &&
          node.last_publish_ns_ >= node.completion_dispatch_ns_,
          "completion timestamps do not separate dispatch and publication");

    // 성공 후에도 타이머 없이 이어서 계산하고, 계산 중 받은 상태는 최신 하나만 사용한다.
    auto success = Install(node);
    update(.001);
    Started(success);
    const auto before_updates = node.attempt_seq_;
    update(.002);
    update(.003);
    node.OnTimer();
    Check(node.attempt_seq_ == before_updates && success->gait_time == .001,
          "incoming states started overlapping solves or changed the running snapshot");
    auto latest = Install(node);
    const auto success_stamp = node.solve_state_stamp_ns_;
    Finish(node, success);
    Started(latest);
    Check(node.last_published_stamp_ns_ == success_stamp &&
          node.attempt_seq_ == before_updates + 1 && latest->gait_time == .003 &&
          latest->had_previous,
          "successful solve did not publish and immediately start the latest state");
    Finish(node, latest);
    node.OnTimer();
    node.OnState(state);
    Check(!node.solve_future_.valid() && node.attempt_seq_ == before_updates + 1,
          "successful state was solved again without new input");

    auto failed = Install(node, false);
    update(.01);
    node.OnTimer();
    Started(failed);
    Check(failed->had_previous, "last success missing before failure");
    update(.02);
    Check(node.state_->gait_time == .02, "state reception blocked during solve");
    auto retry = Install(node);
    const auto before_retry = node.attempt_seq_;
    Finish(node, failed);
    Started(retry);
    Check(node.attempt_seq_ == before_retry + 1, "retry waited for periodic timer");
    Check(retry->had_previous && retry->gait_time == .02, "retry lost seed or latest state");
    Finish(node, retry);

    auto same = Install(node, false);
    update(.03);
    node.OnTimer();
    Started(same);
    const auto before_same = node.attempt_seq_;
    Finish(node, same);
    node.OnTimer();
    node.OnState(state);
    Check(!node.solve_future_.valid() && node.attempt_seq_ == before_same &&
          node.previous_.has_value(),
          "identical failed input was retried or successful seed was erased");
    auto recovered = Install(node);
    update(.04);
    Started(recovered);
    Check(recovered->had_previous, "new state after failures lost seed");
    Finish(node, recovered);

    auto old_epoch = Install(node);
    update(.05);
    node.OnTimer();
    Started(old_epoch);
    const auto old_publish = node.last_published_stamp_ns_;
    state.gait_epoch_ns = 2;
    update(0);
    auto new_epoch = Install(node);
    Finish(node, old_epoch);
    Started(new_epoch);
    Check(node.last_published_stamp_ns_ == old_publish && !new_epoch->had_previous,
          "old epoch result published or seed reused");
    Finish(node, new_epoch);

    auto paused = Install(node);
    update(.01);
    node.OnTimer();
    Started(paused);
    const auto before_pause = node.last_published_stamp_ns_;
    state.gait_paused = true;
    state.contact_confirmed[1] = false;
    update(.02);
    Finish(node, paused);
    Check(!node.solve_future_.valid() && node.last_published_stamp_ns_ == before_pause &&
          node.previous_.has_value(), "pause failed to block publication or erased seed");
    auto resumed = Install(node);
    // 착지는 확인됐지만 WBC가 계획을 기다리는 pause도 계산을 허용한다.
    state.contact_confirmed[1] = true;
    update(.02);
    Started(resumed);
    Finish(node, resumed);

    auto changed_command = Install(node);
    update(.03);
    node.OnTimer();
    Started(changed_command);
    const auto before_command = node.last_published_stamp_ns_;
    node.command_.x() = .1;
    Finish(node, changed_command);
    Check(node.last_published_stamp_ns_ > before_command && !node.solve_future_.valid(),
          "incoming command discarded successful snapshot");
    Check(node.applied_command_.isZero(), "command advanced without a new state time");
    state.gait_paused = false;
    auto next_command = Install(node);
    update(.4);
    node.OnTimer();
    Started(next_command);
    Check(next_command->command.x() > 0 && next_command->command.x() <= .1,
          "new state did not advance command ramp");
    node.command_.x() = -.2;
    Finish(node, next_command);
    Check(node.applied_command_.isApprox(next_command->command),
          "command advanced during solve without new state time");

    auto partial = Install(node, false);
    update(.405);
    node.OnTimer();
    Started(partial);
    const auto before_partial = node.last_published_stamp_ns_;
    const auto successful_seed_gait = node.previous_->seed_context.gait_time;
    auto continued = Install(node);
    update(.406);
    Finish(node, partial);
    Started(continued);
    Check(!continued->had_failed_seed && continued->had_previous,
          "next state reused a failed iterate instead of the last success");
    Check(node.last_published_stamp_ns_ == before_partial &&
          node.previous_->seed_context.gait_time == successful_seed_gait,
          "failed iterate overwrote or published a successful plan");
    Finish(node, continued);

    auto reversed = Install(node);
    update(.41);
    node.OnTimer();
    Started(reversed);
    const auto before_reset = node.last_published_stamp_ns_;
    node.last_clock_ns_ = node.now().nanoseconds() + 1000000000;
    Finish(node, reversed);
    Check(!node.state_ && !node.previous_ && !node.solve_future_.valid() &&
          node.last_published_stamp_ns_ == before_reset, "clock reset accepted stale result");
    // 별도 몸통 clock은 미착지 중에도 현재 두 지지로 계획하고, 재개 시각을 고정한다.
    state.body_clock_valid = true;
    state.body_time = .4;
    state.gait_epoch_ns = 3;
    state.gait_paused = true;
    state.contact_confirmed = {true, false, false, true};
    state.contact_estimated.fill(false);
    auto hold = Install(node);
    update(.4);
    node.OnTimer();
    Started(hold);
    const auto before_hold = node.last_published_stamp_ns_;
    const auto hold_estimated = state.contact_estimated;
    state.contact_estimated.fill(true);
    update(.4);
    auto hold_refresh = Install(node);
    Finish(node, hold);
    Check(hold->command.isZero(), "hold did not request zero velocity");
    Check(hold->hold_duration == node.robot_.period && hold->body_time == .4 &&
          hold->hold_contacts == state.contact_confirmed, "touchdown wait blocked rolling body solve");
    Check(hold->contact_estimated == hold_estimated, "stance overwrote measured terrain contacts");
    Check(node.last_published_stamp_ns_ > before_hold,
          "estimated contact change rejected two-stance hold plan");
    Started(hold_refresh);
    Finish(node, hold_refresh);
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto tripod = Install(node);
    state.contact_confirmed[1] = true;
    state.contact_estimated.fill(false);
    state.body_time = .42;
    update(.4);
    Started(tripod);
    const auto before_tripod = node.last_published_stamp_ns_;
    Finish(node, tripod);
    Check(tripod->command.isZero(), "additional support released the zero-velocity command");
    Check(tripod->hold_contacts == state.contact_confirmed &&
          tripod->hold_duration == node.robot_.period &&
          node.last_published_stamp_ns_ > before_tripod,
          "three-stance hold changed supports or blocked publication");
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto release = Install(node);
    // 착지 후 추정 접촉이 모두 OFF여도 유한한 재개 prefix만 사용한다.
    state.contact_confirmed.fill(true);
    state.contact_estimated.fill(false);
    state.body_time = .44;
    update(.4);
    Started(release);
    const double resume_time = release->body_time + release->hold_duration;
    const auto before_release = node.last_published_stamp_ns_;
    state.contact_estimated.fill(true);
    update(.4);
    auto release_refresh = Install(node);
    Finish(node, release);
    Check(release->hold_duration > 0 && release->hold_duration < .8, "resume prefix missing");
    Check(node.last_published_stamp_ns_ > before_release,
          "estimated contact change rejected stance resume plan");
    Started(release_refresh);
    Finish(node, release_refresh);
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto refresh = Install(node);
    state.contact_estimated.fill(false);
    state.body_time = .46;
    update(.4);
    node.OnTimer();
    Started(refresh);
    const auto before_refresh = node.last_published_stamp_ns_;
    Finish(node, refresh);
    Check(std::abs(refresh->body_time + refresh->hold_duration - resume_time) < 1e-12,
          "rolling replanning postponed gait resume forever");
    Check(node.last_published_stamp_ns_ > before_refresh,
          "estimated contact OFF blocked stance resume refresh");
    CheckSnapshotCompletion();
    CheckCommandRamp();
    CheckTouchdownStop(node);
    CheckSwingReferenceUpdates(node);
    CheckBrakeFallback();

    Input held_model = node.nominal_input_;
    held_model.gait_time = .4;
    held_model.body_time = .8;
    held_model.gait_hold_duration = node.robot_.period;
    held_model.hold_contacts = std::array<bool, 4>{true, false, false, true};
    held_model.contact_estimated = held_model.hold_contacts;
    Settings model_settings = node.settings_;
    model_settings.time_limit = 3;
    const auto held_result = Solve(node.robot_, held_model, model_settings);
    Check(held_result.success, "real-model two-support hold solve failed");
    std::cout << "PASS seed preservation, nonblocking state, immediate latest-state replanning without duplicate solves, "
                 "epoch/pause/command/clock publication guards, latched stance support, "
                 "command ramp, touchdown stop, failed-seed rejection, "
                 "epoch reset, and measured pre-liftoff origins\n";
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    TerrainPlannerRetryTest::Run();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
