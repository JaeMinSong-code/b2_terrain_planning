/**
 * @file planner_retry_test.cpp
 * @brief 지연·실패를 주입해 비동기 재계획과 성공해 보존 및 발행 차단을 검증한다.
 * @version 0.0.9
 * @update 2026-10-23: 보행 중 길이 여유 감속·재개 후 감속 유지·여유 회복 해제를 검증함.
 * @update 2026-10-23: 이륙 전 발 이동 반영과 이륙 후 스윙 곡선·착지점 고정을 검증함.
 */
#define B2_TERRAIN_PLANNER_NO_MAIN
#include "../src/terrain_planner_node.cpp"

#include <atomic>
#include <iostream>
#include <thread>

struct TerrainPlannerRetryTest {
  struct Attempt {
    std::atomic<bool> started{false};
    bool had_previous = false;
    bool had_retry_seed = false;
    bool export_seed = false;
    double gait_time = -1;
    double body_time = -1;
    double hold_duration = -1;
    std::optional<std::array<bool, 4>> hold_contacts;
    std::optional<std::array<bool, 4>> contact_estimated;
    Eigen::Vector3d command = Eigen::Vector3d::Zero();
    std::promise<void> release;
    bool success = true;
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
    node.solver_ = [attempt](const Robot& robot, const Input& input, const Settings&,
                            const Result* previous) {
      attempt->had_previous = previous != nullptr && previous->success;
      attempt->had_retry_seed = previous != nullptr && !previous->success && previous->seed_available;
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
      result.seed_available = attempt->export_seed;
      result.status = "time_limit";
      if (result.success) result.status = "test_success";
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

  static void CheckKinematicBraking(TerrainPlannerNode& node) {
    node.state_.reset();
    node.previous_.reset();
    node.retry_seed_.reset();
    node.published_plans_.clear();
    node.kinematic_braking_ = false;
    node.ResetBrakingCommand();
    const Eigen::Vector3d requested(.30, -.10, .40);
    node.command_ = requested;
    node.applied_command_ = requested;
    node.command_slot_ = 1;
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.gait_clock_valid = true;
    state.gait_epoch_ns = 4;
    state.gait_time = .4;
    state.body_clock_valid = true;
    state.body_time = .5;
    state.gait_paused = false;
    state.contact_confirmed = {true, true, false, true};
    state.contact_estimated.fill(false);
    // 회전·이동한 odom에서도 몸통 축 상대 기구학 여유가 같아야 한다.
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d base(1.2, -.7, .55);
    state.base_orientation.w = orientation.w();
    state.base_orientation.x = orientation.x();
    state.base_orientation.y = orientation.y();
    state.base_orientation.z = orientation.z();
    state.base_position.x = base.x();
    state.base_position.y = base.y();
    state.base_position.z = base.z();
    const Eigen::Vector3d com = base + orientation * node.robot_.com_offset;
    state.com_position.x = com.x();
    state.com_position.y = com.y();
    state.com_position.z = com.z();
    const auto set_margin = [&](int leg, double margin) {
      const Eigen::Vector3d offset = node.robot_.hips.row(leg).transpose() + node.robot_.com_offset;
      const Eigen::Vector3d direction = Eigen::Vector3d(-.6, .2, -.77).normalized();
      const Eigen::Vector3d foot = base + orientation *
          (offset + direction * (node.robot_.reach_max[leg] - margin));
      state.foot_position[leg].x = foot.x();
      state.foot_position[leg].y = foot.y();
      state.foot_position[leg].z = foot.z();
    };
    const auto update = [&] {
      state.body_time += .002;
      state.header.stamp = node.now();
      node.OnState(state);
    };
    for (int leg = 0; leg < 4; ++leg) set_margin(leg, .20);
    set_margin(2, -.20);  // 비지지발의 한계 초과는 감속을 유발하지 않는다.
    auto safe = Install(node);
    update();
    node.OnTimer();
    Started(safe);
    Check(!node.kinematic_braking_ && (safe->command - requested).norm() < 1e-12,
          "safe walking or nonstance foot suppressed requested command");
    Check(std::abs(node.stance_reach_margin_ - .20) < 1e-12,
          "rotated hip-to-foot distance or CoM offset was incorrect");
    Finish(node, safe);

    auto before_brake = Install(node);
    update();
    node.OnTimer();
    Started(before_brake);
    const auto published_before = node.last_published_stamp_ns_;
    set_margin(0, .08);
    update();  // 계산 중 한계 접근은 주기 타이머를 기다리지 않고 재계획을 요청한다.
    auto brake = Install(node);
    Finish(node, before_brake);
    Started(brake);
    Check(node.last_published_stamp_ns_ == published_before &&
          (brake->command - requested).norm() < 1e-12,
          "pre-braking snapshot published or braking entry jumped the command");
    Check(node.kinematic_braking_ && node.stance_reach_leg_ == 0 &&
          (node.applied_command_ - requested).norm() < 1e-12 &&
          (node.command_ - requested).norm() < 1e-12,
          "walking failed to brake from confirmed stance reach or overwrote the operator command");
    Finish(node, brake);

    auto band = Install(node);
    state.gait_paused = true;
    set_margin(0, .12);
    update();
    node.OnTimer();
    Started(band);
    Check(node.kinematic_braking_ && band->command.head<2>().norm() < requested.head<2>().norm() &&
          std::abs((band->command - requested).head<2>().norm() - .002) < 1e-12 &&
          std::abs(band->command.z() - requested.z() + .002) < 1e-12,
          "braking failed to ramp on advancing body time during gait pause");
    Finish(node, band);
    auto release = Install(node);
    set_margin(0, .16);
    update();
    Started(release);
    Check(!node.kinematic_braking_ && (release->command - band->command).norm() < 1e-12,
          "recovered reach jumped to the saved command");
    Finish(node, release);

    auto reenter = Install(node);
    set_margin(0, .08);
    update();
    Started(reenter);
    Check((reenter->command - release->command).norm() < 1e-12,
          "braking reentry reset the command");
    Finish(node, reenter);
    // 보행 시각은 정지한 채 몸통 시각만 진행해 실제 감속 목표가 0에 도달하는지 확인한다.
    for (int i = 0; i < 25; ++i) {
      state.body_time += .018;
      update();
    }
    Check(node.filtered_command_.isZero(1e-12), "sustained braking did not reach zero");
    auto walking = Install(node);
    state.gait_paused = false;
    update();
    Started(walking);
    Check(node.kinematic_braking_ && walking->command.isZero(1e-12),
          "gait resume released braking while stance reach was still unsafe");
    Finish(node, walking);
    auto reach_recovered = Install(node);
    set_margin(0, .16);
    update();
    Started(reach_recovered);
    Check(!node.kinematic_braking_ && reach_recovered->command.isZero(1e-12),
          "walking reach recovery failed to release braking continuously");
    Finish(node, reach_recovered);
    const double resume_time = state.body_time;
    auto recovering = Install(node);
    state.body_time += .048;
    update();
    node.OnTimer();
    Started(recovering);
    Check(std::abs(recovering->command.head<2>().norm() - .05) < 1e-12 &&
          std::abs(recovering->command.z() - .05) < 1e-12,
          "recovery exceeded linear/yaw command rates");
    const Eigen::Vector3d snapshot = node.filtered_command_;
    node.OnTimer();
    node.OnTimer();
    Check((snapshot - node.filtered_command_).norm() < 1e-12,
          "same-state retries advanced recovery");
    // 목표가 다음 상태에서 이동해도 계산 중 후보를 계속 폐기하지 않는다.
    update();
    const auto before_recovery = node.last_published_stamp_ns_;
    Finish(node, recovering);
    Check(node.last_published_stamp_ns_ > before_recovery,
          "continuous command ramp discarded an otherwise valid solve");
    for (int i = 0; i < 25; ++i) {
      state.body_time += .018;
      update();
    }
    Check(state.body_time > resume_time + .5 &&
          (node.filtered_command_ - requested).norm() < 1e-12 &&
          !node.braking_command_filter_active_, "recovery failed to reach the saved command");
    // 새 세션에서는 이전 감속의 시간과 목표를 상속하지 않는다.
    node.filtered_command_.setZero();
    node.braking_command_filter_active_ = true;
    auto reset = Install(node);
    ++state.gait_epoch_ns;
    for (int leg = 0; leg < 4; ++leg) set_margin(leg, .20);
    state.contact_confirmed.fill(true);
    state.body_time = 0;
    update();
    Started(reset);
    Check((reset->command - requested).norm() < 1e-12 &&
          !node.braking_command_filter_active_, "new epoch retained old braking recovery");
    Finish(node, reset);
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
    const auto update = [&](double gait_time) {
      state.gait_time = gait_time;
      plan.seed_context.gait_time = gait_time;
      plan.phases = MakeHorizon(gait_time, node.robot_.period, node.robot_.duty);
      node.UpdateSwingReferences(plan, measured, state);
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
    const auto committed = node.swing_references_[1];
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
  }

  static void Run() {
    TerrainPlannerNode node;
    node.timer_->cancel();
    node.completion_timer_->cancel();
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

    auto first = Install(node);
    update(0);
    node.OnTimer();
    Started(first);
    Check(!first->had_previous, "first solve unexpectedly had a seed");
    Finish(node, first);
    Check(node.previous_.has_value() && node.last_publish_ns_ > 0, "initial plan not published");

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
    auto same_retry = Install(node, false);
    const auto before_same = node.attempt_seq_;
    Finish(node, same);
    Started(same_retry);
    Check(same_retry->had_previous, "failure erased successful seed");
    Finish(node, same_retry);
    node.OnTimer();
    Check(!node.solve_future_.valid() && node.attempt_seq_ == before_same + 1,
          "identical failed input retried without bound");
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
    Check(node.applied_command_.isZero(), "command changed within the same step");
    state.gait_paused = false;
    auto next_command = Install(node);
    update(.4);
    node.OnTimer();
    Started(next_command);
    Check(std::abs(next_command->command.x() - .1) < 1e-12,
          "next confirmed stance did not latch pending command");
    node.command_.x() = -.2;
    Finish(node, next_command);
    Check(std::abs(node.applied_command_.x() - .1) < 1e-12,
          "command changed during solve");

    auto partial = Install(node, false);
    partial->export_seed = true;
    update(.405);
    node.OnTimer();
    Started(partial);
    const auto before_partial = node.last_published_stamp_ns_;
    const auto successful_seed_gait = node.previous_->seed_context.gait_time;
    auto continued = Install(node);
    Finish(node, partial);
    Started(continued);
    Check(continued->had_retry_seed && !continued->had_previous,
          "partial iterate was not used for the bounded retry");
    Check(node.last_published_stamp_ns_ == before_partial &&
          node.previous_->seed_context.gait_time == successful_seed_gait,
          "failed iterate overwrote or published a successful plan");
    Finish(node, continued);
    Check(!node.retry_seed_, "successful publication retained obsolete retry seed");

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
    Finish(node, hold);
    Check(hold->hold_duration == node.robot_.period && hold->body_time == .4 &&
          hold->hold_contacts == state.contact_confirmed, "touchdown wait blocked rolling body solve");
    Check(hold->contact_estimated == hold_estimated, "stance overwrote measured terrain contacts");
    Check(node.last_published_stamp_ns_ > before_hold,
          "estimated contact change rejected two-stance hold plan");
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto tripod = Install(node);
    state.contact_confirmed[1] = true;
    state.contact_estimated.fill(false);
    state.body_time = .42;
    update(.4);
    Started(tripod);
    const auto before_tripod = node.last_published_stamp_ns_;
    Finish(node, tripod);
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
    Finish(node, release);
    Check(release->hold_duration > 0 && release->hold_duration < .8, "resume prefix missing");
    Check(node.last_published_stamp_ns_ > before_release,
          "estimated contact change rejected stance resume plan");
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
    CheckKinematicBraking(node);
    CheckSwingReferenceUpdates(node);

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
    Check(held_result.kinematics_valid.value_or(false) && held_result.ik_checks == 50,
          "hold checked fictitious future touchdown or skipped real supports");
    std::cout << "PASS seed preservation, nonblocking state, immediate/bounded retries, "
                 "epoch/pause/command/clock publication guards, latched stance support, "
                 "reach-gated braking, rate-limited command recovery, epoch reset, and measured pre-liftoff origins\n";
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
