/**
 * @file planner_retry_test.cpp
 * @brief 지연·실패를 주입해 비동기 재계획과 성공해 보존 및 발행 차단을 검증한다.
 * @version 0.0.14
 * @update 2026-10-25: 스윙 중 명령 램프·XY 변화율·방향 반전·대기 후 램프 복귀를 검증함.
 * @update 2026-10-25: 착지 대기 속도 목표 0과 접촉 확인 후 명령 복귀를 검증함.
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
      result.violation = attempt->violation;
      // 실제 Solve의 warm start 사유 규약을 흉내 내 시드 체인 판정을 검증한다.
      if (attempt->had_retry_seed) result.warm_start.reason = "shifted_retry_iterate";
      else if (attempt->had_previous) result.warm_start.reason = "shifted_previous_plan";
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
    node.completion_timer_->cancel();
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
          "same-state retry advanced ramp");
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
    state.contact_confirmed.fill(true);
    node.UpdateTouchdownStop(state);
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isZero(), "resume jumped to saved request");
    state.body_time += .02;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.006, -.008, -.02), 1e-10),
          "resume did not ramp from zero");
    state.body_time += 2.0;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(node.command_), "ramp overshot target");
    node.command_topic_ = "/test/command";
    node.command_stamp_.reset();
    state.body_time += .1;
    node.UpdateCommandRamp(state);
    Check(node.PlanningCommand().isApprox(Eigen::Vector3d(-.27, -.36, -.9), 1e-10),
          "expired command did not ramp toward zero");
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
          node.PlanningCommand().isZero(), "confirmed landing jumped to saved command");
  }

  static void CheckRetrySeedDiscard(TerrainPlannerNode& node) {
    node.state_.reset();
    node.previous_.reset();
    node.DiscardRetrySeed();
    node.published_plans_.clear();
    node.command_.setZero();
    node.applied_command_.setZero();
    TerrainPlannerNode::State state;
    state.header.frame_id = "odom";
    state.base_orientation.w = 1;
    state.com_position.x = node.nominal_input_.position.x();
    state.com_position.y = node.nominal_input_.position.y();
    state.com_position.z = node.nominal_input_.position.z();
    state.gait_clock_valid = true;
    state.gait_epoch_ns = 11;
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

    auto baseline = Install(node);
    update(0);
    node.OnTimer();
    Started(baseline);
    Finish(node, baseline);
    Check(node.previous_.has_value(), "retry-seed scenario missing baseline plan");

    // 개선 중인 체인은 유지하다가 3회째 실패에서 소진 폐기한다.
    auto fail1 = Install(node, false);
    fail1->export_seed = true;
    fail1->violation = .10;
    update(.01);
    node.OnTimer();
    Started(fail1);
    auto fail2 = Install(node, false);
    fail2->export_seed = true;
    fail2->violation = .15;  // 2배 미만 증가는 발산이 아니다.
    Finish(node, fail1);
    Started(fail2);
    Check(fail2->had_retry_seed, "first failure did not seed the retry");
    auto fail3 = Install(node, false);
    fail3->export_seed = true;
    fail3->violation = .12;
    update(.02);
    Finish(node, fail2);
    Started(fail3);
    Check(fail3->had_retry_seed, "non-diverging chain was discarded early");
    auto exhausted = Install(node, false);
    exhausted->export_seed = true;
    exhausted->violation = .10;
    Finish(node, fail3);
    Started(exhausted);
    Check(!exhausted->had_retry_seed && exhausted->had_previous,
          "exhausted retry chain did not fall back to the last published plan");
    Finish(node, exhausted);

    // 위반이 2배 넘게 커지면 즉시 폐기한다. exhausted 실패가 새 체인(viol .10)을 시작했다.
    auto diverged = Install(node, false);
    diverged->export_seed = true;
    diverged->violation = .25;
    update(.03);
    Started(diverged);
    Check(diverged->had_retry_seed, "new chain after discard was not seeded");
    auto recovered = Install(node, false);
    recovered->export_seed = true;
    Finish(node, diverged);
    Started(recovered);
    Check(!recovered->had_retry_seed && recovered->had_previous,
          "diverged retry seed was not discarded");
    Finish(node, recovered);
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
    Check(hold->command.isZero(), "hold did not request zero velocity");
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
    CheckCommandRamp();
    CheckTouchdownStop(node);
    CheckRetrySeedDiscard(node);
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
    Check(!held_result.kinematics_valid && held_result.ik_checks == 0,
          "hold still ran the removed final IK checker");
    std::cout << "PASS seed preservation, nonblocking state, immediate/bounded retries, "
                 "epoch/pause/command/clock publication guards, latched stance support, "
                 "command ramp, touchdown stop, retry-seed discard, "
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
