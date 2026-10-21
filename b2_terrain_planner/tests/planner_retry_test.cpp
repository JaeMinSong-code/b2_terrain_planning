/**
 * @file planner_retry_test.cpp
 * @brief 지연·실패를 주입해 비동기 재계획과 성공해 보존 및 발행 차단을 검증한다.
 * @version 0.0.3
 * @update 2026-10-21: 착지 대기 중 몸통 시간 진행과 유한 재계획·보행 재개 연속성을 반영함.
 * @update 2026-10-19: 명령 경계와 실패 반복값의 별도 보존·발행 금지·재시도를 검증함.
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
    state.contact_estimated = state.contact_confirmed;
    auto hold = Install(node);
    update(.4);
    node.OnTimer();
    Started(hold);
    Check(hold->hold_duration == node.robot_.period && hold->body_time == .4 &&
          hold->hold_contacts == state.contact_confirmed, "touchdown wait blocked rolling body solve");
    Finish(node, hold);
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto release = Install(node);
    state.contact_confirmed.fill(true);
    state.contact_estimated.fill(true);
    state.body_time = .42;
    update(.4);
    Started(release);
    Check(release->hold_duration > 0 && release->hold_duration < .8, "resume prefix missing");
    const double resume_time = release->body_time + release->hold_duration;
    Finish(node, release);
    state.active_plan_stamp_ns = node.last_published_stamp_ns_;
    auto refresh = Install(node);
    state.body_time = .44;
    update(.4);
    node.OnTimer();
    Started(refresh);
    Check(std::abs(refresh->body_time + refresh->hold_duration - resume_time) < 1e-12,
          "rolling replanning postponed gait resume forever");
    Finish(node, refresh);
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
                 "epoch/pause/command/clock publication guards\n";
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
