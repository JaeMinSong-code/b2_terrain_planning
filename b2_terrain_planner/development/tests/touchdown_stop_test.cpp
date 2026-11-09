/**
 * @file touchdown_stop_test.cpp
 * @brief B2 모델의 착지 대기 목표 속도 감속과 기존 궤적 재사용·연결을 검증한다.
 * @version 0.0.6
 * @update 2026-11-09: 순수 선회 명령의 yaw-lead nominal 회전(반경 보존·리드 각 범위)을 검증함.
 * @update 2026-11-09: nominal capture 보정(이동평균−명령, 이득·상한)을 직접 검증함.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace b2_terrain_planner;

void Check(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}

void Run() {
  auto model = std::make_shared<RobotModel>(B2_BUILD_MODEL_PATH);
  ModelState state;
  const double pi = std::acos(-1.0);
  for (int leg = 0; leg < 4; ++leg) {
    state.joints.segment<3>(3 * leg) << 0, 44.30 * pi / 180, -85.94 * pi / 180;
  }
  state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
  Robot robot;
  Input input;
  model->PopulatePlanner(state, &robot, &input);
  input.gait_time = .4;
  input.body_time = 1.0;
  input.gait_hold_duration = robot.period;
  input.hold_contacts = std::array<bool, 4>{true, true, true, true};
  input.contact_estimated = input.hold_contacts;
  input.velocity.x() = .5;
  input.velocity[3] = .5;
  input.command << .5, 0, .5;
  Settings settings;
  settings.time_limit = .1;
  settings.sample_intervals = 6;
  settings.realtime_iterations = 2;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  settings.block_assembly = true;
  settings.polynomial_tail = true;
  settings.qp_working_set = true;
  settings.qp_reuse = QpReuseMode::kWarmStart;
  settings.equality_rank_tolerance = 1e-10;
  settings.projection_residual_tolerance = 5e-7;
  settings.qp_regularization = .3;
  settings.qp_eps_abs = 1e-3;
  settings.qp_eps_rel = 1e-3;
  settings.qp_max_iterations = 4000;
  settings.orientation_limit = pi / 3;
  settings.pitch_weight = 10;
  settings.nominal_kinematics_weight = 7;
  settings.vz_weight = 10;
  const auto moving = Solve(robot, input, settings);
  Check(moving.success, "moving reference failed: " + moving.status);

  Input waiting = input;
  waiting.body_time = 1.1;
  waiting.reference_prefix = std::make_shared<Result>(moving);
  waiting.prefix_duration = .14;
  waiting.position = Evaluate(moving, .1);
  waiting.velocity = Evaluate(moving, .1, 1);
  waiting.command.setZero();
  const auto slowed = Solve(robot, waiting, settings, &moving);
  Check(slowed.success, "zero-command hold failed: " + slowed.status);
  Check(slowed.warm_start.used, "landing wait discarded the previous plan seed");
  Check(slowed.profile.final_validation_seconds == 0 && slowed.profile.ik_seconds == 0,
        "removed post-processing still ran");
  for (double time : {0.0, .07, .14}) {
    Check((Evaluate(slowed, time) - Evaluate(moving, time + .1)).norm() < 1e-5 &&
              (Evaluate(slowed, time, 1) - Evaluate(moving, time + .1, 1)).norm() < 1e-5,
          "zero-command replan broke the immutable prefix");
  }
  Input continued = waiting;
  continued.command = input.command;
  const auto cruising = Solve(robot, continued, settings, &moving);
  Check(cruising.success, "comparison plan failed: " + cruising.status);
  Check(Evaluate(slowed, robot.period, 1).head<2>().norm() <
            Evaluate(cruising, robot.period, 1).head<2>().norm(),
        "zero command did not reduce the planned terminal horizontal speed");
  std::cout << "zero-command hold: " << slowed.elapsed << " s, "
            << slowed.iterations << " iterations\n";
}

void CheckCaptureNominal() {
  Robot robot;
  Input input;
  input.position.z() = robot.height;
  input.feet = robot.hips;
  input.command << 0.2, 0.0, 0.0;
  input.gait_time = 0.0;
  const auto phases = MakeHorizon(input.gait_time, robot.period, robot.duty);
  const auto base = NominalFootholds(robot, input, phases, 1.0, 0.12);
  // 이동평균이 명령과 같으면 보정 없음(기존 nominal 보존).
  input.average_velocity = Eigen::Vector2d(0.2, 0.0);
  const auto same = NominalFootholds(robot, input, phases, 1.0, 0.12);
  Check(base.isApprox(same, 1e-12), "capture shifted nominal with zero velocity error");
  // 측정이 명령보다 +Y로 흐르면 발도 +Y로 √(h/g)배 이동한다.
  input.average_velocity = Eigen::Vector2d(0.2, 0.3);
  const auto shifted = NominalFootholds(robot, input, phases, 1.0, 0.12);
  const double capture_time = std::sqrt(robot.height / 9.81);
  const Eigen::Vector2d expected(0.0, capture_time * 0.3);
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector2d delta = (shifted.row(leg) - base.row(leg)).transpose();
    Check((delta - expected).norm() < 1e-12, "capture shift mismatch");
  }
  // 큰 속도 오차는 norm 상한에서 포화한다(과보정 왕복 발진 차단).
  input.average_velocity = Eigen::Vector2d(0.2, 5.0);
  const auto capped = NominalFootholds(robot, input, phases, 1.0, 0.12);
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector2d delta = (capped.row(leg) - base.row(leg)).transpose();
    Check(std::abs(delta.norm() - 0.12) < 1e-12, "capture shift not capped at limit");
  }
  // 이득 0이면 이동평균이 있어도 기존 nominal 그대로다.
  const auto disabled = NominalFootholds(robot, input, phases, 0.0, 0.12);
  Check(base.isApprox(disabled, 1e-12), "capture active despite zero gain");
  std::cout << "PASS capture nominal shift, cap, and zero-gain preservation\n";

  // Yaw-lead: 순수 선회 명령은 발을 CoM 기준 같은 반경에서 회전 방향으로
  // 리드시킨다. 리드 각은 yaw_rate × (touchdown + 0.5·stance) 범위 안이다.
  Input turning = input;
  turning.average_velocity.reset();
  turning.command << 0.0, 0.0, 0.5;
  Input straight = turning;
  straight.command.z() = 0.0;
  const auto led = NominalFootholds(robot, turning, phases, 1.0, 0.18);
  const auto still = NominalFootholds(robot, straight, phases, 1.0, 0.18);
  const Eigen::Vector2d com = turning.position.head<2>();
  const double max_lead = 0.5 * (robot.period + .5 * robot.period * robot.duty);
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector2d a = still.row(leg).transpose() - com;
    const Eigen::Vector2d b = led.row(leg).transpose() - com;
    Check(std::abs(a.norm() - b.norm()) < 1e-9, "yaw lead changed the hip radius");
    const double cross = a.x() * b.y() - a.y() * b.x();
    const double angle = std::atan2(cross, a.dot(b));
    Check(angle > 0.05 && angle < max_lead + 1e-9,
          "yaw lead angle out of range: " + std::to_string(angle));
  }
  std::cout << "PASS yaw-lead rotation of nominal footholds\n";
}
}  // namespace

int main() {
  try {
    Run();
    std::cout << "PASS touchdown zero-command deceleration, warm start, prefix continuity\n";
    CheckCaptureNominal();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
