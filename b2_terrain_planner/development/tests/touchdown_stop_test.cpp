/**
 * @file touchdown_stop_test.cpp
 * @brief B2 모델의 착지 대기 목표 속도 감속과 기존 궤적 재사용·연결을 검증한다.
 * @version 0.0.3
 * @update 2026-10-29: midstance nominal 비용 설정으로 착지 대기 감속 회귀를 실행함.
 * @update 2026-10-25: 추가 정지 제약 없이 목표 속도 0으로 감속하는 기존 SQP 경로를 검증함.
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
  Check(!slowed.terrain_validation && !slowed.kinematics_valid && slowed.ik_checks == 0,
        "removed final checkers still ran");
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
}  // namespace

int main() {
  try {
    Run();
    std::cout << "PASS touchdown zero-command deceleration, warm start, prefix continuity\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
