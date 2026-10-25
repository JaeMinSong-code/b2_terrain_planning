/**
 * @file touchdown_stop_test.cpp
 * @brief B2 모델의 착지 대기 감속·정지 유지와 재계획 연결을 검증한다.
 * @version 0.0.1
 * @update 2026-10-25: 이동 상태의 정지·도달 여유·고정 앞부분 인계 회귀를 추가함.
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

void CheckStopped(const Result& plan, double stop, const Robot& robot,
                  const Input& input) {
  Check(plan.success, "stop solve failed: " + plan.status);
  Check(plan.kinematics_valid.value_or(false), "stop failed actual joint IK");
  for (double time = stop; time <= robot.period + 1e-9; time += .01) {
    Check(Evaluate(plan, time, 1).norm() < 1e-4, "body continued after stop deadline");
    Check(Evaluate(plan, time, 2).norm() < 1e-4, "nonzero acceleration after stop");
    Check((Evaluate(plan, time) - Evaluate(plan, stop)).norm() < 1e-5,
          "stopped pose moved");
  }
  for (int sample = 0; sample <= 160; ++sample) {
    const double time = robot.period * sample / 160.0;
    const Vector6 pose = Evaluate(plan, time);
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitX())).toRotationMatrix();
    for (int leg = 0; leg < 4; ++leg) {
      if (!(*input.hold_contacts)[leg]) continue;
      const auto hip = pose.head<3>() + rotation * robot.hips.row(leg).transpose();
      Check((hip - input.feet.row(leg).transpose()).norm() <=
                robot.reach_max[leg] - input.stop_reach_margin + 1e-5,
            "stop consumed reserved leg reach");
    }
  }
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
  input.touchdown_stop_time = .6;
  input.velocity.x() = .5;
  input.velocity[3] = .5;
  input.command << .5, 0, .5;  // 정지 계약은 보관된 사용자 명령보다 우선한다.
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
  settings.leg_vertical_weight = 30;
  settings.vz_weight = 10;
  const auto stop = Solve(robot, input, settings);
  CheckStopped(stop, .6, robot, input);
  std::cout << "four supports stop: " << stop.elapsed << " s\n";
  Check((Evaluate(stop, 0) - input.position).norm() < 1e-5 &&
            (Evaluate(stop, 0, 1) - input.velocity).norm() < 1e-5,
        "stop jumped the initial pose/velocity");

  for (const auto& contacts : {std::array<bool, 4>{true, true, false, true},
                               std::array<bool, 4>{true, false, false, true}}) {
    Input waiting = input;
    waiting.hold_contacts = contacts;
    waiting.contact_estimated = contacts;
    waiting.touchdown_stop_time = robot.period;
    if (!contacts[1]) {
      // 대각 2지지는 지지선 방향으로 움직이는 입력의 감속을 검증한다.
      waiting.position.head<2>() = .5 * (input.feet.row(0) + input.feet.row(3)).head<2>().transpose();
      waiting.velocity.setZero();
      waiting.velocity.head<2>() = .2 * (input.feet.row(0) - input.feet.row(3)).head<2>().normalized().transpose();
    }
    const auto limited_support = Solve(robot, waiting, settings);
    CheckStopped(limited_support, robot.period, robot, waiting);
    std::cout << "landing wait stop: " << limited_support.elapsed << " s\n";
  }

  Input shifted = input;
  shifted.body_time = 1.1;
  shifted.touchdown_stop_time = .5;
  shifted.reference_prefix = std::make_shared<Result>(stop);
  shifted.prefix_duration = .14;
  shifted.position = Evaluate(stop, .1);
  shifted.velocity = Evaluate(stop, .1, 1);
  const auto next = Solve(robot, shifted, settings, &stop);
  CheckStopped(next, .5, robot, shifted);
  for (double time : {0.0, .07, .14}) {
    Check((Evaluate(next, time) - Evaluate(stop, time + .1)).norm() < 1e-5 &&
              (Evaluate(next, time, 1) - Evaluate(stop, time + .1, 1)).norm() < 1e-5,
          "stop replan broke the immutable prefix");
  }
  // 이미 정지한 뒤에도 미착지이면 새 0.8초 정지 시한을 만들지 않고 정지 자세를 유지한다.
  Input held = shifted;
  held.body_time = 1.62;
  held.touchdown_stop_time = 0;
  held.position = Evaluate(stop, .62);
  held.velocity = Evaluate(stop, .62, 1);
  const auto stationary = Solve(robot, held, settings, &stop);
  CheckStopped(stationary, 0, robot, held);

  Input impossible = input;
  impossible.stop_reach_margin = robot.reach_max.minCoeff() * .8;
  const auto rejected = Solve(robot, impossible, settings);
  Check(!rejected.success, "impossible stop was published as feasible");
}
}  // namespace

int main() {
  try {
    Run();
    std::cout << "PASS touchdown stop, reserved reach, prefix continuity, stationary hold\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
