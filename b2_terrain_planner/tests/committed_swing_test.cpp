/**
 * @file committed_swing_test.cpp
 * @brief 명령·지도 변경 시 진행 중 착지점의 최적화 고정과 착지 후 해제를 검증한다.
 * @version 0.0.3
 * @update 2026-10-21: 착지 대기 중 몸통 시간 진행과 유한 재계획·보행 재개 연속성을 반영함.
 * @update 2026-10-19: 연속 몸통 연결·구간 이동과 중단 반복값의 전체 재검증을 확인함.
 */
#include "b2_terrain_planner/committed_swing.h"
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool value, const std::string& why) {
  if (!value) throw std::runtime_error(why);
}
}

int main() {
  using namespace b2_terrain_planner;
  try {
    Robot robot;
    Settings settings;
    settings.time_limit = 3.0;
    settings.sample_intervals = 4;
    settings.structured_equalities = true;
    settings.polynomial_tail = true;
    settings.lapack_svd = true;
    Input initial;
    initial.position.z() = robot.height;
    initial.feet = robot.hips;
    initial.command.x() = .1;
    HeightMap map;
    map.origin = Eigen::Vector2d::Constant(-1.975);
    map.heights = Eigen::MatrixXd::Zero(80, 80);
    map.valid = Eigen::ArrayXXi::Ones(80, 80);
    map.variance = Eigen::MatrixXd::Zero(80, 80);
    initial.terrain = map;
    const auto first = Solve(robot, initial, settings);
    Check(first.success, "initial: " + first.status);
    std::array<SwingReference, 4> references;
    for (int leg : {1, 2}) {
      auto& ref = references[leg];
      ref.start = .08; ref.end = .4; ref.apex_time = .24; ref.apex_z = .08;
      ref.position = initial.feet.row(leg).transpose();
      ref.target = first.footholds.row(leg).transpose();
    }
    Input input = initial;
    input.gait_time = .2;
    input.position = Evaluate(first, .2);
    input.velocity = Evaluate(first, .2, 1);
    for (int leg : {1, 2}) input.feet.row(leg) = references[leg].Sample(.2).first.transpose();
    Check(CommitActiveSwingFootholds(robot, references, &input), "active reference missing");
    Check(input.committed_footholds[1] && input.committed_footholds[2] &&
          !input.committed_footholds[0] && !input.committed_footholds[3], "wrong committed legs");
    const auto baseline = Solve(robot, input, settings, &first);
    Check(baseline.success, "baseline: " + baseline.status);
    input.command.x() = -.15;
    input.command.y() = .03;
    input.terrain->origin += Eigen::Vector2d(.05, -.05);
    const auto changed = Solve(robot, input, settings, &first);
    Check(changed.success, "command change: " + changed.status);
    for (int leg : {1, 2}) {
      Check((changed.footholds.row(leg).transpose() - references[leg].target).norm() < 1e-7,
            "active touchdown changed");
    }
    Check((changed.footholds.row(0) - baseline.footholds.row(0)).norm() > 1e-4 ||
          (changed.footholds.row(3) - baseline.footholds.row(3)).norm() > 1e-4,
          "future swing ignored new command");
    Input handover = input;
    handover.reference_prefix = std::make_shared<Result>(first);
    handover.prefix_duration = .14;
    handover.position.x() += .001;  // 측정 상태와 실행 기준점이 달라도 고정된 앞부분은 유지한다.
    const auto joined = Solve(robot, handover, settings, &first);
    Check(joined.success, "handover: " + joined.status);
    Check((Evaluate(joined, 0) - Evaluate(first, .2)).norm() < 2e-5,
          "committed reference prefix changed");
    for (int derivative = 0; derivative < 2; ++derivative) {
      Check((Evaluate(joined, .14, derivative) - Evaluate(first, .34, derivative)).norm() < 2e-5,
            "body handover is discontinuous");
    }
    const auto stitched = SpliceBody(first, .2, joined, .14);
    for (int derivative = 0; derivative < 3; ++derivative) {
      Check((Evaluate(stitched, .03, derivative) - Evaluate(first, .23, derivative)).norm() < 1e-9,
            "prefix changed while slicing");
      Check((Evaluate(stitched, .3, derivative) - Evaluate(joined, .3, derivative)).norm() < 1e-9,
            "optimized tail changed while slicing");
    }
    handover.reference_prefix = std::make_shared<Result>(stitched);
    const auto reused = Solve(robot, handover, settings, &joined);
    Check(reused.success && reused.warm_start.used, "split-phase warm start: " + reused.status);
    handover.gait_time = .26;
    handover.position = Evaluate(stitched, .06);
    handover.velocity = Evaluate(stitched, .06, 1);
    for (int leg : {1, 2}) handover.feet.row(leg) = references[leg].Sample(.26).first.transpose();
    const auto shifted_join = Solve(robot, handover, settings, &stitched);
    Check(shifted_join.success && shifted_join.warm_start.used,
          "crossing a split in warm start: " + shifted_join.status);
    Check(shifted_join.warm_start.polynomial_extended_phases == 0,
          "short terminal polynomial was extrapolated beyond its horizon");
    handover.gait_time = .478;
    handover.prefix_duration = 0;
    handover.position = Evaluate(stitched, .278);
    handover.velocity = Evaluate(stitched, .278, 1);
    handover.position.y() += .003;
    handover.feet = stitched.footholds;
    handover.committed_footholds = {};
    const auto paused_join = Solve(robot, handover, settings, &stitched);
    Check(paused_join.success, "paused handover: " + paused_join.status);
    for (int derivative = 0; derivative < 2; ++derivative) {
      Check((Evaluate(paused_join, 0, derivative) - Evaluate(stitched, .278, derivative)).norm() < 2e-5,
            "paused handover changed current reference");
    }
    // 정지한 발 위상과 계속 진행한 몸통 시간을 독립적으로 연결한다.
    Input rolling = initial;
    rolling.gait_time = .4;
    rolling.body_time = .48;
    rolling.gait_hold_duration = robot.period;
    rolling.hold_contacts = std::array<bool, 4>{true, true, true, true};
    rolling.feet = first.footholds;
    rolling.reference_prefix = std::make_shared<Result>(first);
    rolling.prefix_duration = .14;
    const auto held = Solve(robot, rolling, settings, &first);
    Check(held.success, "rolling hold: " + held.status);
    const auto held_body = SpliceBody(first, .48, held, .14);
    for (int derivative = 0; derivative < 3; ++derivative) {
      Check((Evaluate(held_body, .05, derivative) - Evaluate(first, .53, derivative)).norm() < 1e-8,
            "body clock did not advance during gait hold");
    }
    rolling.body_time = .54;
    rolling.gait_hold_duration = .10;
    rolling.reference_prefix = std::make_shared<Result>(held_body);
    const auto restart = Solve(robot, rolling, settings, &held_body);
    Check(restart.success, "rolling resume: " + restart.status);
    for (int derivative = 0; derivative < 2; ++derivative) {
      Check((Evaluate(restart, .14, derivative) - Evaluate(held_body, .20, derivative)).norm() < 2e-5,
            "resumed body p/v did not match advancing reference");
    }
    const std::array<bool, 4> two_supports{true, false, false, true};
    const auto waiting_horizon = MakeHorizon(.4, .8, .6, .8, two_supports);
    Check(waiting_horizon.size() == 1 && waiting_horizon[0].duration == .8 &&
          waiting_horizon[0].contacts == two_supports, "assumed unconfirmed touchdown");
    const auto resumed_horizon = MakeHorizon(.4, .8, .6, .14, rolling.hold_contacts);
    Check(resumed_horizon.front().duration == .14 &&
          std::abs(resumed_horizon.back().start + resumed_horizon.back().duration - .8) < 1e-12,
          "rolling horizon grew beyond one gait cycle");

    Settings interrupted_settings = settings;
    interrupted_settings.time_limit = 1e-6;
    const auto interrupted = Solve(robot, handover, interrupted_settings, &stitched);
    Check(!interrupted.success && interrupted.seed_available && !interrupted.coefficients.empty(),
          "interrupted iterate was not retained separately from success");
    const auto continued = Solve(robot, handover, settings, &interrupted);
    Check(continued.success && continued.warm_start.used,
          "failed iterate could not resume with full validation: " + continued.status);
    input.gait_time = .078;
    Check(CommitActiveSwingFootholds(robot, references, &input), "pre-liftoff");
    Check(!input.committed_footholds[1], "locked before liftoff");
    input.gait_time = .08;
    Check(CommitActiveSwingFootholds(robot, references, &input) && input.committed_footholds[1],
          "liftoff boundary not locked");
    input.gait_time = .4;
    Check(CommitActiveSwingFootholds(robot, references, &input), "touchdown boundary");
    for (const auto& target : input.committed_footholds) Check(!target, "lock not released");
    input.gait_time = .2;
    references[1].end = 1.2;
    Check(!CommitActiveSwingFootholds(robot, references, &input), "stale swing accepted");
    std::cout << "PASS committed XYZ, changed future targets, liftoff/release and stale reference\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
