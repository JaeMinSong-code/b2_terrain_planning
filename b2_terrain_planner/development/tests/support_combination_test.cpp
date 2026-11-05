/**
 * @file support_combination_test.cpp
 * @brief 세 지지의 실제 발 조합·미분·몸통 계획과 trot 재개를 검증한다.
 * @version 0.0.6
 * @update 2026-11-05: 2cm 도달거리 여유에서 지지 조합별 계획과 반경 상한을 검증함.
 * @update 2026-11-03: 제거한 IK 검증 결과 필드 검사를 정리함.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"
#include "tamols_kernels.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace b2_terrain_planner;
using Kernel = decltype(&tamols_without_fl);

void Check(bool value, const std::string& reason) {
  if (!value) throw std::runtime_error(reason);
}

struct Sample {
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd residual_jacobian = Eigen::MatrixXd::Zero(12, 31);
  Eigen::VectorXd constraint = Eigen::VectorXd::Zero(22);
  Eigen::MatrixXd constraint_jacobian = Eigen::MatrixXd::Zero(22, 31);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(22);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(22);
};

Sample EvaluateKernel(Kernel kernel, const Eigen::VectorXd& z,
                      const Eigen::VectorXd& parameters, int mode = 0) {
  Sample sample;
  const double* arguments[] = {z.data(), parameters.data()};
  std::array<double*, 6> outputs{sample.residual.data(), sample.residual_jacobian.data(),
      sample.constraint.data(), sample.constraint_jacobian.data(),
      sample.lower.data(), sample.upper.data()};
  if (mode == 1) {
    outputs = {sample.residual.data(), sample.constraint.data(),
               sample.lower.data(), sample.upper.data(), nullptr, nullptr};
  } else if (mode == 2) {
    outputs = {sample.constraint.data(), sample.constraint_jacobian.data(),
               sample.lower.data(), sample.upper.data(), nullptr, nullptr};
  }
  Check(kernel(arguments, outputs.data(), nullptr, nullptr, 0) == 0,
        "three-support kernel evaluation failed");
  return sample;
}

void CheckKernels() {
  const std::array<Kernel, 4> full{tamols_without_fl, tamols_without_fr,
                                 tamols_without_rl, tamols_without_rr};
  const std::array<Kernel, 4> values{tamols_without_fl_values, tamols_without_fr_values,
      tamols_without_rl_values, tamols_without_rr_values};
  const std::array<Kernel, 4> jacobians{tamols_without_fl_constraint_jacobian,
      tamols_without_fr_constraint_jacobian, tamols_without_rl_constraint_jacobian,
      tamols_without_rr_constraint_jacobian};
  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(54);
  parameters[0] = 74;
  Eigen::Map<Eigen::Matrix3d>(parameters.data() + 1).diagonal() << 1.5, 4, 4;
  parameters[22] = .55;
  parameters[23] = .5;
  parameters[24] = .25;
  parameters[25] = .75;
  parameters[42] = std::sqrt(10.0);
  parameters[43] = std::sqrt(30.0);
  parameters[44] = 1;
  Eigen::VectorXd z = Eigen::VectorXd::Zero(31);
  z.head<6>() << .01, -.02, .55, .1, -.02, .03;
  z.segment<6>(6) << .1, .02, .01, .1, .02, -.01;
  z.segment<6>(12) << .2, -.1, .05, .03, -.02, .01;
  z.segment<12>(18) << .32, .19, 0, .32, -.19, 0,
      -.34, .19, 0, -.34, -.19, 0;
  for (int missing = 0; missing < 4; ++missing) {
    const auto baseline = EvaluateKernel(full[missing], z, parameters);
    const auto only_values = EvaluateKernel(values[missing], z, parameters, 1);
    const auto only_jacobian = EvaluateKernel(jacobians[missing], z, parameters, 2);
    Check(baseline.residual.isApprox(only_values.residual, 1e-12) &&
          baseline.constraint.isApprox(only_values.constraint, 1e-12) &&
          baseline.constraint.isApprox(only_jacobian.constraint, 1e-12) &&
          baseline.constraint_jacobian.isApprox(only_jacobian.constraint_jacobian, 1e-12),
          "three-support evaluation paths disagree");
    Eigen::VectorXd shifted = z;
    shifted.segment<3>(18 + 3 * missing) += Eigen::Vector3d(.04, -.03, .05);
    const auto absent = EvaluateKernel(full[missing], shifted, parameters);
    Check(baseline.residual.isApprox(absent.residual, 1e-12) &&
          baseline.constraint.isApprox(absent.constraint, 1e-12),
          "non-supporting foot affected body constraints or costs");
    for (int leg = 0; leg < 4; ++leg) {
      if (leg == missing) continue;
      shifted = z;
      shifted[18 + 3 * leg] += .04;
      const auto moved = EvaluateKernel(full[missing], shifted, parameters);
      Check((baseline.constraint - moved.constraint).norm() > 1e-4,
            "actual supporting foot was ignored");
    }
    for (int axis = 0; axis < 31; ++axis) {
      auto plus = z;
      auto minus = z;
      plus[axis] += 1e-6;
      minus[axis] -= 1e-6;
      const auto high = EvaluateKernel(values[missing], plus, parameters, 1);
      const auto low = EvaluateKernel(values[missing], minus, parameters, 1);
      Check(((high.residual - low.residual) / 2e-6 -
             baseline.residual_jacobian.col(axis)).lpNorm<Eigen::Infinity>() < 1e-5 &&
            ((high.constraint - low.constraint) / 2e-6 -
             baseline.constraint_jacobian.col(axis)).lpNorm<Eigen::Infinity>() < 1e-5,
            "three-support Jacobian disagrees with finite differences");
    }
  }
}

// nominal의 수직 목표는 몸통과 함께 기울이지 않는다. 월드 평행이동에는 불변이다.
void CheckNominalKinematics() {
  Eigen::Matrix<double, 9, 1> state;
  state << .1, -.2, .55, .15, -.06, .08, .4, .1, .02;
  Eigen::Vector4d parameters(.3, .2, -.03, .52);
  Eigen::Vector3d residual;
  Eigen::Matrix<double, 3, 9> jacobian;
  auto evaluate = [&](const Eigen::Matrix<double, 9, 1>& input) {
    Eigen::Vector3d value;
    const double* arguments[] = {input.data(), parameters.data()};
    double* outputs[] = {value.data(), nullptr};
    Check(tamols_nominal_kinematics(arguments, outputs, nullptr, nullptr, 0) == 0,
          "nominal kernel failed");
    return value;
  };
  const double* arguments[] = {state.data(), parameters.data()};
  double* outputs[] = {residual.data(), jacobian.data()};
  tamols_nominal_kinematics(arguments, outputs, nullptr, nullptr, 0);
  for (int axis = 0; axis < 9; ++axis) {
    auto plus = state;
    auto minus = state;
    plus[axis] += 1e-6;
    minus[axis] -= 1e-6;
    Check(((evaluate(plus) - evaluate(minus)) / 2e-6 - jacobian.col(axis)).norm() < 1e-7,
          "nominal Jacobian disagrees with finite differences");
  }
  auto shifted = state;
  shifted.head<3>() += Eigen::Vector3d(.7, -.4, .2);
  shifted.tail<3>() += Eigen::Vector3d(.7, -.4, .2);
  Check(evaluate(shifted).isApprox(residual, 1e-12), "nominal uses an absolute position anchor");
  // hip offset이 0이면 몸통을 기울여도 l_des=[0,0,h]는 월드 수직으로 남는다.
  parameters.head<3>().setZero();
  state.tail<3>() = state.head<3>() - Eigen::Vector3d(0, 0, parameters[3]);
  Check(evaluate(state).norm() < 1e-12, "nominal rotated the desired vertical leg vector");

  const auto phases = MakeHorizon(0, .8, .6);
  const auto fl = StanceMidpoints(phases, 0);
  const auto fr = StanceMidpoints(phases, 1);
  Check(fl.size() == 1 && std::abs(fl[0] - .24) < 1e-10,
        "FL nominal must be sampled once at full midstance");
  Check(fr.size() == 2 && std::abs(fr[0] - .04) < 1e-10 &&
        std::abs(fr[1] - .6) < 1e-10, "clipped stance midpoints are incorrect");
  auto split = phases;
  auto tail = split.front();
  tail.start += .03;
  tail.duration -= .03;
  split.front().duration = .03;
  split.insert(split.begin() + 1, tail);
  Check(StanceMidpoints(split, 0) == fl, "spline split duplicated nominal cost");
  const std::array<bool, 4> contacts{false, true, true, true};
  const auto hold = MakeHorizon(.4, .8, .6, .8, contacts);
  Check(StanceMidpoints(hold, 0).empty(), "non-supporting leg received nominal cost");
  const auto held = StanceMidpoints(hold, 1);
  Check(held.size() == 1 && std::abs(held[0] - .4) < 1e-10,
        "hold must receive one nominal sample per supporting leg");
}

void CheckPlanner() {
  auto model = std::make_shared<RobotModel>(B2_BUILD_MODEL_PATH);
  ModelState state;
  const double pi = std::acos(-1.0);
  for (int leg = 0; leg < 4; ++leg) {
    state.joints.segment<3>(3 * leg) << 0, 44.30 * pi / 180, -85.94 * pi / 180;
  }
  state.base_position.z() = -model->Evaluate(state).feet.col(2).mean();
  Robot robot;
  Input nominal;
  model->PopulatePlanner(state, &robot, &nominal);
  Check(robot.nominal_hips.has_value(), "B2 nominal leg origins missing");
  for (int leg = 0; leg < 4; ++leg) {
    const Eigen::Vector3d origin = robot.nominal_hips->row(leg).transpose();
    const Eigen::Vector3d foot = nominal.feet.row(leg).transpose();
    const Eigen::Vector3d target = nominal.position.head<3>() + origin -
        Eigen::Vector3d(0, 0, robot.height + origin.z());
    // The distal URDF links include lateral offsets below 0.1 mm.
    Check(std::abs(target.y() - foot.y()) < 1e-4,
          "nominal target pulls neutral B2 feet inward");
    Check(std::abs(target.z() - foot.z()) < 1e-8,
          "nominal leg height differs from neutral foot height");
    Check(std::abs(std::abs(origin.y() - robot.hips(leg, 1)) - .11973) < 1e-8,
          "nominal lateral offset missing or reach hip changed");
    Eigen::Matrix<double, 9, 1> kernel_state;
    kernel_state.head<6>() = nominal.position;
    kernel_state.tail<3>() = target;
    Eigen::Vector4d parameters;
    parameters << origin, robot.height + origin.z();
    Eigen::Vector3d residual;
    const double* arguments[] = {kernel_state.data(), parameters.data()};
    double* outputs[] = {residual.data(), nullptr};
    tamols_nominal_kinematics(arguments, outputs, nullptr, nullptr, 0);
    Check(residual.norm() < 1e-10, "neutral leg target has nominal residual");
    kernel_state[7] = nominal.position[1] + robot.hips(leg, 1);
    tamols_nominal_kinematics(arguments, outputs, nullptr, nullptr, 0);
    Check(residual.norm() > .11, "inward foot target is not penalized");
  }
  nominal.gait_time = .4;
  nominal.body_time = .8;
  nominal.gait_hold_duration = robot.period;
  nominal.command.x() = .1;
  Settings settings;
  settings.time_limit = 3;
  settings.max_reach_margin = .02;
  settings.sample_intervals = 4;
  settings.structured_equalities = true;
  settings.lapack_svd = true;
  for (int missing = 0; missing < 4; ++missing) {
    Input input = nominal;
    std::array<bool, 4> contacts{true, true, true, true};
    contacts[missing] = false;
    input.hold_contacts = contacts;
    input.contact_estimated = contacts;
    settings.block_assembly = true;
    const auto result = Solve(robot, input, settings);
    Check(result.success, "B2 three-support solve: " + result.status);
    Check(result.phases.size() == 1 && result.phases.front().contacts == contacts &&
          result.margin_variable_count == 1, "wrong three-support plan");
    const auto equality = std::find_if(result.constraint_groups.begin(), result.constraint_groups.end(),
        [](const auto& group) { return group.name == "giac-equality"; });
    Check(equality != result.constraint_groups.end() && equality->rows == 0,
          "three-support polygon retained a diagonal balance equality");
    Input moved = input;
    moved.feet(missing, 1) += .03;
    const auto independent = Solve(robot, moved, settings);
    Check(independent.success, "non-supporting foot move: " + independent.status);
    settings.block_assembly = false;
    const auto dense = Solve(robot, input, settings);
    Check(dense.success, "dense three-support solve: " + dense.status);
    for (double time : {0.0, .2, .4, .6, .8}) {
      const Vector6 pose = Evaluate(result, time);
      const Eigen::Matrix3d rotation =
          (Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitX())).toRotationMatrix();
      for (int leg = 0; leg < 4; ++leg) {
        if (!contacts[leg]) continue;
        const Eigen::Vector3d hip = pose.head<3>() + rotation * robot.hips.row(leg).transpose();
        Check((hip - input.feet.row(leg).transpose()).norm() <=
                  robot.reach_max[leg] - settings.max_reach_margin + 1e-4,
              "support exceeds reduced reach limit");
      }
      Check((Evaluate(result, time) - Evaluate(independent, time)).norm() < 2e-5,
            "non-supporting foot changed the body plan");
      Check((Evaluate(result, time) - Evaluate(dense, time)).norm() < 2e-5,
            "three-support block and dense assembly disagree");
    }
  }
  // 대기 접촉만 앞에 붙이고 이륙·착지의 trot 순서는 지연된 원래 일정과 일치한다.
  for (int missing = 0; missing < 4; ++missing) {
    std::array<bool, 4> contacts{true, true, true, true};
    contacts[missing] = false;
    const auto held = MakeHorizon(.4, robot.period, robot.duty, .14, contacts);
    const auto trot = MakeHorizon(.4, robot.period, robot.duty);
    Check(held.front().contacts == contacts, "hold changed actual support combination");
    for (std::size_t k = 1; k < held.size(); ++k) {
      Check(held[k].contacts == trot[k - 1].contacts &&
            held[k].landed == trot[k - 1].landed &&
            std::abs(held[k].start - .14 - trot[k - 1].start) < 1e-10,
            "three-support hold changed the trot order");
    }
  }
}
}  // namespace

int main() {
  try {
    CheckKernels();
    CheckNominalKinematics();
    CheckPlanner();
    std::cout << "PASS four tripod combinations, support sensitivity, Jacobians, B2 planning, trot order\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
