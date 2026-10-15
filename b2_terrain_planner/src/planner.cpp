/**
 * @file planner.cpp
 * @brief 생성된 Jacobian과 지형 비용의 Gauss–Newton QP로 trot을 최적화한다.
 * @version 0.0.27
 * @update 2026-10-15: 제약 해제 분기와 탐색 추적 전용 계산을 제거하고 전체 제약 경로를 유지함.
 * @update 2026-10-14: 최적화 후 2배 조밀 격자 재검증 게이트(dense_validation)를 제거함.
 */
#include "b2_terrain_planner/planner.h"
#include "b2_terrain_planner/robot_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include "dense_svd.h"
#include <osqp.h>

extern "C" {
#include "tamols_kernels.h"
}

namespace b2_terrain_planner {
namespace {
using Clock = std::chrono::steady_clock;
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
enum class EvaluationMode { kFull, kValues, kConstraintJacobian };

class ScopedTimer {
 public:
  explicit ScopedTimer(double* seconds) : seconds_(seconds), begin_(Clock::now()) {}
  ~ScopedTimer() { Stop(); }
  void Stop() {
    if (seconds_ != nullptr) {
      *seconds_ += std::chrono::duration<double>(Clock::now() - begin_).count();
      seconds_ = nullptr;
    }
  }
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  double* seconds_;
  Clock::time_point begin_;
};

std::array<bool, 4> Contacts(double time, double period, double duty) {
  std::array<bool, 4> contacts{};
  const std::array<double, 4> offsets{0.0, 0.5, 0.5, 0.0};
  for (int leg = 0; leg < 4; ++leg) {
    double phase = time / period + offsets[leg];
    phase -= std::floor(phase);
    contacts[leg] = phase < duty - 1e-10;
  }
  return contacts;
}

Eigen::Matrix3d Rotation(const Vector6& q) {
  return (Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Matrix<double, 5, 1> Basis(double s, double duration, int derivative) {
  Eigen::Matrix<double, 5, 1> result = Eigen::Matrix<double, 5, 1>::Zero();
  for (int power = derivative; power < 5; ++power) {
    double factor = 1.0;
    for (int order = 0; order < derivative; ++order) factor *= power - order;
    result[power] = factor * std::pow(s, power - derivative) /
                    std::pow(duration, derivative);
  }
  return result;
}

enum ConstraintGroup {
  kStructure, kFoothold, kGiacEquality, kGiacInequality,
  kFriction, kKinematic, kMotion, kTerrain, kGroupCount
};
const std::array<std::string, kGroupCount> kGroupNames{
    "structure", "foothold", "giac-equality", "giac-inequality",
    "friction", "kinematic", "motion", "terrain"};

struct Evaluation {
  double cost = 0.0;
  Vector gradient;
  Matrix hessian;
  Vector values;
  Vector lower;
  Vector upper;
  Matrix jacobian;
  std::vector<ConstraintDiagnostic> groups;
  std::vector<int> row_groups;
  std::vector<int> jacobian_indices;

  auto JacobianRow(int row) const {
    int index = row;
    if (!jacobian_indices.empty()) index = jacobian_indices[row];
    if (index < 0) throw std::logic_error("inactive constraint Jacobian requested");
    return jacobian.row(index);
  }

  Vector LinearChange(const Vector& direction) const {
    Vector change = Vector::Zero(values.size());
    if (values.size() > 0) change.noalias() = jacobian * direction;
    return change;
  }
};

struct EqualitySolution {
  Matrix inverse;
  Vector particular;
  Matrix nullspace;
  int rank = 0;
};

// 초기조건과 C1 연결은 선형·상수이다. 이 부분을 정확하게 제거한 뒤
// GIAC 및 활성 부등식의 작은 행렬만 반복 분해한다. 수명은 한 Solve다.
class StructureCache {
 public:
  EqualitySolution Solve(const Evaluation& eval, const std::vector<int>& indices,
                          const Vector& scaling, double threshold, bool lapack,
                          bool need_nullspace, Profile* profile, bool need_inverse = false) {
    ScopedTimer timer(&profile->svd_seconds);
    std::vector<int> structure, nonlinear;
    for (int row : indices) {
      if (eval.row_groups[row] == kStructure) structure.push_back(row);
      else nonlinear.push_back(row);
    }
    const int n = eval.jacobian.cols();
    if (structure != rows_ || nullspace_.rows() != n) {
      rows_ = structure;
      Matrix a(structure.size(), n);
      for (int i = 0; i < static_cast<int>(structure.size()); ++i) a.row(i) = eval.JacobianRow(structure[i]);
      if (structure.empty()) {
        inverse_.resize(n, 0);
        nullspace_ = Matrix::Identity(n, n);
      } else {
        // 구조 행은 설계상 독립이다. 직교 QR로 최소 노름 특수해와 영공간을
        // 함께 얻으며, 매우 짧은 구간의 수치 퇴화는 기존 SVD로 재확인한다.
        const int m = a.rows();
        Eigen::HouseholderQR<Matrix> qr(a.transpose());
        const Matrix triangular = qr.matrixQR().topRows(m).template triangularView<Eigen::Upper>();
        const double largest = triangular.diagonal().cwiseAbs().maxCoeff();
        const double smallest = triangular.diagonal().cwiseAbs().minCoeff();
        if (smallest > 1e-12 * largest) {
          const Matrix orthogonal = qr.householderQ() * Matrix::Identity(n, n);
          inverse_ = orthogonal.leftCols(m) *
              triangular.transpose().template triangularView<Eigen::Lower>().solve(Matrix::Identity(m, m));
          nullspace_ = orthogonal.rightCols(n - m);
        } else {
          ++profile->svd_calls;
          DenseSvd svd(a, 1e-12, true, lapack);
          if (svd.rank() != a.rows()) throw std::runtime_error("dependent structural equalities");
          inverse_.resize(n, a.rows());
          for (int i = 0; i < a.rows(); ++i) {
            inverse_.col(i) = svd.Solve(Vector::Unit(a.rows(), i));
          }
          nullspace_ = svd.v().rightCols(n - svd.rank());
        }
      }
    }
    Vector structural_rhs(structure.size());
    for (int i = 0; i < structural_rhs.size(); ++i) {
      const int row = structure[i];
      structural_rhs[i] = eval.lower[row] - eval.values[row];
    }
    EqualitySolution solution;
    solution.particular = Vector::Zero(n);
    if (!structure.empty()) solution.particular.noalias() = inverse_ * structural_rhs;
    Matrix residual_map;
    if (need_inverse) {
      Matrix structure_map = Matrix::Zero(structure.size(), indices.size());
      residual_map = Matrix::Zero(nonlinear.size(), indices.size());
      int structural_row = 0, nonlinear_row = 0;
      for (int col = 0; col < static_cast<int>(indices.size()); ++col) {
        const int row = indices[col];
        if (eval.row_groups[row] == kStructure) structure_map(structural_row++, col) = 1.0;
        else residual_map(nonlinear_row++, col) = 1.0 / scaling[row];
      }
      solution.inverse = Matrix::Zero(n, indices.size());
      if (!structure.empty()) solution.inverse.noalias() = inverse_ * structure_map;
    }
    solution.rank = structure.size();
    if (nonlinear.empty()) {
      if (need_nullspace) solution.nullspace = nullspace_;
      return solution;
    }
    Matrix a(nonlinear.size(), n);
    Vector rhs(nonlinear.size());
    for (int i = 0; i < static_cast<int>(nonlinear.size()); ++i) {
      const int row = nonlinear[i];
      a.row(i) = eval.JacobianRow(row) / scaling[row];
      rhs[i] = (std::clamp(eval.values[row], eval.lower[row], eval.upper[row]) -
                eval.values[row]) / scaling[row];
    }
    rhs.noalias() -= a * solution.particular;
    if (nullspace_.cols() == 0) {
      solution.nullspace.resize(n, 0);
      return solution;
    }
    ++profile->svd_calls;
    DenseSvd svd(a * nullspace_, threshold, need_nullspace, lapack);
    solution.particular.noalias() += nullspace_ * svd.Solve(rhs);
    if (need_inverse) {
      residual_map.noalias() -= a * solution.inverse;
      solution.inverse.noalias() += nullspace_ * svd.PseudoInverse() * residual_map;
    }
    solution.rank += svd.rank();
    if (need_nullspace) {
      solution.nullspace.noalias() = nullspace_ *
          svd.v().rightCols(nullspace_.cols() - svd.rank());
    }
    return solution;
  }

 private:
  std::vector<int> rows_;
  Matrix inverse_, nullspace_;
};

Eigen::Matrix<double, 4, 2, Eigen::RowMajor> NominalFootholds(
    const Robot& robot, const Input& input, const std::vector<Phase>& phases) {
  if (input.has_nominal) return input.nominal;
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> nominal;
  Eigen::Vector3d command(input.command.x(), input.command.y(), 0);
  command = Eigen::AngleAxisd(input.position[3], Eigen::Vector3d::UnitZ()) * command;
  for (int leg = 0; leg < 4; ++leg) {
    double touchdown = robot.period;
    for (const auto& phase : phases) {
      if (phase.landed[leg]) { touchdown = phase.start; break; }
    }
    Eigen::Vector3d offset = robot.hips.row(leg).transpose();
    if (robot.use_reference_legs) offset = robot.nominal_offsets.row(leg).transpose();
    nominal.row(leg) = (input.position.head<3>() + Rotation(input.position) * offset +
        command * (touchdown + .5 * robot.period * robot.duty)).head<2>().transpose();
  }
  return nominal;
}

class Problem {
 public:
  Problem(const Robot& robot, const Input& input, int intervals,
          Profile* profile, bool full_evaluation, const TerrainModel* terrain,
          bool block_assembly, bool polynomial_tail,
          double pitch_weight, double leg_vertical_weight, double orientation_limit,
          double backward_weight, double vz_weight)
      : robot_(robot), input_(input), intervals_(intervals),
        phases_(MakeHorizon(input.gait_time, robot.period, robot.duty)),
        profile_(profile), full_evaluation_(full_evaluation), terrain_(terrain),
        block_assembly_(block_assembly), polynomial_tail_(polynomial_tail) {
    feet_offset_ = 30 * static_cast<int>(phases_.size());
    if (terrain_ != nullptr) foot_dimensions_ = 3;
    size_ = feet_offset_ + 4 * foot_dimensions_;
    for (const Phase& phase : phases_) {
      margin_.push_back(-1);
      if (std::count(phase.contacts.begin(), phase.contacts.end(), true) == 4) {
        margin_.back() = size_++;
      }
    }
    parameters_ = Vector::Zero(54);
    parameters_[0] = robot.mass;
    parameters_.segment<9>(1) = Eigen::Map<const Vector>(robot.inertia.data(), 9);
    parameters_.segment<12>(10) = Eigen::Map<const Vector>(robot.hips.data(), 12);
    parameters_[22] = robot.height;
    parameters_[23] = robot.friction;
    parameters_[24] = robot.leg_min;
    parameters_[25] = robot.leg_max;
    parameters_.segment<3>(26) = input.command;
    parameters_.segment<12>(29) = Eigen::Map<const Vector>(robot.reference_legs.data(), 12);
    if (robot.use_reference_legs) parameters_[41] = 1;
    parameters_[42] = std::sqrt(pitch_weight);
    parameters_[43] = std::sqrt(leg_vertical_weight);
    parameters_[44] = orientation_limit;
    parameters_.segment<4>(45) = robot.reach_min;
    parameters_.segment<4>(49) = robot.reach_max;
    parameters_[53] = std::sqrt(vz_weight);
    backward_weight_ = backward_weight;
    Eigen::Vector3d world_command(input.command.x(), input.command.y(), 0.0);
    world_command = Eigen::AngleAxisd(input.position[3], Eigen::Vector3d::UnitZ()) * world_command;
    if (world_command.head<2>().norm() > 1e-6) {
      direction_ = world_command.head<2>().normalized();
    }
    nominal_ = NominalFootholds(robot, input, phases_);
    if (terrain_ != nullptr) {
      if (input_.frame != terrain_->map().frame) {
        throw std::invalid_argument("terrain/input frame mismatch");
      }
      for (int leg = 0; leg < 4; ++leg) {
        const Eigen::Vector2d xy = input_.feet.row(leg).head<2>().transpose();
        if (!terrain_->IsQueryable(xy) ||
            std::abs(terrain_->Sample(xy).height - input_.feet(leg, 2)) >
                terrain_->settings().contact_tolerance) {
          throw std::invalid_argument("input feet do not match observed terrain height");
        }
        double limit = 0.30;
        if (input_.fixed[leg]) limit = 0.0;
        const Eigen::Vector2d nominal = nominal_.row(leg).transpose();
        terrain_initial_.row(leg) = terrain_->FindFoothold(
            nominal, nominal.array() - limit, nominal.array() + limit,
            direction_, backward_weight_).transpose();
      }
    }
  }

  Vector Initial() const {
    Vector x = Vector::Zero(size_);
    for (int k = 0; k < static_cast<int>(phases_.size()); ++k) {
      for (int axis = 0; axis < 6; ++axis) {
        x[30 * k + 5 * axis] = input_.position[axis] + input_.velocity[axis] * phases_[k].start;
        x[30 * k + 5 * axis + 1] = input_.velocity[axis] * phases_[k].duration;
      }
    }
    for (int leg = 0; leg < 4; ++leg) {
      for (int axis = 0; axis < 2; ++axis) {
        double limit = 0.30;
        if (input_.fixed[leg]) limit = 0.0;
        x[feet_offset_ + foot_dimensions_ * leg + axis] = std::clamp(input_.feet(leg, axis),
            nominal_(leg, axis) - limit, nominal_(leg, axis) + limit);
      }
    }
    if (terrain_ != nullptr) {
      for (int leg = 0; leg < 4; ++leg) {
        x.segment<3>(feet_offset_ + 3 * leg) = terrain_initial_.row(leg).transpose();
      }
    }
    return x;
  }

  Vector Initial(const Result* previous, WarmStartDiagnostic* diagnostic, int replaced_leg = -1) const {
    Vector x = Initial();
    if (previous == nullptr) return x;
    diagnostic->requested = true;
    const auto& old = *previous;
    const auto& context = old.seed_context;
    auto reject = [&](const char* reason) {
      diagnostic->reason = reason;
      return x;
    };
    if (!old.success) {
      return reject("previous_plan_not_usable");
    }
    if (context.terrain != input_.terrain.has_value() ||
        context.terrain_signature != TerrainSignature(input_.terrain)) {
      return reject("terrain_mismatch");
    }
    if (context.frame != input_.frame || context.robot_signature != RobotSignature(robot_) ||
        context.period != robot_.period || context.duty != robot_.duty) {
      return reject("context_mismatch");
    }
    const double shift = input_.gait_time - context.gait_time;
    if (!std::isfinite(shift) || shift < 0 || shift >= robot_.period) {
      return reject("previous_plan_expired_or_future");
    }
    const auto expected = MakeHorizon(context.gait_time, context.period, context.duty);
    if (old.phases.size() != expected.size() || old.coefficients.size() != expected.size() ||
        old.margins.size() != expected.size() || !old.footholds.allFinite() ||
        !context.initial_feet.allFinite()) return reject("invalid_previous_plan");
    for (size_t k = 0; k < expected.size(); ++k) {
      const auto& phase = old.phases[k];
      if (!std::isfinite(phase.start) || !std::isfinite(phase.duration) ||
          std::abs(phase.start - expected[k].start) > 1e-10 ||
          std::abs(phase.duration - expected[k].duration) > 1e-10 ||
          phase.contacts != expected[k].contacts || phase.landed != expected[k].landed ||
          !old.coefficients[k].allFinite() || !std::isfinite(old.margins[k]) ||
          old.margins[k] < -.10001 || old.margins[k] > 1e-5) {
        return reject("invalid_previous_plan");
      }
    }
    double horizon = 0;
    for (const auto& phase : old.phases) horizon += phase.duration;
    const Vector6 end_position = Evaluate(old, horizon);
    const Vector6 end_velocity = Evaluate(old, horizon, 1);
    const Vector6 position_delta = input_.position - Evaluate(old, shift);
    const Vector6 velocity_delta = input_.velocity - Evaluate(old, shift, 1);
    // 새 측정 상태와의 affine 차이를 전체 구간에 더해 C1 연속성을 보존한다.
    for (size_t k = 0; k < phases_.size(); ++k) {
      const auto& phase = phases_[k];
      const double start = shift + phase.start;
      const double finish = start + phase.duration;
      Eigen::Matrix<double, 6, 5> c = Eigen::Matrix<double, 6, 5>::Zero();
      size_t source = 0;
      while (source + 1 < old.phases.size() &&
             start >= old.phases[source + 1].start - 1e-10) ++source;
      const bool extend_polynomial = polynomial_tail_ && source + 1 == old.phases.size() &&
          start < horizon - 1e-10 && phase.contacts == old.phases[source].contacts;
      if (finish <= horizon + 1e-10 || extend_polynomial) {
        // 4차 다항식을 이항 전개하여 구간의 새 원점으로 정확히 이동한다.
        const double offset = start - old.phases[source].start;
        for (int power = 0; power < 5; ++power) {
          double binomial = 1;
          for (int target = 0; target <= power; ++target) {
            c.col(target) += binomial * std::pow(offset, power - target) *
                             old.coefficients[source].col(power);
            binomial *= static_cast<double>(power - target) / (target + 1);
          }
        }
        if (margin_[k] >= 0) x[margin_[k]] = std::clamp(old.margins[source], -.1, 0.0);
        if (finish > horizon + 1e-10) {
          ++diagnostic->extended_phases;
          ++diagnostic->polynomial_extended_phases;
        } else {
          ++diagnostic->shifted_phases;
        }
      } else {
        // 이전 horizon을 가로지르는 마지막 구간은 Hermite 연결한다.
        // 겹치지 않는 구간은 이전 끝 속도의 선형 연장이 된다.
        Vector6 p0 = end_position + end_velocity * (start - horizon);
        Vector6 v0 = end_velocity;
        if (start < horizon) {
          p0 = Evaluate(old, start);
          v0 = Evaluate(old, start, 1);
        }
        const Vector6 p1 = end_position + end_velocity * (finish - horizon);
        const double duration = phase.duration;
        c.col(0) = p0;
        c.col(1) = v0;
        c.col(2) = 3 * (p1 - p0) / (duration * duration) -
                   (2 * v0 + end_velocity) / duration;
        c.col(3) = 2 * (p0 - p1) / std::pow(duration, 3) +
                   (v0 + end_velocity) / (duration * duration);
        ++diagnostic->extended_phases;
      }
      c.col(0) += position_delta + velocity_delta * phase.start;
      c.col(1) += velocity_delta;
      for (int axis = 0; axis < 6; ++axis) {
        for (int power = 0; power < 5; ++power) {
          x[30 * k + 5 * axis + power] = c(axis, power) * std::pow(phase.duration, power);
        }
      }
    }
    // 이전 몸통 궤적에서 새 발이 지지하는 구간을 예측한다. 후보 선택의 여유이며
    // SQP 하드 제약이나 최종 IK 허용 오차를 완화하지 않는다.
    Feet selected_terrain = terrain_initial_;
    std::array<std::function<bool(const Eigen::Vector3d&)>, 4> reach_accept;
    if (terrain_ != nullptr && robot_.model) {
      for (int leg = 0; leg < 4; ++leg) {
        if (input_.fixed[leg]) continue;
        std::vector<Eigen::Vector3d> hips;
        for (size_t k = 0; k < phases_.size(); ++k) {
          if (!phases_[k].contacts[leg] || !phases_[k].landed[leg]) continue;
          const Eigen::Map<const Eigen::Matrix<double, 6, 5, Eigen::RowMajor>> c(x.data() + 30 * k);
          for (int sample = 0; sample <= 4; ++sample) {
            const Vector6 pose = c * Basis(sample / 4.0, phases_[k].duration, 0);
            hips.push_back(pose.head<3>() + Rotation(pose) * robot_.hips.row(leg).transpose());
          }
        }
        if (hips.empty()) continue;
        const double minimum = robot_.reach_min[leg];
        const double maximum = robot_.reach_max[leg] - 0.02;
        reach_accept[leg] = [hips, minimum, maximum](const Eigen::Vector3d& foot) {
          for (const auto& hip : hips) {
            const double distance = (hip - foot).norm();
            if (distance < minimum || distance > maximum) return false;
          }
          return true;
        };
        const Eigen::Vector2d nominal = nominal_.row(leg).transpose();
        if (reach_accept[leg](terrain_initial_.row(leg).transpose())) continue;
        try {
          selected_terrain.row(leg) = terrain_->FindFoothold(
              nominal, nominal.array() - .30, nominal.array() + .30,
              direction_, backward_weight_, reach_accept[leg]).transpose();
          if ((selected_terrain.row(leg) - terrain_initial_.row(leg)).norm() > 1e-10) {
            ++diagnostic->reach_reselected_footholds;
          }
          x.segment<3>(feet_offset_ + 3 * leg) = selected_terrain.row(leg).transpose();
        } catch (const TerrainDomainError&) {
          // 예측 몸통 자체를 SQP가 수정해야 하는 경우에는 원래 후보로 시작한다.
          ++diagnostic->reach_candidate_fallbacks;
          reach_accept[leg] = {};
        }
      }
    }
    for (int leg = 0; leg < 4; ++leg) {
      if (leg == replaced_leg) continue;
      double touchdown = robot_.period;
      for (const auto& phase : old.phases) {
        if (phase.landed[leg]) {
          touchdown = phase.start;
          break;
        }
      }
      // 이미 소비한 착지 목표는 다음 주기의 목표로 복사하지 않는다.
      if (touchdown <= shift + 1e-10 || input_.fixed[leg]) continue;
      if (terrain_ != nullptr) {
        const Eigen::Vector3d foot = old.footholds.row(leg).transpose();
        // XY를 잘라 다른 단으로 옮기면 Z가 맞지 않는다. 기존 목표 전체가
        // 현재 범위·관측 지형 높이에 유효할 때만 복사하고 아니면 새 후보를 유지한다.
        if ((foot.head<2>() - nominal_.row(leg).transpose()).cwiseAbs().maxCoeff() > .30 ||
            !terrain_->IsQueryable(foot.head<2>()) ||
            std::abs(foot.z() - terrain_->Sample(foot.head<2>()).height) >
                terrain_->settings().contact_tolerance) continue;
        if (reach_accept[leg] && !reach_accept[leg](foot)) continue;
        if (backward_weight_ > 0 && direction_.squaredNorm() > 0) {
          const Eigen::Vector2d nominal = nominal_.row(leg).transpose();
          const double old_score = terrain_->FootholdCost(
              foot.head<2>(), nominal, direction_, backward_weight_);
          const double new_score = terrain_->FootholdCost(
              selected_terrain.row(leg).head<2>().transpose(), nominal,
              direction_, backward_weight_);
          if (new_score + 1e-10 < old_score) continue;
        }
        x.segment<3>(feet_offset_ + 3 * leg) = foot;
        ++diagnostic->reused_footholds;
        continue;
      }
      for (int axis = 0; axis < 2; ++axis) {
        x[feet_offset_ + foot_dimensions_ * leg + axis] = std::clamp(old.footholds(leg, axis),
            nominal_(leg, axis) - .30, nominal_(leg, axis) + .30);
      }
      ++diagnostic->reused_footholds;
    }
    if (!x.allFinite()) {
      *diagnostic = WarmStartDiagnostic();
      diagnostic->requested = true;
      diagnostic->reason = "nonfinite_shifted_plan";
      return Initial();
    }
    diagnostic->used = true;
    diagnostic->reason = "shifted_previous_plan";
    diagnostic->elapsed = shift;
    return x;
  }

  Feet InitialFootholds(const Vector& x) const {
    Feet feet = input_.feet;
    for (int leg = 0; leg < 4; ++leg) {
      feet.row(leg) = x.segment<3>(feet_offset_ + 3 * leg).transpose();
    }
    return feet;
  }

  void ReplaceFootholdSeed(int leg, const Eigen::Vector3d& seed, Vector* x) const {
    x->segment<3>(feet_offset_ + 3 * leg) = seed;
  }

  Matrix SampleTransform(int k, double s) const {
    Matrix transform = Matrix::Zero(31, size_);
    for (int derivative = 0; derivative < 3; ++derivative) {
      auto basis = Basis(s, phases_[k].duration, derivative);
      for (int axis = 0; axis < 6; ++axis) {
        transform.block<1, 5>(6 * derivative + axis, 30 * k + 5 * axis) = basis.transpose();
      }
    }
    for (int leg = 0; leg < 4; ++leg) {
      if (phases_[k].landed[leg]) {
        transform.block(18 + 3 * leg, feet_offset_ + foot_dimensions_ * leg,
                        foot_dimensions_, foot_dimensions_).setIdentity();
      }
    }
    if (margin_[k] >= 0) transform(30, margin_[k]) = 1.0;
    return transform;
  }

  Evaluation EvaluateAt(const Vector& x, EvaluationMode mode = EvaluationMode::kFull,
                        int intervals = 0) const {
    if (full_evaluation_) mode = EvaluationMode::kFull;
    double* seconds = &profile_->full_evaluation_seconds;
    if (mode == EvaluationMode::kFull) {
      ++profile_->full_evaluations;
    } else if (mode == EvaluationMode::kValues) {
      seconds = &profile_->values_evaluation_seconds;
      ++profile_->values_evaluations;
    } else {
      seconds = &profile_->constraint_jacobian_seconds;
      ++profile_->constraint_jacobian_evaluations;
    }
    ScopedTimer timer(seconds);
    const bool full = mode == EvaluationMode::kFull;
    const bool need_jacobian = mode != EvaluationMode::kValues;
    const bool need_cost = mode != EvaluationMode::kConstraintJacobian;
    const bool compact = block_assembly_ && mode == EvaluationMode::kConstraintJacobian;
    if (intervals == 0) intervals = intervals_;
    Evaluation out;
    if (full) {
      out.gradient = Vector::Zero(size_);
      out.hessian = Matrix::Zero(size_, size_);
    }
    out.groups.resize(kGroupCount);
    for (int group = 0; group < kGroupCount; ++group) {
      out.groups[group].name = kGroupNames[group];
    }
    ConstraintGroup group = kFoothold;
    int phase_index = -1, sample_index = -1, local_row = -1;
    std::vector<double> values, lower, upper;
    std::vector<Eigen::RowVectorXd> rows;
    const int row_capacity = 128 + phases_.size() * (13 + 26 * (intervals + 1));
    rows.reserve(row_capacity);
    values.reserve(row_capacity);
    lower.reserve(row_capacity);
    upper.reserve(row_capacity);
    out.row_groups.reserve(row_capacity);
    auto add = [&](double value, double lo, double hi) {
      auto& diagnostic = out.groups[group];
      ++diagnostic.rows;
      const double violation = std::max({0.0, lo - value, value - hi});
      if (violation > diagnostic.max_violation || diagnostic.rows == 1) {
        diagnostic.max_violation = violation;
        diagnostic.phase = phase_index;
        diagnostic.sample = sample_index;
        diagnostic.row = local_row;
      }
      out.row_groups.push_back(group);
      values.push_back(value);
      lower.push_back(lo);
      upper.push_back(hi);
    };
    for (int leg = 0; leg < 4; ++leg) {
      const int foot_column = feet_offset_ + foot_dimensions_ * leg;
      const Eigen::Vector2d offset = x.segment<2>(foot_column) - nominal_.row(leg).transpose();
      const double behind = std::min(0.0, direction_.dot(offset));
      if (need_cost) out.cost += backward_weight_ * behind * behind;
      if (full && behind < 0) {
        out.gradient.segment<2>(foot_column) += 2 * backward_weight_ * behind * direction_;
        out.hessian.block<2, 2>(foot_column, foot_column) +=
            2 * backward_weight_ * direction_ * direction_.transpose();
      }
      for (int axis = 0; axis < 2; ++axis) {
        const int column = feet_offset_ + foot_dimensions_ * leg + axis;
        const double residual = x[column] - nominal_(leg, axis);
        if (need_cost) out.cost += 0.7 * residual * residual;
        if (full) {
          out.gradient[column] += 1.4 * residual;
          out.hessian(column, column) += 1.4;
        }
        double limit = 0.30;
        if (input_.fixed[leg]) limit = 0.0;
        add(x[column], nominal_(leg, axis) - limit, nominal_(leg, axis) + limit);
        if (need_jacobian) rows.push_back(Eigen::RowVectorXd::Unit(size_, column));
      }
    }
    if (terrain_ != nullptr) {
      group = kTerrain;
      for (int leg = 0; leg < 4; ++leg) {
        const int column = feet_offset_ + 3 * leg;
        const auto residual = terrain_->Residual(x.segment<3>(column));
        if (need_cost) out.cost += residual.values.squaredNorm();
        if (full) {
          out.gradient.segment<3>(column).noalias() +=
              2 * residual.jacobian.transpose() * residual.values;
          out.hessian.block<3, 3>(column, column).noalias() +=
              2 * residual.jacobian.transpose() * residual.jacobian;
        }
        double z_limit = robot_.leg_max;
        if (input_.fixed[leg]) z_limit = 0.0;
        add(x[column + 2], terrain_initial_(leg, 2) - z_limit,
            terrain_initial_(leg, 2) + z_limit);
        if (need_jacobian) rows.push_back(Eigen::RowVectorXd::Unit(size_, column + 2));
      }
      // 현재/새 착지점의 조합마다 검사한다. 실제 스윙 링크 형상은 모델링하지 않는다.
      std::set<std::array<bool, 4>> configurations;
      for (const auto& phase : phases_) configurations.insert(phase.landed);
      configurations.insert({true, true, true, true});
      for (const auto& landed : configurations) {
        const Feet feet = Configuration(x, landed);
        for (int i = 0; i < 4; ++i) {
          for (int j = i + 1; j < 4; ++j) {
            const auto residual = terrain_->Separation(
                (feet.row(i) - feet.row(j)).head<2>().transpose());
            if (need_cost) out.cost += residual.value * residual.value;
            if (full) {
              Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(size_);
              if (landed[i]) row.segment<2>(feet_offset_ + 3 * i) += residual.gradient.transpose();
              if (landed[j]) row.segment<2>(feet_offset_ + 3 * j) -= residual.gradient.transpose();
              out.gradient.noalias() += 2 * residual.value * row.transpose();
              out.hessian.noalias() += 2 * row.transpose() * row;
            }
          }
        }
        const std::array<int, 4> order{0, 2, 3, 1};
        for (int index = 0; index < 4; ++index) {
          const int i = order[index], j = order[(index + 1) % 4];
          const int h = order[(index + 2) % 4];
          const Eigen::Vector2d a = (feet.row(j) - feet.row(i)).head<2>().transpose();
          const Eigen::Vector2d b = (feet.row(h) - feet.row(i)).head<2>().transpose();
          add(a.x() * b.y() - a.y() * b.x(), 0.01,
              std::numeric_limits<double>::infinity());
          if (need_jacobian) {
            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(size_);
            if (landed[i]) row.segment<2>(feet_offset_ + 3 * i) <<
                feet(j, 1) - feet(h, 1), feet(h, 0) - feet(j, 0);
            if (landed[j]) row.segment<2>(feet_offset_ + 3 * j) <<
                feet(h, 1) - feet(i, 1), feet(i, 0) - feet(h, 0);
            if (landed[h]) row.segment<2>(feet_offset_ + 3 * h) <<
                feet(i, 1) - feet(j, 1), feet(j, 0) - feet(i, 0);
            rows.push_back(row);
          }
        }
      }
    }
    for (int k = 0; k < static_cast<int>(phases_.size()); ++k) {
      group = kStructure;
      phase_index = k;
      sample_index = -1;
      Matrix start = SampleTransform(k, 0.0).topRows(12);
      Vector target(12);
      target << input_.position, input_.velocity;
      if (k > 0) {
        start -= SampleTransform(k - 1, 1.0).topRows(12);
        target.setZero();
      }
      Vector start_value = start * x;
      for (int row = 0; row < 12; ++row) {
        local_row = row;
        add(start_value[row], target[row], target[row]);
        if (need_jacobian) rows.push_back(start.row(row));
      }
      if (margin_[k] >= 0) {
        const int column = margin_[k];
        if (need_cost) out.cost += 0.007 * x[column];
        if (full) out.gradient[column] += 0.007;
        group = kGiacInequality;
        local_row = -1;
        add(x[column], -0.1, 0.0);
        if (need_jacobian) rows.push_back(Eigen::RowVectorXd::Unit(size_, column));
      }
      auto kernel = tamols_diagonal_a;
      auto values_kernel = tamols_diagonal_a_values;
      auto constraints_kernel = tamols_diagonal_a_constraint_jacobian;
      int nr = 20;
      int ng = 17;
      const bool full_support = std::count(phases_[k].contacts.begin(), phases_[k].contacts.end(), true) == 4;
      if (full_support) {
        kernel = tamols_full_support;
        values_kernel = tamols_full_support_values;
        constraints_kernel = tamols_full_support_constraint_jacobian;
        nr = 28;
        ng = 26;
      } else if (!phases_[k].contacts[0]) {
        kernel = tamols_diagonal_b;
        values_kernel = tamols_diagonal_b_values;
        constraints_kernel = tamols_diagonal_b_constraint_jacobian;
      }
      std::vector<int> columns;
      if (block_assembly_) {
        for (int col = 30 * k; col < 30 * (k + 1); ++col) columns.push_back(col);
        for (int leg = 0; leg < 4; ++leg) {
          if (!phases_[k].landed[leg]) continue;
          for (int axis = 0; axis < foot_dimensions_; ++axis) {
            columns.push_back(feet_offset_ + foot_dimensions_ * leg + axis);
          }
        }
        if (margin_[k] >= 0) columns.push_back(margin_[k]);
      }
      const double weight = phases_[k].duration / (intervals + 1);
      for (int sample = 0; sample <= intervals; ++sample) {
        const double sample_time = static_cast<double>(sample) / intervals;
        Matrix transform;
        std::array<Eigen::Matrix<double, 5, 1>, 3> basis;
        Vector z;
        if (block_assembly_) {
          z = Vector::Zero(31);
          using Coefficients = Eigen::Matrix<double, 6, 5, Eigen::RowMajor>;
          const Eigen::Map<const Coefficients> coefficients(x.data() + 30 * k);
          for (int derivative = 0; derivative < 3; ++derivative) {
            basis[derivative] = Basis(sample_time, phases_[k].duration, derivative);
            z.segment<6>(6 * derivative).noalias() = coefficients * basis[derivative];
          }
          for (int leg = 0; leg < 4; ++leg) {
            if (phases_[k].landed[leg]) z.segment(18 + 3 * leg, foot_dimensions_) =
                x.segment(feet_offset_ + foot_dimensions_ * leg, foot_dimensions_);
          }
          if (margin_[k] >= 0) z[30] = x[margin_[k]];
        } else {
          transform = SampleTransform(k, sample_time);
          z.noalias() = transform * x;
        }
        for (int leg = 0; leg < 4; ++leg) {
          if (terrain_ == nullptr) z[20 + 3 * leg] = input_.feet(leg, 2);
          if (!phases_[k].landed[leg]) {
            z.segment<3>(18 + 3 * leg) = input_.feet.row(leg).transpose();
          }
        }
        Vector residual, constraints(ng), lo(ng), hi(ng);
        Matrix jr, jg;
        if (need_cost) residual.resize(nr);
        if (need_jacobian) jg.resize(ng, 31);
        if (full) jr.resize(nr, 31);
        const double* arguments[] = {z.data(), parameters_.data()};
        double* outputs[6] = {};
        if (full) {
          outputs[0] = residual.data();
          outputs[1] = jr.data();
          outputs[2] = constraints.data();
          outputs[3] = jg.data();
          outputs[4] = lo.data();
          outputs[5] = hi.data();
        } else if (mode == EvaluationMode::kValues) {
          kernel = values_kernel;
          outputs[0] = residual.data();
          outputs[1] = constraints.data();
          outputs[2] = lo.data();
          outputs[3] = hi.data();
        } else {
          kernel = constraints_kernel;
          outputs[0] = constraints.data();
          outputs[1] = jg.data();
          outputs[2] = lo.data();
          outputs[3] = hi.data();
        }
        // SX 커널은 미리 생성된 해당 평가 경로의 계산만 수행한다.
        if (kernel(arguments, outputs, nullptr, nullptr, 0) != 0) {
          throw std::runtime_error("generated kernel failed");
        }
        Matrix jacobian_g;
        if (need_cost) out.cost += weight * residual.squaredNorm();
        if (block_assembly_ && need_jacobian) {
          auto compose = [&](const Matrix& derivative) {
            Matrix local(derivative.rows(), columns.size());
            for (int axis = 0; axis < 6; ++axis) {
              for (int power = 0; power < 5; ++power) {
                local.col(5 * axis + power) = derivative.col(axis) * basis[0][power] +
                    derivative.col(6 + axis) * basis[1][power] +
                    derivative.col(12 + axis) * basis[2][power];
              }
            }
            int column = 30;
            for (int leg = 0; leg < 4; ++leg) {
              if (!phases_[k].landed[leg]) continue;
              for (int axis = 0; axis < foot_dimensions_; ++axis) {
                local.col(column++) = derivative.col(18 + 3 * leg + axis);
              }
            }
            if (margin_[k] >= 0) local.col(column++) = derivative.col(30);
            return local;
          };
          if (!compact) {
            const Matrix local_g = compose(jg);
            jacobian_g = Matrix::Zero(ng, size_);
            for (int col = 0; col < static_cast<int>(columns.size()); ++col) {
              jacobian_g.col(columns[col]) = local_g.col(col);
            }
          }
          if (full) {
            const Matrix local_r = compose(jr);
            const Vector gradient = 2 * weight * local_r.transpose() * residual;
            const Matrix hessian = 2 * weight * local_r.transpose() * local_r;
            for (int col = 0; col < static_cast<int>(columns.size()); ++col) {
              out.gradient[columns[col]] += gradient[col];
              for (int row = 0; row < static_cast<int>(columns.size()); ++row) {
                out.hessian(columns[row], columns[col]) += hessian(row, col);
              }
            }
          }
        } else {
          if (need_jacobian) jacobian_g = jg * transform;
          if (full) {
            Matrix jacobian_r = jr * transform;
            out.gradient.noalias() += 2 * weight * jacobian_r.transpose() * residual;
            out.hessian.noalias() += 2 * weight * jacobian_r.transpose() * jacobian_r;
          }
        }
        for (int row = 0; row < ng; ++row) {
          double upper_bound = hi[row];
          // 마찰 경계의 표본 사이 overshoot를 줄이는 작은 가속도 여유.
          // 조밀한 사후 검증은 원래 물리 경계(0)를 사용한다.
          if (intervals == intervals_ && row >= 1 && row <= 4) upper_bound -= 0.01;
          sample_index = sample;
          local_row = row;
          if (row < 5) group = kFriction;
          else if (row < 10) group = kMotion;
          else if (full_support && row < 18) group = kGiacInequality;
          else if (!full_support && row == 10) group = kGiacEquality;
          else if (!full_support && row < 13) group = kGiacInequality;
          else group = kKinematic;
          add(constraints[row], lo[row], upper_bound);
          if (need_jacobian) {
            if (compact) {
              if (lo[row] != upper_bound && constraints[row] >= lo[row] && constraints[row] <= upper_bound) {
                rows.emplace_back();
              } else {
                Eigen::RowVectorXd derivative = Eigen::RowVectorXd::Zero(size_);
                for (int axis = 0; axis < 6; ++axis) {
                  for (int power = 0; power < 5; ++power) {
                    derivative[30 * k + 5 * axis + power] = jg(row, axis) * basis[0][power] +
                        jg(row, 6 + axis) * basis[1][power] + jg(row, 12 + axis) * basis[2][power];
                  }
                }
                for (int leg = 0; leg < 4; ++leg) {
                  if (!phases_[k].landed[leg]) continue;
                  for (int axis = 0; axis < foot_dimensions_; ++axis) {
                    derivative[feet_offset_ + foot_dimensions_ * leg + axis] = jg(row, 18 + 3 * leg + axis);
                  }
                }
                if (margin_[k] >= 0) derivative[margin_[k]] = jg(row, 30);
                rows.push_back(std::move(derivative));
              }
            } else {
              rows.push_back(jacobian_g.row(row));
            }
          }
        }
      }
    }
    const int count = out.row_groups.size();
    out.values.resize(count);
    out.lower.resize(count);
    out.upper.resize(count);
    out.jacobian.resize(0, size_);
    if (need_jacobian) {
      int jacobian_count = count;
      if (compact) {
        jacobian_count = 0;
        out.jacobian_indices.assign(count, -1);
        for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
          if (rows[row].size() > 0) ++jacobian_count;
        }
      }
      out.jacobian.resize(jacobian_count, size_);
    }
    int destination = 0, jacobian_destination = 0;
    for (int row = 0; row < static_cast<int>(values.size()); ++row) {
      out.values[destination] = values[row];
      out.lower[destination] = lower[row];
      out.upper[destination] = upper[row];
      if (need_jacobian) {
        if (compact) {
          if (rows[row].size() > 0) {
            out.jacobian_indices[destination] = jacobian_destination;
            out.jacobian.row(jacobian_destination++) = rows[row];
          }
        } else {
          out.jacobian.row(destination) = rows[row];
        }
      }
      ++destination;
    }
    if (!std::isfinite(out.cost) || !out.gradient.allFinite() || !out.hessian.allFinite() ||
        !out.values.allFinite() || !out.jacobian.allFinite()) throw std::runtime_error("nonfinite evaluation");
    return out;
  }

  Feet Configuration(const Vector& x, const std::array<bool, 4>& landed) const {
    Feet feet = input_.feet;
    for (int leg = 0; leg < 4; ++leg) {
      if (landed[leg]) {
        feet.row(leg).head(foot_dimensions_) =
            x.segment(feet_offset_ + foot_dimensions_ * leg, foot_dimensions_).transpose();
      }
    }
    return feet;
  }

  void CheckKinematics(const Vector& x, Result* result) const {
    Result candidate;
    candidate.success = true;
    Export(x, &candidate);
    result->kinematics_valid = true;
    auto check = [&](double time, int leg, const Eigen::Vector3d& foot) {
      const Vector6 pose = b2_terrain_planner::Evaluate(candidate, time);
      const Eigen::Vector3d target = Rotation(pose).transpose() *
          (foot - pose.head<3>()) + robot_.com_offset;
      const auto ik = robot_.model->LegIk(leg, target,
                                         robot_.reference_joints.segment<3>(3 * leg));
      ++result->ik_checks;
      result->max_ik_error = std::max(result->max_ik_error, ik.error);
      if (!ik.success && result->kinematics_valid.value()) {
        result->kinematics_valid = false;
        result->ik_failed_leg = leg;
        result->ik_failed_time = time;
      }
    };
    for (const auto& phase : phases_) {
      const Feet feet = Configuration(x, phase.landed);
      for (int sample = 0; sample <= 24; ++sample) {
        const double time = phase.start + phase.duration * sample / 24;
        for (int leg = 0; leg < 4; ++leg) {
          if (phase.contacts[leg]) check(time, leg, feet.row(leg).transpose());
        }
      }
    }
    for (int leg = 0; leg < 4; ++leg) {
      double touchdown = robot_.period;
      for (const auto& phase : phases_) {
        if (phase.landed[leg]) { touchdown = phase.start; break; }
      }
      check(touchdown, leg, candidate.footholds.row(leg).transpose());
    }
  }

  TerrainValidation CheckTerrain(const Vector& x) const {
    TerrainValidation result;
    result.passed = true;
    result.min_foot_distance = std::numeric_limits<double>::infinity();
    const auto& settings = terrain_->settings();
    for (int leg = 0; leg < 4; ++leg) {
      const Eigen::Vector3d foot = x.segment<3>(feet_offset_ + 3 * leg);
      const double error = std::abs(foot.z() - terrain_->Sample(foot.head<2>()).height);
      result.max_contact_error = std::max(result.max_contact_error, error);
      if (!terrain_->IsQueryable(foot.head<2>())) {
        result.passed = false;
        result.reason = "foothold_not_queryable";
      }
    }
    if (result.max_contact_error > settings.contact_tolerance) {
      result.passed = false;
      result.reason = "terrain_contact_error";
    }
    std::vector<std::array<bool, 4>> configurations{{true, true, true, true}};
    for (const auto& phase : phases_) configurations.push_back(phase.landed);
    for (const auto& landed : configurations) {
      const Feet feet = Configuration(x, landed);
      for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
          result.min_foot_distance = std::min(result.min_foot_distance,
              (feet.row(i) - feet.row(j)).head<2>().norm());
        }
      }
    }
    if (result.passed) result.reason = "validated";
    return result;
  }

  void Describe(const Vector& x, const Evaluation& evaluation, Result* result) const {
    result->variable_count = x.size();
    result->margin_variable_count = std::count_if(margin_.begin(), margin_.end(),
                                                 [](int index) { return index >= 0; });
    result->constraint_count = evaluation.values.size();
    result->constraint_groups = evaluation.groups;
  }

  void Export(const Vector& x, Result* result) const {
    result->phases = phases_;
    result->footholds = input_.feet;
    for (int leg = 0; leg < 4; ++leg) {
      result->footholds.row(leg).head(foot_dimensions_) =
          x.segment(feet_offset_ + foot_dimensions_ * leg, foot_dimensions_).transpose();
    }
    for (int k = 0; k < static_cast<int>(phases_.size()); ++k) {
      Eigen::Matrix<double, 6, 5> coefficients;
      for (int axis = 0; axis < 6; ++axis) {
        for (int power = 0; power < 5; ++power) coefficients(axis, power) =
            x[30 * k + 5 * axis + power] / std::pow(phases_[k].duration, power);
      }
      result->coefficients.push_back(coefficients);
      double margin = 0;
      if (margin_[k] >= 0) margin = x[margin_[k]];
      result->margins.push_back(margin);
    }
  }

 private:
  Robot robot_;
  Input input_;
  int intervals_;
  int size_;
  int feet_offset_;
  std::vector<Phase> phases_;
  std::vector<int> margin_;
  Profile* profile_;
  bool full_evaluation_;
  const TerrainModel* terrain_;
  bool block_assembly_;
  bool polynomial_tail_;
  int foot_dimensions_ = 2;
  Feet terrain_initial_ = Feet::Zero();
  Eigen::Vector2d direction_ = Eigen::Vector2d::Zero();
  double backward_weight_ = 0.0;
  Vector parameters_;
  Eigen::Matrix<double, 4, 2, Eigen::RowMajor> nominal_;
};

double Violation(const Evaluation& eval, const Vector& scaling) {
  double result = 0.0;
  for (int row = 0; row < eval.values.size(); ++row) {
    result = std::max(result, (eval.lower[row] - eval.values[row]) / scaling[row]);
    result = std::max(result, (eval.values[row] - eval.upper[row]) / scaling[row]);
  }
  return result;
}

using QpSparse = Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int>;

struct QpPattern {
  int rows = 0;
  int cols = 0;
  std::vector<c_int> outer;
  std::vector<c_int> inner;

  bool Matches(const QpSparse& matrix) const {
    return rows == matrix.rows() && cols == matrix.cols() &&
        inner.size() == static_cast<size_t>(matrix.nonZeros()) &&
        std::equal(outer.begin(), outer.end(), matrix.outerIndexPtr(),
                   matrix.outerIndexPtr() + matrix.cols() + 1) &&
        std::equal(inner.begin(), inner.end(), matrix.innerIndexPtr(),
                   matrix.innerIndexPtr() + matrix.nonZeros());
  }

  void Assign(const QpSparse& matrix) {
    rows = matrix.rows();
    cols = matrix.cols();
    outer.assign(matrix.outerIndexPtr(), matrix.outerIndexPtr() + matrix.cols() + 1);
    inner.assign(matrix.innerIndexPtr(), matrix.innerIndexPtr() + matrix.nonZeros());
  }
};

// 한 Solve가 reduced와 elastic의 해·구조를 각각 소유하여 서로 섞이지 않게 한다.
struct QpCache {
  explicit QpCache(Profile* destination) : profile(destination) {}
  ~QpCache() { ResetWorkspace(); }
  QpCache(const QpCache&) = delete;
  QpCache& operator=(const QpCache&) = delete;

  void ResetWorkspace() {
    if (workspace == nullptr) return;
    ScopedTimer timer(&profile->qp_cleanup_seconds);
    osqp_cleanup(workspace);
    workspace = nullptr;
  }

  Profile* profile;
  OSQPWorkspace* workspace = nullptr;
  QpPattern p_pattern;
  QpPattern a_pattern;
  Vector target;  // 원래 SQP 변수에서의 이전 선형화점 + QP 방향.
  double slack = 0;
  std::map<int, double> duals;  // 원래 제약 행 단위의 multiplier.
  bool has_solution = false;
};

struct QpCoordinates {
  const Vector& origin;
  const Matrix* basis = nullptr;  // null: 원래 변수. 그 외에는 SVD의 직교 영공간.
  const Vector* particular = nullptr;
  const std::vector<int>* row_ids = nullptr;

  int RowId(int row) const {
    if (row_ids != nullptr) return (*row_ids)[row];
    return row;
  }
};

// OSQP 0.6 API adapter. 기본 경로는 매번 setup/cleanup 한다.
bool QpStep(const Evaluation& eval, const Vector& scaling, double penalty,
            double remaining, Vector* step, double* slack, std::string* status,
            Profile* profile, const Settings& solver_settings, int sqp_iteration,
            QpCache* cache, const QpCoordinates& coordinates,
            bool elastic = false) {
  profile->qp_diagnostics.emplace_back();
  auto& diagnostic = profile->qp_diagnostics.back();
  diagnostic.sqp_iteration = sqp_iteration;
  diagnostic.kind = "reduced";
  if (elastic) diagnostic.kind = "elastic";
  ScopedTimer preparation(&profile->qp_prepare_seconds);
  const bool reuse = solver_settings.qp_reuse != QpReuseMode::kNone;
  const bool warm = solver_settings.qp_reuse == QpReuseMode::kWarmStart;
  using Triplet = Eigen::Triplet<c_float, c_int>;
  const int n = eval.gradient.size();
  Matrix regularized = eval.hessian;
  regularized.diagonal().array() += solver_settings.qp_regularization;
  Eigen::LLT<Matrix> factor(regularized);
  if (factor.info() != Eigen::Success) {
    *status = "hessian_factorization_failed";
    diagnostic.status = *status;
    diagnostic.acceptance_reason = "preparation_failed";
    return false;
  }
  Matrix variable_transform = factor.matrixU().solve(Matrix::Identity(n, n));
  Matrix constraint_matrix(eval.jacobian.rows(), n);
  if (eval.jacobian.rows() > 0) constraint_matrix.noalias() = eval.jacobian * variable_transform;
  std::vector<Triplet> p_entries, a_entries;
  for (int col = 0; col < n; ++col) p_entries.emplace_back(col, col, 1.0);
  p_entries.emplace_back(n, n, 1e-5);
  Vector q(n + 1);
  q.head(n) = variable_transform.transpose() * eval.gradient;
  q[n] = 0.0;
  if (elastic) q[n] = penalty;
  std::vector<c_float> lower, upper;
  std::vector<int> row_ids;
  std::vector<double> row_scales;
  auto add_row = [&](int original_row, double bound, bool is_upper) {
    int row = lower.size();
    for (int col = 0; col < n; ++col) {
      double value = constraint_matrix(original_row, col) / scaling[original_row];
      if (value != 0) a_entries.emplace_back(row, col, value);
    }
    if (is_upper) {
      row_ids.push_back(3 * coordinates.RowId(original_row) + 2);
      a_entries.emplace_back(row, n, -1.0);
      lower.push_back(-OSQP_INFTY);
      upper.push_back(bound);
    } else {
      row_ids.push_back(3 * coordinates.RowId(original_row) + 1);
      a_entries.emplace_back(row, n, 1.0);
      lower.push_back(bound);
      upper.push_back(OSQP_INFTY);
    }
    row_scales.push_back(scaling[original_row]);
  };
  for (int row = 0; row < eval.values.size(); ++row) {
    if (elastic) {
      if (std::isfinite(eval.lower[row])) add_row(row, (eval.lower[row] - eval.values[row]) / scaling[row], false);
      if (std::isfinite(eval.upper[row])) add_row(row, (eval.upper[row] - eval.values[row]) / scaling[row], true);
    } else {
      row_ids.push_back(3 * coordinates.RowId(row));
      row_scales.push_back(scaling[row]);
      for (int col = 0; col < n; ++col) {
        double value = constraint_matrix(row, col) / scaling[row];
        if (value != 0) a_entries.emplace_back(lower.size(), col, value);
      }
      lower.push_back(std::max(-OSQP_INFTY, (eval.lower[row] - eval.values[row]) / scaling[row]));
      upper.push_back(std::min(OSQP_INFTY, (eval.upper[row] - eval.values[row]) / scaling[row]));
    }
  }
  for (int col = 0; col <= n; ++col) {
    if (col == n) {
      a_entries.emplace_back(lower.size(), col, 1.0);
      lower.push_back(0.0);
      double slack_limit = 0.0;
      if (elastic) slack_limit = OSQP_INFTY;
      upper.push_back(slack_limit);
    } else {
      for (int j = 0; j < n; ++j) {
        if (variable_transform(col, j) != 0) a_entries.emplace_back(lower.size(), j, variable_transform(col, j));
      }
      lower.push_back(-0.5);
      upper.push_back(0.5);
    }
  }
  QpSparse p(n + 1, n + 1), a(lower.size(), n + 1);
  p.setFromTriplets(p_entries.begin(), p_entries.end());
  a.setFromTriplets(a_entries.begin(), a_entries.end());
  p.makeCompressed();
  a.makeCompressed();
  csc p_csc{p.nonZeros(), p.rows(), p.cols(), p.outerIndexPtr(), p.innerIndexPtr(), p.valuePtr(), -1};
  csc a_csc{a.nonZeros(), a.rows(), a.cols(), a.outerIndexPtr(), a.innerIndexPtr(), a.valuePtr(), -1};
  OSQPData data{};
  data.n = n + 1;
  data.m = lower.size();
  data.P = &p_csc;
  data.A = &a_csc;
  data.q = q.data();
  data.l = lower.data();
  data.u = upper.data();
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.adaptive_rho_interval = 50;
  settings.eps_abs = solver_settings.qp_eps_abs;
  settings.eps_rel = solver_settings.qp_eps_rel;
  settings.eps_prim_inf = 1e-10;
  settings.eps_dual_inf = 1e-10;
  settings.max_iter = solver_settings.qp_max_iterations;
  settings.time_limit = std::max(1e-5, remaining);
  if (reuse) settings.warm_start = 0;
  diagnostic.variables = data.n;
  diagnostic.rows = data.m;
  diagnostic.workspace_reason = "mode_disabled";
  if (reuse) {
    diagnostic.workspace_reason = "first_use";
    if (cache->workspace != nullptr) {
      diagnostic.workspace_reason = "dimension_changed";
      if (cache->a_pattern.rows == a.rows() && cache->a_pattern.cols == a.cols()) {
        diagnostic.workspace_reason = "sparsity_changed";
        if (cache->p_pattern.Matches(p) && cache->a_pattern.Matches(a)) {
          diagnostic.workspace_reused = true;
          diagnostic.workspace_reason = "compatible";
        }
      }
    }
  }
  preparation.Stop();
  ++profile->qp_calls;
  if (diagnostic.workspace_reused) {
    ++profile->qp_updates;
    double seconds = 0;
    int code = 0;
    {
      ScopedTimer timer(&seconds);
      // 새 q/l/u를 먼저 전달하고 P/A 갱신에서 전체 데이터를 다시 스케일링한다.
      code = osqp_update_lin_cost(cache->workspace, q.data());
      if (code == 0) code = osqp_update_bounds(cache->workspace, lower.data(), upper.data());
      if (code == 0) code = osqp_update_P_A(cache->workspace, p.valuePtr(), nullptr,
                                          p.nonZeros(), a.valuePtr(), nullptr, a.nonZeros());
      // rho 이력을 재사용 효과에 포함하지 않고 각 QP의 기본값으로 되돌린다.
      if (code == 0 && cache->workspace->settings->rho != settings.rho) {
        code = osqp_update_rho(cache->workspace, settings.rho);
      }
      if (code == 0) code = osqp_update_time_limit(cache->workspace, settings.time_limit);
      if (code == 0) code = osqp_update_warm_start(cache->workspace, 0);
    }
    diagnostic.update_return = code;
    diagnostic.update_seconds = seconds;
    profile->qp_update_seconds += seconds;
    if (code != 0) {
      diagnostic.workspace_reused = false;
      diagnostic.workspace_reason = "update_failed_rebuilt";
    }
  }
  if (!diagnostic.workspace_reused) {
    cache->ResetWorkspace();
    ++profile->qp_setups;
    double seconds = 0;
    {
      ScopedTimer timer(&seconds);
      diagnostic.setup_return = osqp_setup(&cache->workspace, &data, &settings);
    }
    diagnostic.setup_seconds = seconds;
    profile->qp_setup_seconds += seconds;
    if (*diagnostic.setup_return != 0 || cache->workspace == nullptr) {
      cache->ResetWorkspace();
      cache->has_solution = false;
      *status = "qp_setup_failed";
      diagnostic.status = *status;
      diagnostic.acceptance_reason = "setup_failed";
      return false;
    }
    cache->p_pattern.Assign(p);
    cache->a_pattern.Assign(a);
  }
  OSQPWorkspace* workspace = cache->workspace;
  diagnostic.warm_start_reason = "mode_disabled";
  if (warm) {
    diagnostic.warm_start_reason = "no_previous_solution";
    if (cache->has_solution && cache->target.size() == coordinates.origin.size()) {
      double seconds = 0;
      {
        ScopedTimer timer(&seconds);
        const Vector desired = cache->target - coordinates.origin;
        Vector reduced = desired;
        if (coordinates.basis != nullptr) {
          reduced = coordinates.basis->transpose() * (desired - *coordinates.particular);
        }
        Vector primal = Vector::Zero(n + 1);
        primal.head(n) = factor.matrixU() * reduced;
        if (elastic) primal[n] = std::max(0.0, cache->slack);
        Vector dual = Vector::Zero(data.m);
        for (size_t row = 0; row < row_ids.size(); ++row) {
          const auto found = cache->duals.find(row_ids[row]);
          if (found != cache->duals.end()) {
            dual[row] = found->second * row_scales[row];
            ++diagnostic.reused_duals;
          }
        }
        // 바뀌는 reduced 좌표의 박스 제약과 slack의 dual은 0으로 초기화한다.
        if (primal.allFinite() && dual.allFinite()) {
          diagnostic.warm_start_roundtrip_error =
              (variable_transform * primal.head(n) - reduced).lpNorm<Eigen::Infinity>();
          Vector projected = reduced;
          if (coordinates.basis != nullptr) {
            projected = *coordinates.particular + *coordinates.basis * reduced;
          }
          diagnostic.warm_start_projection_error = (projected - desired).lpNorm<Eigen::Infinity>();
          diagnostic.warm_start_return = osqp_warm_start(workspace, primal.data(), dual.data());
          diagnostic.warm_start_used = *diagnostic.warm_start_return == 0;
          diagnostic.warm_start_reason = "api_failed_cold_start";
          if (diagnostic.warm_start_used) {
            ++profile->qp_warm_starts;
            diagnostic.warm_start_reason = "mapped_previous_target";
          } else {
            osqp_update_warm_start(workspace, 0);
          }
        } else {
          diagnostic.warm_start_reason = "nonfinite_mapping_cold_start";
        }
      }
      diagnostic.warm_start_seconds = seconds;
      profile->qp_warm_start_seconds += seconds;
    }
  }
  double solve_seconds = 0.0;
  {
    ScopedTimer timer(&solve_seconds);
    diagnostic.solve_return = osqp_solve(workspace);
  }
  diagnostic.solve_seconds = solve_seconds;
  profile->qp_solve_seconds += solve_seconds;
  profile->qp_iterations += workspace->info->iter;
  profile->qp_iterations_per_solve.push_back(workspace->info->iter);
  const int code = workspace->info->status_val;
  diagnostic.status_code = code;
  diagnostic.status = workspace->info->status;
  diagnostic.iterations = workspace->info->iter;
  diagnostic.primal_residual = workspace->info->pri_res;
  diagnostic.dual_residual = workspace->info->dua_res;
  diagnostic.polish_status = workspace->info->status_polish;
  diagnostic.rho_updates = workspace->info->rho_updates;
  bool solved = code == OSQP_SOLVED;
  diagnostic.acceptance_reason = "status_or_residual_rejected";
  if (solved) diagnostic.acceptance_reason = "solved";
  if (code == OSQP_SOLVED_INACCURATE && workspace->info->pri_res < 1e-5) {
    solved = true;
    diagnostic.acceptance_reason = "inaccurate_primal_accepted";
  }
  // 기존 inexact SQP 방향 채택 기준을 유지한다. 최종 계획 검증은 별도다.
  if (code == OSQP_MAX_ITER_REACHED && workspace->info->pri_res < 1e-3 &&
      workspace->info->dua_res < 5e-3) {
    solved = true;
    diagnostic.acceptance_reason = "iteration_limit_residual_accepted";
  }
  std::ostringstream status_stream;
  status_stream << std::setprecision(17) << workspace->info->status
                << " (primal=" << workspace->info->pri_res
                << ", dual=" << workspace->info->dua_res << ")";
  *status = status_stream.str();
  if (solved) {
    *step = variable_transform * Eigen::Map<Vector>(workspace->solution->x, n);
    *slack = workspace->solution->x[n];
    solved = step->allFinite() && std::isfinite(*slack);
    if (!solved) diagnostic.acceptance_reason = "nonfinite_direction";
  }
  diagnostic.accepted = solved;
  if (warm) {
    ScopedTimer timer(&profile->qp_prepare_seconds);
    cache->has_solution = solved;
    if (solved) {
      if (coordinates.basis != nullptr) {
        cache->target = coordinates.origin + *coordinates.particular + *coordinates.basis * *step;
      } else {
        cache->target = coordinates.origin + *step;
      }
      cache->slack = *slack;
      cache->duals.clear();
      for (size_t row = 0; row < row_ids.size(); ++row) {
        const double multiplier = workspace->solution->y[row] / row_scales[row];
        if (std::isfinite(multiplier)) cache->duals[row_ids[row]] = multiplier;
      }
    }
  }
  if (!reuse) cache->ResetWorkspace();
  return solved;
}

// 초기 활성 후보만 풀되, 생략한 모든 행을 검사하여 위반한 제약을 추가한다.
// 이미 포함된 행의 OSQP 허용 오차·근사 해 채택 기준은 QpStep과 동일하다.
bool WorkingSetQpStep(const Evaluation& eval, const Vector& scaling, double penalty,
                      double remaining, Vector* step, double* slack, std::string* status,
                      Profile* profile, const Settings& settings, int sqp_iteration,
                      QpCache* cache, const QpCoordinates& coordinates,
                      const Matrix* constraint_basis = nullptr) {
  if (!settings.qp_working_set) {
    return QpStep(eval, scaling, penalty, remaining, step, slack, status,
                  profile, settings, sqp_iteration, cache, coordinates);
  }
  const auto begin = Clock::now();
  const int count = eval.values.size();
  std::vector<bool> selected(count, false);
  for (int row = 0; row < count; ++row) {
    const double distance = std::min(eval.values[row] - eval.lower[row],
                                     eval.upper[row] - eval.values[row]) / scaling[row];
    selected[row] = distance <= 1e-4;
  }
  for (int pass = 0; pass < 9; ++pass) {
    if (pass == 8) std::fill(selected.begin(), selected.end(), true);
    const int active = std::count(selected.begin(), selected.end(), true);
    Evaluation restricted;
    restricted.gradient = eval.gradient;
    restricted.hessian = eval.hessian;
    restricted.values.resize(active);
    restricted.lower.resize(active);
    restricted.upper.resize(active);
    restricted.jacobian.resize(active, eval.gradient.size());
    Vector scales(active);
    std::vector<int> ids;
    int destination = 0;
    for (int row = 0; row < count; ++row) {
      if (!selected[row]) continue;
      restricted.values[destination] = eval.values[row];
      restricted.lower[destination] = eval.lower[row];
      restricted.upper[destination] = eval.upper[row];
      if (constraint_basis != nullptr) {
        restricted.jacobian.row(destination).noalias() = eval.jacobian.row(row) * *constraint_basis;
      } else {
        restricted.jacobian.row(destination) = eval.jacobian.row(row);
      }
      scales[destination] = scaling[row];
      ids.push_back(coordinates.RowId(row));
      ++destination;
    }
    const double budget = remaining - std::chrono::duration<double>(Clock::now() - begin).count();
    if (budget <= 0) { *status = "working_set_time_limit"; return false; }
    if (!QpStep(restricted, scales, penalty, budget, step, slack, status,
                profile, settings, sqp_iteration, cache,
                QpCoordinates{coordinates.origin, coordinates.basis, coordinates.particular, &ids})) {
      return false;
    }
    auto& diagnostic = profile->qp_diagnostics.back();
    diagnostic.working_set_pass = pass + 1;
    diagnostic.working_set_total_rows = count;
    Vector direction = *step;
    if (constraint_basis != nullptr) direction.noalias() = *constraint_basis * *step;
    const Vector values = eval.values + eval.LinearChange(direction);
    double omitted_violation = 0.0, full_violation = 0.0;
    int added = 0;
    for (int row = 0; row < count; ++row) {
      const double violation = std::max({0.0, eval.lower[row] - values[row],
                                         values[row] - eval.upper[row]}) / scaling[row];
      full_violation = std::max(full_violation, violation);
      if (!selected[row]) {
        omitted_violation = std::max(omitted_violation, violation);
        if (violation > 1e-9) { selected[row] = true; ++added; }
      }
    }
    diagnostic.working_set_omitted_violation = omitted_violation;
    diagnostic.working_set_full_violation = full_violation;
    if (added == 0) return true;
    diagnostic.accepted = false;
    diagnostic.acceptance_reason = "working_set_expanded";
  }
  *status = "working_set_internal_error";
  return false;
}

// 중복된 두 접점 등식은 SVD로 분리하고 OSQP는 영공간의 QP를 푼다.
// 버린 특이 방향의 오차도 원래 비선형 제약 검증에서는 그대로 검사한다.
bool ReducedQpStep(const Evaluation& eval, const Vector& scaling,
                   double penalty, double remaining, Vector* step,
                   double* slack, std::string* status, Profile* profile,
                   const Settings& settings, int sqp_iteration,
                   QpCache* cache, const Vector& origin, StructureCache* structure_cache) {
  std::vector<int> equality, inequality;
  for (int row = 0; row < eval.values.size(); ++row) {
    if (eval.lower[row] == eval.upper[row]) equality.push_back(row);
    else inequality.push_back(row);
  }
  if (equality.empty()) {
    return WorkingSetQpStep(eval, scaling, penalty, remaining, step, slack, status, profile,
                            settings, sqp_iteration, cache, QpCoordinates{origin});
  }
  Matrix a(equality.size(), eval.gradient.size());
  Vector b(equality.size());
  for (int row = 0; row < static_cast<int>(equality.size()); ++row) {
    const int i = equality[row];
    a.row(row) = eval.jacobian.row(i) / scaling[i];
    b[row] = (eval.lower[i] - eval.values[i]) / scaling[i];
  }
  Vector particular;
  Matrix nullspace;
  if (settings.structured_equalities) {
    const auto solution = structure_cache->Solve(eval, equality, scaling,
        settings.equality_rank_tolerance, settings.lapack_svd, true, profile);
    particular = solution.particular;
    nullspace = solution.nullspace;
  } else {
    ScopedTimer svd_timer(&profile->svd_seconds);
    ++profile->svd_calls;
    DenseSvd svd(a, settings.equality_rank_tolerance, true, settings.lapack_svd);
    const int rank = svd.rank();
    particular = svd.Solve(b);
    nullspace = svd.v().rightCols(a.cols() - rank);
  }
  if (nullspace.cols() == 0) {
    *step = particular;
    *slack = 0;
    *status = "equality_only";
    return step->allFinite();
  }
  Evaluation reduced;
  reduced.gradient = nullspace.transpose() * (eval.gradient + eval.hessian * particular);
  reduced.hessian = nullspace.transpose() * eval.hessian * nullspace;
  reduced.values.resize(inequality.size());
  reduced.lower.resize(inequality.size());
  reduced.upper.resize(inequality.size());
  const bool lazy_projection = settings.block_assembly && settings.qp_working_set;
  int jacobian_columns = nullspace.cols();
  if (lazy_projection) jacobian_columns = eval.jacobian.cols();
  reduced.jacobian.resize(inequality.size(), jacobian_columns);
  Vector reduced_scale(inequality.size());
  Matrix projected_jacobian;
  Vector particular_values;
  if (settings.block_assembly) {
    // 행별 strided GEMV 대신 전역 GEMM으로 같은 영공간 변환을 계산한다.
    if (!lazy_projection) projected_jacobian.noalias() = eval.jacobian * nullspace;
    particular_values.noalias() = eval.jacobian * particular;
  }
  for (int row = 0; row < static_cast<int>(inequality.size()); ++row) {
    const int i = inequality[row];
    if (settings.block_assembly) {
      reduced.values[row] = eval.values[i] + particular_values[i];
      if (lazy_projection) reduced.jacobian.row(row) = eval.jacobian.row(i);
      else reduced.jacobian.row(row) = projected_jacobian.row(i);
    } else {
      reduced.values[row] = eval.values[i] + eval.jacobian.row(i).dot(particular);
      reduced.jacobian.row(row) = eval.jacobian.row(i) * nullspace;
    }
    reduced.lower[row] = eval.lower[i];
    reduced.upper[row] = eval.upper[i];
    reduced_scale[row] = scaling[i];
  }
  const Matrix* constraint_basis = nullptr;
  if (lazy_projection) constraint_basis = &nullspace;
  Vector reduced_step;
  bool ok = WorkingSetQpStep(reduced, reduced_scale, penalty, remaining,
                   &reduced_step, slack, status, profile, settings, sqp_iteration,
                   cache, QpCoordinates{origin, &nullspace, &particular, &inequality}, constraint_basis);
  if (ok) *step = particular + nullspace * reduced_step;
  return ok;
}

struct ChordProjectionCache {
  std::vector<int> rows;
  Matrix inverse;
};

Vector ProjectConstraints(const Problem& problem, Vector x, Profile* profile,
                          const Settings& settings, StructureCache* structure_cache, int intervals = 0,
                          const Evaluation* linearization = nullptr,
                          ChordProjectionCache* chord_cache = nullptr) {
  ScopedTimer projection_timer(&profile->projection_seconds);
  for (int iteration = 0; iteration < 8; ++iteration) {
    EvaluationMode mode = EvaluationMode::kConstraintJacobian;
    if (linearization != nullptr) mode = EvaluationMode::kValues;
    const Evaluation eval = problem.EvaluateAt(x, mode, intervals);
    const Evaluation* derivatives = &eval;
    if (linearization != nullptr) derivatives = linearization;
    std::vector<int> indices;
    for (int row = 0; row < eval.values.size(); ++row) {
      if (eval.lower[row] == eval.upper[row] || eval.values[row] < eval.lower[row] ||
          eval.values[row] > eval.upper[row]) indices.push_back(row);
    }
    if (indices.empty()) break;
    Matrix a(indices.size(), x.size());
    Vector b(indices.size());
    for (int row = 0; row < static_cast<int>(indices.size()); ++row) {
      const int i = indices[row];
      a.row(row) = derivatives->JacobianRow(i);
      b[row] = std::clamp(eval.values[i], eval.lower[i], eval.upper[i]) - eval.values[i];
    }
    if (b.lpNorm<Eigen::Infinity>() < settings.projection_residual_tolerance) break;
    Vector correction;
    if (linearization != nullptr && chord_cache != nullptr) {
      if (indices != chord_cache->rows || chord_cache->inverse.rows() != x.size()) {
        chord_cache->rows = indices;
        if (settings.structured_equalities) {
          chord_cache->inverse = structure_cache->Solve(*linearization, indices,
              Vector::Ones(eval.values.size()), settings.projection_rank_tolerance,
              settings.lapack_svd, false, profile, true).inverse;
        } else {
          ScopedTimer timer(&profile->svd_seconds);
          ++profile->svd_calls;
          DenseSvd svd(a, settings.projection_rank_tolerance, false, settings.lapack_svd);
          chord_cache->inverse = svd.PseudoInverse();
        }
      }
      correction = chord_cache->inverse * b;
    } else if (settings.structured_equalities) {
      correction = structure_cache->Solve(eval, indices, Vector::Ones(eval.values.size()),
          settings.projection_rank_tolerance, settings.lapack_svd, false, profile).particular;
    } else {
      ScopedTimer svd_timer(&profile->svd_seconds);
      ++profile->svd_calls;
      DenseSvd svd(a, settings.projection_rank_tolerance, false, settings.lapack_svd);
      correction = svd.Solve(b);
    }
    double alpha = std::min(1.0, 0.05 / std::max(1e-12, correction.lpNorm<Eigen::Infinity>()));
    bool accepted = false;
    for (int search = 0; search < 8; ++search) {
      try {
        Evaluation trial = problem.EvaluateAt(x + alpha * correction, EvaluationMode::kValues, intervals);
        double error = Violation(trial, Vector::Ones(trial.values.size()));
        double current_error = b.lpNorm<Eigen::Infinity>();
        if (settings.projection_l2) {
          error = 0;
          for (int row = 0; row < trial.values.size(); ++row) {
            const double residual = std::max({0.0, trial.lower[row] - trial.values[row],
                                              trial.values[row] - trial.upper[row]});
            error += residual * residual;
          }
          current_error = b.squaredNorm();
        }
        if (error < current_error) {
          x += alpha * correction;
          accepted = true;
          break;
        }
      } catch (const TerrainDomainError&) {
        // 맵 밖/미관측 보정 후보는 버리고 더 짧은 보정량을 검사한다.
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return x;
}

void Validate(const Robot& r, const Input& input, const Settings& settings) {
  if (!std::isfinite(settings.foothold_backward_weight) || settings.foothold_backward_weight < 0) {
    throw std::invalid_argument("invalid foothold backward weight");
  }
  if (!std::isfinite(settings.orientation_limit) || settings.orientation_limit <= 0 ||
      settings.orientation_limit >= std::acos(-1.0) / 2.0) {
    throw std::invalid_argument("invalid orientation limit");
  }
  if (!std::isfinite(settings.pitch_weight) || settings.pitch_weight < 0 ||
      !std::isfinite(settings.leg_vertical_weight) || settings.leg_vertical_weight < 0 ||
      !std::isfinite(settings.vz_weight) || settings.vz_weight < 0) {
    throw std::invalid_argument("invalid posture cost weight");
  }
  if (!std::isfinite(settings.qp_regularization) || settings.qp_regularization <= 0)
    throw std::invalid_argument("invalid QP regularization");
  if (!std::isfinite(settings.projection_rank_tolerance) || settings.projection_rank_tolerance <= 0 ||
      settings.projection_rank_tolerance >= 1) throw std::invalid_argument("invalid projection rank tolerance");
  if (!std::isfinite(settings.equality_rank_tolerance) || settings.equality_rank_tolerance <= 0 ||
      settings.equality_rank_tolerance >= 1) throw std::invalid_argument("invalid equality rank tolerance");
  if (QpReuseModeName(settings.qp_reuse) == "invalid") {
    throw std::invalid_argument("invalid QP reuse mode");
  }
  if (r.model && (!r.reach_min.allFinite() || !r.reach_max.allFinite() ||
      (r.reach_min.array() < 0).any() ||
      (r.reach_max.array() <= r.reach_min.array()).any())) {
    throw std::invalid_argument("invalid joint-limited leg reach");
  }
  if (!std::isfinite(r.mass + r.height + r.friction + r.leg_min + r.leg_max + r.period + r.duty) ||
      std::min({r.mass, r.height, r.friction, r.period}) <= 0 || r.leg_min < 0 ||
      r.leg_max <= r.leg_min || r.duty <= 0.5 || r.duty >= 1.0 ||
      !r.hips.allFinite() || !r.nominal_offsets.allFinite() ||
      !r.reference_legs.allFinite() || !r.com_offset.allFinite() ||
      !r.reference_joints.allFinite() || r.use_reference_legs != static_cast<bool>(r.model) ||
      !r.inertia.allFinite() || !r.inertia.isApprox(r.inertia.transpose()) ||
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(r.inertia).eigenvalues().minCoeff() <= 0) {
    throw std::invalid_argument("invalid robot constants");
  }
  if (!input.position.allFinite() || !input.velocity.allFinite() || !input.feet.allFinite() ||
      !input.command.allFinite() || !std::isfinite(input.gait_time) ||
      std::abs(input.gait_time / r.period) > 1e6 ||
      (input.has_nominal && !input.nominal.allFinite())) throw std::invalid_argument("invalid planner input");
  if (!input.terrain.has_value() &&
      input.feet.col(2).maxCoeff() - input.feet.col(2).minCoeff() > 0.015) {
    throw std::invalid_argument("flat planner rejects non-flat support");
  }
  if (!std::isfinite(settings.qp_eps_abs) || settings.qp_eps_abs <= 0 ||
      !std::isfinite(settings.qp_eps_rel) || settings.qp_eps_rel <= 0 ||
      settings.realtime_iterations < 0 || settings.realtime_iterations > settings.max_iterations ||
      settings.qp_max_iterations <= 0 || settings.max_iterations <= 0 || settings.sample_intervals < 2 || settings.sample_intervals > 100 ||
      !std::isfinite(settings.time_limit) || settings.time_limit <= 0 ||
      !std::isfinite(settings.projection_residual_tolerance) ||
      settings.projection_residual_tolerance <= 0 || settings.projection_residual_tolerance > 1e-5 ||
      !std::isfinite(settings.feasibility_tolerance) || settings.feasibility_tolerance <= 0 ||
      !std::isfinite(settings.step_tolerance) || settings.step_tolerance <= 0) {
    throw std::invalid_argument("invalid solver settings");
  }
}
}  // namespace

Robot::Robot() {
  hips << 0.35, 0.2, 0.0, 0.35, -0.2, 0.0,
          -0.35, 0.2, 0.0, -0.35, -0.2, 0.0;
  nominal_offsets = hips;
}

std::string QpReuseModeName(QpReuseMode mode) {
  switch (mode) {
    case QpReuseMode::kNone: return "cold";
    case QpReuseMode::kWorkspace: return "reuse";
    case QpReuseMode::kWarmStart: return "warm";
  }
  return "invalid";
}

std::vector<Phase> MakeHorizon(double time, double period, double duty) {
  if (!std::isfinite(time) || !std::isfinite(period) || !std::isfinite(duty) ||
      period <= 0 || duty <= 0.5 || duty >= 1 || std::abs(time / period) > 1e6) {
    throw std::invalid_argument("invalid gait timing");
  }
  std::set<double> events{time, time + period};
  std::vector<std::pair<double, int>> touchdowns;
  const std::array<double, 4> offsets{0, 0.5, 0.5, 0};
  for (int leg = 0; leg < 4; ++leg) {
    for (int cycle = static_cast<int>(std::floor(time / period)) - 1;
         cycle < static_cast<int>(std::ceil((time + period) / period)) + 2; ++cycle) {
      const double touchdown = (cycle - offsets[leg]) * period;
      const double liftoff = (cycle + duty - offsets[leg]) * period;
      for (double event : {touchdown, liftoff}) {
        if (event > time + 1e-9 && event < time + period - 1e-9) events.insert(std::round(event * 1e12) / 1e12);
      }
      if (touchdown > time + 1e-9) touchdowns.emplace_back(touchdown, leg);
    }
  }
  std::vector<double> times(events.begin(), events.end());
  std::vector<Phase> result;
  for (size_t k = 0; k + 1 < times.size(); ++k) {
    Phase phase;
    phase.start = times[k] - time;
    phase.duration = times[k + 1] - times[k];
    const double middle = 0.5 * (times[k] + times[k + 1]);
    phase.contacts = Contacts(middle, period, duty);
    for (const auto& touchdown : touchdowns) if (touchdown.first <= middle) phase.landed[touchdown.second] = true;
    result.push_back(phase);
  }
  return result;
}

std::string RobotSignature(const Robot& robot) {
  std::ostringstream stream;
  stream << std::setprecision(17);
  if (robot.model) stream << robot.model->path();
  else stream << "synthetic";
  stream << ' ' << robot.mass << ' ' << robot.height << ' ' << robot.friction
         << ' ' << robot.leg_min << ' ' << robot.leg_max << ' ' << robot.use_reference_legs;
  if (robot.model) {
    for (int leg = 0; leg < 4; ++leg) stream << ' ' << robot.reach_min[leg] << ' ' << robot.reach_max[leg];
  }
  for (const auto* feet : {&robot.hips, &robot.nominal_offsets, &robot.reference_legs}) {
    for (int k = 0; k < feet->size(); ++k) stream << ' ' << feet->data()[k];
  }
  for (int k = 0; k < 9; ++k) stream << ' ' << robot.inertia.data()[k];
  for (int k = 0; k < 3; ++k) stream << ' ' << robot.com_offset[k];
  for (int k = 0; k < 12; ++k) stream << ' ' << robot.reference_joints[k];
  return stream.str();
}

std::string TerrainSignature(const std::optional<HeightMap>& terrain) {
  if (!terrain.has_value()) return "";
  // FNV-1a는 로컬 정적 맵의 변경 감지용이며 입력 유효성 검증을 대체하지 않는다.
  std::uint64_t hash = 14695981039346656037ULL;
  auto append = [&](const void* data, size_t count) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < count; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ULL;
    }
  };
  const auto& map = *terrain;
  append(map.frame.data(), map.frame.size());
  append(&map.resolution, sizeof(map.resolution));
  append(map.origin.data(), 2 * sizeof(double));
  for (const auto* matrix : {&map.heights, &map.variance}) {
    const std::int64_t shape[] = {matrix->rows(), matrix->cols()};
    append(shape, sizeof(shape));
    append(matrix->data(), matrix->size() * sizeof(double));
  }
  const std::int64_t shape[] = {map.valid.rows(), map.valid.cols()};
  append(shape, sizeof(shape));
  append(map.valid.data(), map.valid.size() * sizeof(int));
  std::ostringstream stream;
  stream << "fnv1a64:" << std::hex << hash;
  return stream.str();
}

static Result SolveAttempt(const Robot& robot, const Input& input, const Settings& settings,
                           const Result* previous, const TerrainAttempt* alternative,
                           Feet* initial_feet) {
  const auto begin = Clock::now();
  auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - begin).count(); };
  Result result;
  result.seed_context = {input.gait_time, robot.period, robot.duty, input.frame,
                         RobotSignature(robot), input.terrain.has_value(), input.feet,
                         TerrainSignature(input.terrain)};
  try {
    ScopedTimer initialization_timer(&result.profile.initialization_seconds);
    Validate(robot, input, settings);
    QpCache reduced_cache(&result.profile);
    QpCache elastic_cache(&result.profile);
    StructureCache structure_cache;
    std::optional<TerrainModel> terrain;
    const TerrainModel* terrain_pointer = nullptr;
    if (input.terrain.has_value()) {
      terrain.emplace(*input.terrain, settings.terrain);
      terrain_pointer = &*terrain;
    }
    Problem problem(robot, input, settings.sample_intervals,
                    &result.profile, settings.full_evaluation, terrain_pointer,
                    settings.block_assembly, settings.polynomial_tail,
                    settings.pitch_weight, settings.leg_vertical_weight, settings.orientation_limit,
                    settings.foothold_backward_weight, settings.vz_weight);
    int replaced_leg = -1;
    if (alternative != nullptr) replaced_leg = alternative->leg;
    Vector x = problem.Initial(previous, &result.warm_start, replaced_leg);
    if (alternative != nullptr) problem.ReplaceFootholdSeed(alternative->leg, alternative->seed, &x);
    if (initial_feet != nullptr && input.terrain.has_value()) {
      *initial_feet = problem.InitialFootholds(x);
    }
    initialization_timer.Stop();
    const bool realtime = settings.realtime_iterations > 0 && result.warm_start.used;
    double penalty = 10.0;
    int small_improvements = 0;
    result.status = "iteration_limit";
    for (int iteration = 0; iteration < settings.max_iterations; ++iteration) {
      result.iterations = iteration + 1;
      Evaluation eval = problem.EvaluateAt(x);
      if (elapsed() >= settings.time_limit) {
        result.status = "time_limit";
        break;
      }
      Vector scaling = Vector::Ones(eval.values.size());
      Vector variable_scale = eval.hessian.diagonal().cwiseMax(1.0).cwiseSqrt().cwiseInverse();
      if (settings.block_assembly) {
        // Column-major 저장 순서로 같은 행별 최대값을 누적한다.
        for (int col = 0; col < eval.jacobian.cols(); ++col) {
          scaling = scaling.cwiseMax((eval.jacobian.col(col) * variable_scale[col]).cwiseAbs());
        }
      } else {
        for (int row = 0; row < scaling.size(); ++row) scaling[row] = std::max(1.0,
            eval.jacobian.row(row).cwiseProduct(variable_scale.transpose()).cwiseAbs().maxCoeff());
      }
      Vector step;
      double slack = 0.0;
      std::string qp_status;
      bool qp_ok = ReducedQpStep(
          eval, scaling, penalty, settings.time_limit - elapsed(), &step,
          &slack, &qp_status, &result.profile, settings, iteration + 1, &reduced_cache, x, &structure_cache);
      if (!qp_ok && elapsed() < settings.time_limit) {
        qp_ok = QpStep(eval, scaling, penalty, settings.time_limit - elapsed(),
                       &step, &slack, &qp_status, &result.profile, settings, iteration + 1,
                       &elastic_cache, QpCoordinates{x}, true);
      }
      if (!qp_ok) {
        result.status = "qp_failed: " + qp_status;
        break;
      }
      const double violation = Violation(eval, Vector::Ones(eval.values.size()));
      const double predicted_change = eval.gradient.dot(step) +
          0.5 * step.dot(eval.hessian * step);
      const bool small_model_change = step.lpNorm<Eigen::Infinity>() < 1e-3 &&
          std::abs(predicted_change) < 1e-8;
      if (violation < settings.feasibility_tolerance &&
          (step.lpNorm<Eigen::Infinity>() < settings.step_tolerance || small_model_change)) {
        result.success = true;
        result.status = "converged";
        break;
      }
      const double current_violation = Violation(eval, scaling);
      const double current_merit = eval.cost + penalty * current_violation;
      const double slope = eval.gradient.dot(step) + penalty * (slack - current_violation);
      bool accepted = false;
      double accepted_violation = std::numeric_limits<double>::infinity();
      double alpha = 1.0;
      ChordProjectionCache chord_cache;
      const Evaluation* projection_linearization = nullptr;
      if (settings.chord_projection) projection_linearization = &eval;
      ScopedTimer search_timer(&result.profile.line_search_seconds);
      for (int search = 0; search < 10; ++search) {
        if (elapsed() >= settings.time_limit) break;
        Vector trial_x;
        Evaluation candidate;
        try {
          trial_x = ProjectConstraints(problem, x + alpha * step, &result.profile, settings,
                                       &structure_cache, 0, projection_linearization, &chord_cache);
          candidate = problem.EvaluateAt(trial_x, EvaluationMode::kValues);
        } catch (const TerrainDomainError&) {
          alpha *= 0.5;
          continue;
        }
        const double merit = candidate.cost + penalty * Violation(candidate, scaling);
        if (merit <= current_merit + 1e-4 * alpha * std::min(-1e-12, slope)) {
          const double improvement = eval.cost - candidate.cost;
          const double candidate_violation = Violation(candidate, Vector::Ones(candidate.values.size()));
          if (improvement >= 0.0 && improvement < 1e-6 && candidate_violation < 1e-5) {
            ++small_improvements;
          } else {
            small_improvements = 0;
          }
          x = trial_x;
          accepted_violation = candidate_violation;
          accepted = true;
          break;
        }
        alpha *= 0.5;
      }
      search_timer.Stop();
      if (realtime && accepted && iteration + 1 >= settings.realtime_iterations &&
          accepted_violation <= settings.feasibility_tolerance && elapsed() < settings.time_limit) {
        result.success = true;
        result.status = "feasible_realtime_step";
        break;
      }
      if (((!accepted && violation < 1e-5) || small_improvements >= 3) && elapsed() < settings.time_limit) {
        result.success = true;
        result.status = "feasible_stalled";
        break;
      }
      if (!accepted || slack > 1e-4 ||
          (step.lpNorm<Eigen::Infinity>() < 1e-3 && violation > settings.feasibility_tolerance)) {
        penalty = std::min(1e6, penalty * 10);
      }
      if (!accepted && penalty >= 1e6) {
        result.status = "line_search_failed";
        break;
      }
    }
    ScopedTimer validation_timer(&result.profile.final_validation_seconds);
    Evaluation final = problem.EvaluateAt(x, EvaluationMode::kValues);
    result.cost = final.cost;
    result.violation = Violation(final, Vector::Ones(final.values.size()));
    problem.Describe(x, final, &result);
    if (elapsed() > settings.time_limit) {
      result.success = false;
      result.status = "time_limit";
    }
    if (terrain_pointer != nullptr) {
      result.terrain_validation = problem.CheckTerrain(x);
      if (result.success && !result.terrain_validation->passed) {
        result.success = false;
        result.status = "terrain_validation_failed: " + result.terrain_validation->reason;
      }
    }
    if (elapsed() > settings.time_limit) {
      result.success = false;
      result.status = "time_limit";
    }
    if (result.success && robot.model) {
      ScopedTimer ik_timer(&result.profile.ik_seconds);
      problem.CheckKinematics(x, &result);
      if (!result.kinematics_valid.value()) {
        result.success = false;
        result.status = "kinematics_validation_failed";
      }
    }
    if (elapsed() > settings.time_limit) {
      result.success = false;
      result.status = "time_limit";
    }
    if (result.success) problem.Export(x, &result);
  } catch (const std::exception& error) {
    result.success = false;
    result.coefficients.clear();
    result.phases.clear();
    result.margins.clear();
    result.footholds.setZero();
    result.status = error.what();
  }
  result.elapsed = elapsed();
  return result;
}

Result Solve(const Robot& robot, const Input& input, const Settings& settings,
             const Result* previous) {
  const auto begin = Clock::now();
  auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - begin).count(); };
  Feet initial = input.feet;
  Result result = SolveAttempt(robot, input, settings, previous, nullptr, &initial);
  if (!input.terrain.has_value() || !robot.model) return result;
  std::vector<TerrainAttempt> attempts{{-1, Eigen::Vector3d::Zero(), result.status,
                                      result.elapsed, result.success}};
  const bool retryable = result.status == "kinematics_validation_failed" ||
      result.status == "line_search_failed" || result.status == "iteration_limit";
  if (!result.success && retryable && elapsed() < settings.time_limit) {
    // 같은 nominal과 목적함수를 유지하고, 각 다리의 다른 높이 지지면에서 시작한다.
    // 후보별로 몸통 SQP와 최종 지형·IK 검사를 전부 다시 수행한다.
    TerrainModel terrain(*input.terrain, settings.terrain);
    const auto nominal = NominalFootholds(robot, input,
        MakeHorizon(input.gait_time, robot.period, robot.duty));
    Eigen::Vector3d command(input.command.x(), input.command.y(), 0);
    command = Eigen::AngleAxisd(input.position[3], Eigen::Vector3d::UnitZ()) * command;
    Eigen::Vector2d direction = Eigen::Vector2d::Zero();
    if (command.head<2>().norm() > 1e-6) direction = command.head<2>().normalized();
    std::vector<std::pair<double, TerrainAttempt>> alternatives;
    for (int leg = 0; leg < 4; ++leg) {
      if (input.fixed[leg]) continue;
      const Eigen::Vector2d center = nominal.row(leg).transpose();
      const auto other_platform = [&](const Eigen::Vector3d& foot) {
        // 다른 높이 후보를 구분하는 값이며 주변 평탄성 판정이 아니다.
        constexpr double kDifferentPlatformHeightTolerance = 0.005;
        return std::abs(foot.z() - initial(leg, 2)) > kDifferentPlatformHeightTolerance &&
            direction.dot(foot.head<2>() - input.feet.row(leg).head<2>().transpose()) >= -1e-10;
      };
      try {
        TerrainAttempt candidate;
        candidate.leg = leg;
        candidate.seed = terrain.FindFoothold(center, center.array() - .30,
            center.array() + .30, direction, settings.foothold_backward_weight, other_platform);
        const Eigen::Vector3d hip = input.position.head<3>() +
            Rotation(input.position) * robot.hips.row(leg).transpose();
        double priority = (hip - initial.row(leg).transpose()).norm() / robot.reach_max[leg];
        if (result.ik_failed_leg == leg) priority += 1;
        alternatives.emplace_back(priority, candidate);
      } catch (const TerrainDomainError&) {
        // 이 다리의 탐색 범위에 다른 지지면이 없으면 시도하지 않는다.
      }
    }
    std::stable_sort(alternatives.begin(), alternatives.end(),
                     [](const auto& a, const auto& b) { return a.first > b.first; });
    for (auto& entry : alternatives) {
      Settings retry = settings;
      retry.time_limit = settings.time_limit - elapsed();
      if (retry.time_limit <= 0) break;
      // 지지면을 바꾸면 선형화도 크게 바뀌므로 5회 RTI 조기 종료를 기다리지 않는다.
      if (retry.realtime_iterations > 0) retry.realtime_iterations = std::max(20, retry.realtime_iterations);
      auto trial = SolveAttempt(robot, input, retry, previous, &entry.second, nullptr);
      entry.second.status = trial.status;
      entry.second.elapsed = trial.elapsed;
      entry.second.success = trial.success;
      attempts.push_back(entry.second);
      if (trial.success) { result = std::move(trial); break; }
    }
  }
  result.terrain_attempts = std::move(attempts);
  result.elapsed = elapsed();
  if (result.elapsed > settings.time_limit) {
    result.success = false;
    result.status = "time_limit";
    result.coefficients.clear();
    result.phases.clear();
    result.margins.clear();
    result.footholds.setZero();
  }
  return result;
}

Vector6 Evaluate(const Result& result, double elapsed, int derivative) {
  if (!result.success || result.phases.empty() ||
      result.coefficients.size() != result.phases.size() || !std::isfinite(elapsed) ||
      derivative < 0 || derivative > 2 || elapsed < 0) throw std::invalid_argument("invalid evaluation");
  double horizon = 0.0;
  for (const auto& phase : result.phases) horizon += phase.duration;
  const double rounding = 16 * std::numeric_limits<double>::epsilon() * std::max(1.0, horizon);
  if (elapsed > horizon + rounding) throw std::out_of_range("trajectory extrapolation is forbidden");
  elapsed = std::min(elapsed, horizon);
  double begin = 0.0;
  for (size_t k = 0; k < result.phases.size(); ++k) {
    const double end = begin + result.phases[k].duration;
    if (elapsed <= end) return result.coefficients[k] * Basis(elapsed - begin, 1.0, derivative);
    begin = end;
  }
  throw std::out_of_range("trajectory extrapolation is forbidden");
}
}  // namespace b2_terrain_planner
