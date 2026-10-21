/**
 * @file robot_model.cpp
 * @brief RBDL의 이름 기반 관절 매핑·합성 관성·FK·제한 IK를 계산한다.
 * @version 0.0.4
 * @update 2026-10-21: 측정 관절값의 제한 경계 오버슈트는 허용하고 큰 이상값은 거부함.
 * @update 2026-10-14: URDF 무릎 제한으로 다리별 최소·최대 고관절 거리를 계산함.
 */
#include "b2_terrain_planner/robot_model.h"

#include "b2_terrain_planner/planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <tinyxml2.h>

namespace b2_terrain_planner {
namespace {
namespace rb = RigidBodyDynamics;
const std::array<std::string, 4> kLegs{"FL", "FR", "RL", "RR"};
const std::array<std::string, 3> kJoints{"hip", "thigh", "calf"};
constexpr double kMeasuredJointLimitToleranceRad = 0.05;

Eigen::Vector3d EulerRates(const Eigen::Vector3d& angles,
                           const Eigen::Vector3d& omega) {
  const double pitch = angles[1], roll = angles[2];
  if (std::abs(std::cos(pitch)) < .1) throw std::invalid_argument("Euler singularity");
  Eigen::Matrix3d transform;
  transform << -std::sin(pitch), 0, 1,
      std::sin(roll) * std::cos(pitch), std::cos(roll), 0,
      std::cos(roll) * std::cos(pitch), -std::sin(roll), 0;
  return transform.inverse() * omega;
}
}  // namespace

struct RobotModel::Impl {
  struct LegChain {
    std::array<Eigen::Isometry3d, 4> origins;
    std::array<Eigen::Vector3d, 3> axes;

    Eigen::Vector3d Point(const Eigen::Vector3d& joints,
                          Eigen::Matrix3d* jacobian = nullptr) const {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      std::array<Eigen::Vector3d, 3> centers, directions;
      for (int i = 0; i < 3; ++i) {
        pose = pose * origins[i];
        if (jacobian != nullptr) {
          centers[i] = pose.translation();
          directions[i] = pose.linear() * axes[i];
        }
        pose.rotate(Eigen::AngleAxisd(joints[i], axes[i]));
      }
      const Eigen::Vector3d point = pose * origins[3].translation();
      if (jacobian != nullptr) {
        for (int i = 0; i < 3; ++i) {
          jacobian->col(i) = directions[i].cross(point - centers[i]);
        }
      }
      return point;
    }
  };
  std::array<LegChain, 4> chains;
  rb::Model model;
  std::string path;
  std::array<unsigned int, 12> indices{};
  std::array<unsigned int, 4> feet{}, hips{};
  unsigned int base = 0;
  JointVector lower, upper;
  Eigen::Vector4d reach_min, reach_max;

  Eigen::VectorXd Positions(const ModelState& state) const {
    if (!state.base_position.allFinite() || !state.base_orientation.coeffs().allFinite() ||
        std::abs(state.base_orientation.norm() - 1) > 1e-8 || !state.joints.allFinite() ||
        (state.joints.array() < lower.array() - kMeasuredJointLimitToleranceRad).any() ||
        (state.joints.array() > upper.array() + kMeasuredJointLimitToleranceRad).any()) {
      throw std::invalid_argument("invalid B2 model state or joint limits");
    }
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.q_size);
    q.head<3>() = state.base_position;
    const auto& orientation = state.base_orientation;
    model.SetQuaternion(base, rb::Math::Quaternion(orientation.x(), orientation.y(),
                                                  orientation.z(), orientation.w()), q);
    for (int i = 0; i < 12; ++i) q[indices[i]] = state.joints[i];
    return q;
  }
};

RobotModel::RobotModel(const std::string& urdf_path) : impl_(std::make_unique<Impl>()) {
  auto& p = *impl_;
  p.path = urdf_path;
  tinyxml2::XMLDocument xml;
  if (xml.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS ||
      xml.FirstChildElement("robot") == nullptr) {
    throw std::invalid_argument("cannot read B2 URDF: " + urdf_path);
  }
  if (!rb::Addons::URDFReadFromFile(urdf_path.c_str(), &p.model, true, false) ||
      p.model.q_size != 19 || p.model.qdot_size != 18) {
    throw std::invalid_argument("B2 requires a floating base and 12 joints");
  }
  p.model.gravity << 0, 0, -9.81;
  p.base = p.model.GetBodyId("base_link");
  if (!p.model.IsBodyId(p.base) || p.model.IsFixedBodyId(p.base) ||
      p.model.mJoints[p.base].mJointType != rb::JointTypeSpherical) {
    throw std::invalid_argument("missing floating base_link");
  }
  auto* root = xml.FirstChildElement("robot");
  for (int leg = 0; leg < 4; ++leg) {
    p.feet[leg] = p.model.GetBodyId((kLegs[leg] + "_foot").c_str());
    p.hips[leg] = p.model.GetBodyId((kLegs[leg] + "_hip").c_str());
    if (!p.model.IsBodyId(p.feet[leg]) || !p.model.IsBodyId(p.hips[leg])) {
      throw std::invalid_argument("missing B2 foot/hip link");
    }
    std::string parent_link = "base_link";
    for (int axis = 0; axis < 4; ++axis) {
      std::string suffix = "foot";
      if (axis < 3) suffix = kJoints[axis];
      const std::string name = kLegs[leg] + "_" + suffix + "_joint";
      const tinyxml2::XMLElement* element = nullptr;
      for (auto* joint = root->FirstChildElement("joint"); joint != nullptr;
           joint = joint->NextSiblingElement("joint")) {
        if (joint->Attribute("name") != nullptr && name == joint->Attribute("name")) {
          element = joint;
          break;
        }
      }
      if (element == nullptr) throw std::invalid_argument("missing joint " + name);
      const auto* parent = element->FirstChildElement("parent");
      const auto* chain_child = element->FirstChildElement("child");
      const char* type = element->Attribute("type");
      std::string expected_type = "revolute";
      if (axis == 3) expected_type = "fixed";
      if (parent == nullptr || parent->Attribute("link") == nullptr ||
          parent_link != parent->Attribute("link") || chain_child == nullptr ||
          chain_child->Attribute("link") == nullptr || type == nullptr ||
          expected_type != type) throw std::invalid_argument("invalid leg chain " + name);
      parent_link = chain_child->Attribute("link");
      auto vector_attribute = [&](const tinyxml2::XMLElement* item, const char* key,
                                  const Eigen::Vector3d& fallback) -> Eigen::Vector3d {
        if (item == nullptr || item->Attribute(key) == nullptr) return fallback;
        std::istringstream stream(item->Attribute(key));
        Eigen::Vector3d value;
        if (!(stream >> value.x() >> value.y() >> value.z()) || !value.allFinite()) {
          throw std::invalid_argument("invalid leg transform " + name);
        }
        return value;
      };
      const auto* chain_origin = element->FirstChildElement("origin");
      const Eigen::Vector3d xyz = vector_attribute(chain_origin, "xyz", Eigen::Vector3d::Zero());
      const Eigen::Vector3d rpy = vector_attribute(chain_origin, "rpy", Eigen::Vector3d::Zero());
      auto& transform = p.chains[leg].origins[axis];
      transform = Eigen::Isometry3d::Identity();
      transform.translation() = xyz;
      transform.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
      if (axis < 3) {
        Eigen::Vector3d direction = vector_attribute(element->FirstChildElement("axis"),
                                                     "xyz", Eigen::Vector3d::UnitX());
        if (direction.norm() < 1e-12) throw std::invalid_argument("zero joint axis " + name);
        p.chains[leg].axes[axis] = direction.normalized();
      } else if (parent_link != kLegs[leg] + "_foot") {
        throw std::invalid_argument("invalid foot chain " + name);
      }
      if (axis == 3) continue;
      const auto* child = element->FirstChildElement("child");
      const auto* limit = element->FirstChildElement("limit");
      const int index = 3 * leg + axis;
      if (child == nullptr || child->Attribute("link") == nullptr || limit == nullptr ||
          limit->QueryDoubleAttribute("lower", &p.lower[index]) != tinyxml2::XML_SUCCESS ||
          limit->QueryDoubleAttribute("upper", &p.upper[index]) != tinyxml2::XML_SUCCESS ||
          !std::isfinite(p.lower[index]) || !std::isfinite(p.upper[index]) ||
          p.lower[index] >= p.upper[index]) throw std::invalid_argument("invalid joint " + name);
      const unsigned int id = p.model.GetBodyId(child->Attribute("link"));
      if (!p.model.IsBodyId(id) || p.model.IsFixedBodyId(id) ||
          p.model.mJoints[id].mDoFCount != 1) throw std::invalid_argument("joint mapping " + name);
      p.indices[index] = p.model.mJoints[id].q_index;
    }
    // B2의 평행 thigh/calf 축과 축 방향 hip 오프셋에서 거리는 calf 각도만의 함수다.
    // 이 전제가 다른 URDF를 조용히 같은 모델로 취급하지 않는다.
    const auto& chain = p.chains[leg];
    for (int axis = 1; axis < 4; ++axis) {
      if (!chain.origins[axis].linear().isApprox(Eigen::Matrix3d::Identity(), 1e-10) ||
          std::abs(chain.origins[axis].translation().x()) > 1e-10) {
        throw std::invalid_argument("unsupported B2 radial reach geometry");
      }
    }
    if (!chain.axes[1].isApprox(Eigen::Vector3d::UnitY(), 1e-10) ||
        !chain.axes[2].isApprox(Eigen::Vector3d::UnitY(), 1e-10) ||
        std::abs(chain.origins[1].translation().z()) > 1e-10) {
      throw std::invalid_argument("unsupported B2 leg axes");
    }
    const double a = -chain.origins[2].translation().z();
    const double b = -chain.origins[3].translation().z();
    if (a <= 0 || b <= 0) throw std::invalid_argument("invalid B2 link lengths");
    const double lateral = chain.origins[1].translation().y() +
        chain.origins[2].translation().y() + chain.origins[3].translation().y();
    const double low = p.lower[3 * leg + 2], high = p.upper[3 * leg + 2];
    double min_cos = std::min(std::cos(low), std::cos(high));
    double max_cos = std::max(std::cos(low), std::cos(high));
    const double pi = std::acos(-1.0);
    if (std::ceil(low / (2 * pi)) <= std::floor(high / (2 * pi))) max_cos = 1;
    if (std::ceil((low - pi) / (2 * pi)) <= std::floor((high - pi) / (2 * pi))) min_cos = -1;
    p.reach_min[leg] = std::sqrt(lateral * lateral + a * a + b * b + 2 * a * b * min_cos);
    p.reach_max[leg] = std::sqrt(lateral * lateral + a * a + b * b + 2 * a * b * max_cos);
  }
}
RobotModel::~RobotModel() = default;
const JointVector& RobotModel::lower() const { return impl_->lower; }
const JointVector& RobotModel::upper() const { return impl_->upper; }
const std::string& RobotModel::path() const { return impl_->path; }

ModelSnapshot RobotModel::Evaluate(const ModelState& state) const {
  const auto& p = *impl_;
  auto model = p.model;  // 호출 사이에 RBDL의 가변 캐시를 공유하지 않는다.
  const Eigen::VectorXd q = p.Positions(state);
  if (!state.base_linear_velocity.allFinite() || !state.base_angular_velocity.allFinite() ||
      !state.joint_velocities.allFinite()) throw std::invalid_argument("invalid B2 velocity");
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.qdot_size);
  dq.head<3>() = state.base_linear_velocity;
  dq.segment<3>(3) = state.base_angular_velocity;
  for (int i = 0; i < 12; ++i) dq[p.indices[i]] = state.joint_velocities[i];
  ModelSnapshot out;
  rb::Math::Vector3d com, com_velocity;
  rb::Utils::CalcCenterOfMass(model, q, dq, nullptr, out.mass, com, &com_velocity);
  out.com = com;
  out.com_velocity = com_velocity;
  const Eigen::Matrix3d world_body = state.base_orientation.toRotationMatrix();
  out.com_body = world_body.transpose() * (out.com - state.base_position);
  Eigen::Matrix3d inertia_world = Eigen::Matrix3d::Zero();
  // 고정 링크의 질량은 movable body에 합성돼 있으므로 중복 합산하지 않는다.
  for (unsigned int id = 1; id < model.mBodies.size(); ++id) {
    const auto& body = model.mBodies[id];
    const Eigen::Vector3d center = rb::CalcBodyToBaseCoordinates(model, q, id, body.mCenterOfMass, false);
    const Eigen::Matrix3d rotation = rb::CalcBodyWorldOrientation(model, q, id, false).transpose();
    const Eigen::Vector3d delta = center - out.com;
    inertia_world += rotation * body.mInertia * rotation.transpose() +
        body.mMass * (delta.squaredNorm() * Eigen::Matrix3d::Identity() - delta * delta.transpose());
  }
  out.inertia_body = world_body.transpose() * inertia_world * world_body;
  for (int leg = 0; leg < 4; ++leg) {
    out.feet.row(leg) = rb::CalcBodyToBaseCoordinates(model, q, p.feet[leg], Eigen::Vector3d::Zero(), false).transpose();
    out.hips.row(leg) = rb::CalcBodyToBaseCoordinates(model, q, p.hips[leg], Eigen::Vector3d::Zero(), false).transpose();
  }
  out.mass_matrix = Eigen::MatrixXd::Zero(model.qdot_size, model.qdot_size);
  rb::CompositeRigidBodyAlgorithm(model, q, out.mass_matrix, false);
  out.bias = Eigen::VectorXd::Zero(model.qdot_size);
  rb::NonlinearEffects(model, q, dq, out.bias);
  if (!std::isfinite(out.mass) || out.mass <= 0 || !out.inertia_body.allFinite() ||
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(out.inertia_body).eigenvalues().minCoeff() <= 0) {
    throw std::runtime_error("invalid B2 mass/inertia");
  }
  return out;
}

Eigen::Matrix3d RobotModel::LegJacobian(const ModelState& state, int leg) const {
  if (leg < 0 || leg >= 4) throw std::invalid_argument("invalid leg");
  auto model = impl_->model;
  const auto q = impl_->Positions(state);
  Eigen::MatrixXd full = Eigen::MatrixXd::Zero(3, model.qdot_size);
  rb::CalcPointJacobian(model, q, impl_->feet[leg], Eigen::Vector3d::Zero(), full, true);
  Eigen::Matrix3d result;
  for (int i = 0; i < 3; ++i) result.col(i) = full.col(impl_->indices[3 * leg + i]);
  return result;
}

LegIkResult RobotModel::LegIk(int leg, const Eigen::Vector3d& target_body,
                             const Eigen::Vector3d& seed) const {
  if (leg < 0 || leg >= 4 || !target_body.allFinite() || !seed.allFinite()) {
    throw std::invalid_argument("invalid IK query");
  }
  const auto& chain = impl_->chains[leg];
  const Eigen::Vector3d lower = impl_->lower.segment<3>(3 * leg);
  const Eigen::Vector3d upper = impl_->upper.segment<3>(3 * leg);
  LegIkResult result;
  result.joints = seed.cwiseMax(lower).cwiseMin(upper);
  for (int iteration = 0; iteration < 80; ++iteration) {
    Eigen::Matrix3d jacobian;
    const Eigen::Vector3d point = chain.Point(result.joints, &jacobian);
    const Eigen::Vector3d error = target_body - point;
    result.error = error.norm();
    if (result.error < 1e-6) { result.success = true; break; }
    Eigen::Vector3d step = jacobian.transpose() *
        (jacobian * jacobian.transpose() + 1e-8 * Eigen::Matrix3d::Identity()).ldlt().solve(error);
    double scale = std::min(1.0, .2 / std::max(1e-12, step.norm()));
    bool accepted = false;
    for (int search = 0; search < 12; ++search) {
      const Eigen::Vector3d trial = (result.joints + scale * step).cwiseMax(lower).cwiseMin(upper);
      const Eigen::Vector3d point_trial = chain.Point(trial);
      if ((target_body - point_trial).norm() < result.error) {
        result.joints = trial;
        accepted = true;
        break;
      }
      scale *= .5;
    }
    if (!accepted) break;
  }
  result.error = (target_body - chain.Point(result.joints)).norm();
  result.success = result.error < 1e-6;
  return result;
}

void RobotModel::PopulatePlanner(const ModelState& state, Robot* robot, Input* input) const {
  if (robot == nullptr || input == nullptr) throw std::invalid_argument("null planner state");
  const auto owner = shared_from_this();
  const auto snapshot = Evaluate(state);
  robot->model = owner;
  const Eigen::Matrix3d rotation = state.base_orientation.toRotationMatrix();
  robot->mass = snapshot.mass;
  robot->inertia = snapshot.inertia_body;
  robot->com_offset = snapshot.com_body;
  robot->reference_joints = state.joints;
  robot->reach_min = impl_->reach_min;
  robot->reach_max = impl_->reach_max;
  robot->leg_min = impl_->reach_min.minCoeff();
  robot->leg_max = impl_->reach_max.maxCoeff();
  robot->height = snapshot.com.z() - snapshot.feet.col(2).mean();
  for (int leg = 0; leg < 4; ++leg) {
    robot->hips.row(leg) = (rotation.transpose() * (snapshot.hips.row(leg).transpose() - snapshot.com)).transpose();
    robot->nominal_offsets.row(leg) = (rotation.transpose() * (snapshot.feet.row(leg).transpose() - snapshot.com)).transpose();
    robot->reference_legs.row(leg) = robot->hips.row(leg) - robot->nominal_offsets.row(leg);
  }
  robot->use_reference_legs = true;
  input->position.head<3>() = snapshot.com;
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  input->position.tail<3>() << std::atan2(rotation(1, 0), rotation(0, 0)), pitch,
      std::atan2(rotation(2, 1), rotation(2, 2));
  input->velocity.head<3>() = snapshot.com_velocity;
  input->velocity.tail<3>() = EulerRates(input->position.tail<3>(), state.base_angular_velocity);
  input->feet = snapshot.feet;
}
}  // namespace b2_terrain_planner
