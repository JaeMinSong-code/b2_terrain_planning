/**
 * @file robot_model.h
 * @brief B2 URDF의 RBDL 동역학·FK·관절 제한 IK와 planner 상태 변환.
 * @version 0.0.2
 * @update 2026-11-03: 미사용 LegIk·LegJacobian·질량행렬/바이어스를 제거하고 공용 DefaultModelPath를 추가함.
 * @update 2026-10-13: 실제 B2 모델과 CoM 기준 계획 모델의 변환 계약을 추가함.
 */
#ifndef B2_TERRAIN_PLANNER_ROBOT_MODEL_H_
#define B2_TERRAIN_PLANNER_ROBOT_MODEL_H_

#include <array>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace b2_terrain_planner {
struct Robot;
struct Input;
using JointVector = Eigen::Matrix<double, 12, 1>;
using FootMatrix = Eigen::Matrix<double, 4, 3, Eigen::RowMajor>;

// 관절 순서는 FL/FR/RL/RR의 hip/thigh/calf. 각속도는 몸통 좌표계다.
struct ModelState {
  Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond base_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d base_linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_angular_velocity = Eigen::Vector3d::Zero();
  JointVector joints = JointVector::Zero();
  JointVector joint_velocities = JointVector::Zero();
};

struct ModelSnapshot {
  double mass = 0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Vector3d com_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d com_body = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia_body = Eigen::Matrix3d::Zero();
  FootMatrix feet = FootMatrix::Zero();
  FootMatrix hips = FootMatrix::Zero();
};

class RobotModel : public std::enable_shared_from_this<RobotModel> {
 public:
  explicit RobotModel(const std::string& urdf_path);
  ~RobotModel();
  RobotModel(const RobotModel&) = delete;
  RobotModel& operator=(const RobotModel&) = delete;
  ModelSnapshot Evaluate(const ModelState& state) const;
  // make_shared로 생성한 모델의 소유권도 Robot에 함께 연결한다.
  void PopulatePlanner(const ModelState& state, Robot* robot, Input* input) const;
  const JointVector& lower() const;
  const JointVector& upper() const;
  const std::string& path() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 설치본의 share 경로에서 B2 URDF를 찾고, 없으면 build_fallback을 반환한다.
// 두 실행파일(CLI·ROS 노드)이 같은 탐색 규칙을 공유한다.
std::string DefaultModelPath(const std::string& build_fallback);
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_ROBOT_MODEL_H_
