/**
 * @file terrain.h
 * @brief 정적 높이맵의 보간·유효성·착지 비용과 후보 검색 계약.
 * @version 0.0.6
 * @update 2026-10-14: 맵 조회 가능 여부와 비용 계약에서 추가 하드 판정 설정을 제거함.
 * @update 2026-10-14: 예측 도달성 조건을 후보 검색에 선택적으로 적용함.
 */
#ifndef B2_TERRAIN_PLANNER_TERRAIN_H_
#define B2_TERRAIN_PLANNER_TERRAIN_H_

#include <functional>
#include <stdexcept>
#include <string>
#include <Eigen/Core>

namespace b2_terrain_planner {

// 행은 +Y, 열은 +X. origin은 (0, 0) 셀 중심이며 높이는 frame 기준 절대 Z다.
struct HeightMap {
  Eigen::MatrixXd heights;
  Eigen::ArrayXXi valid;
  Eigen::MatrixXd variance;
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  double resolution = 0.05;
  std::string frame = "odom";
};

struct TerrainSettings {
  double smoothing_sigma = 0.05;  // m, Gaussian 표준편차.
  double max_variance = 0.0025;   // m².
  double contact_tolerance = 0.005;  // m.
  double ground_weight = 1e4;
  double edge_weight = 3.0;
  double collision_weight = 100.0;
  double collision_distance = 0.20;  // 비용이 활성화되는 발 사이 거리, m.
};

class TerrainDomainError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct TerrainSample {
  double height = 0.0;
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  Eigen::Vector4d edge = Eigen::Vector4d::Zero();
  Eigen::Matrix<double, 4, 2> edge_jacobian =
      Eigen::Matrix<double, 4, 2>::Zero();
};

struct TerrainResidual {
  Eigen::Matrix<double, 5, 1> values;
  Eigen::Matrix<double, 5, 3> jacobian;
};

struct CollisionResidual {
  double value = 0.0;
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();  // delta에 대한 미분.
};

class TerrainModel {
 public:
  TerrainModel(const HeightMap& map, const TerrainSettings& settings);
  TerrainSample Sample(const Eigen::Vector2d& xy) const;
  TerrainResidual Residual(const Eigen::Vector3d& foot) const;
  CollisionResidual Separation(const Eigen::Vector2d& delta) const;
  bool IsQueryable(const Eigen::Vector2d& xy) const;
  // direction은 월드 XY 단위 진행 방향, 정지 시 Zero이다.
  double FootholdCost(const Eigen::Vector2d& xy, const Eigen::Vector2d& nominal,
                      const Eigen::Vector2d& direction, double backward_weight) const;
  Eigen::Vector3d FindFoothold(const Eigen::Vector2d& nominal,
                             const Eigen::Vector2d& lower,
                             const Eigen::Vector2d& upper,
                             const Eigen::Vector2d& direction = Eigen::Vector2d::Zero(),
                             double backward_weight = 0.0,
                             const std::function<bool(const Eigen::Vector3d&)>& admissible = {}) const;
  const HeightMap& map() const { return map_; }
  const TerrainSettings& settings() const { return settings_; }

 private:
  HeightMap map_;
  TerrainSettings settings_;
  Eigen::MatrixXd gradients_[4];  // 원본·평활 높이의 X/Y 기울기.
  Eigen::ArrayXXi usable_;
};
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_TERRAIN_H_
