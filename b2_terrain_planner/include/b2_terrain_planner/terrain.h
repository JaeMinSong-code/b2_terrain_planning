/**
 * @file terrain.h
 * @brief 정적 높이맵의 보간·유효성·착지 비용과 후보 검색 계약.
 * @version 0.0.14
 * @update 2026-11-04: 앞/뒤 좌우 발 쌍의 Y 간격에만 근접 지수 비용을 적용함.
 * @update 2026-11-04: 발 주변 셀·착지점의 방향별 지수 거리 비용을 연결함.
 */
#ifndef B2_TERRAIN_PLANNER_TERRAIN_H_
#define B2_TERRAIN_PLANNER_TERRAIN_H_

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace b2_terrain_planner {

// 발 상태·계획은 foot 링크 원점(구 중심), 지도는 실제 지면의 절대 Z를 사용한다.
inline constexpr double kDefaultFootRadius = 0.032;

using SupportPositions =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

// 높이맵의 수직 반경 근사다. 경사면 법선 방향의 정확한 구 접촉점은 아니다.
inline double GroundToFootCenterZ(double ground_z, double foot_radius) {
  return ground_z + foot_radius;
}

inline double FootCenterToGroundZ(double foot_center_z, double foot_radius) {
  return foot_center_z - foot_radius;
}

// 행은 +Y, 열은 +X. origin은 (0, 0) 셀 중심이며 높이는 frame 기준 지면 Z다.
// 발 반경을 더한 지도나 foot 링크 높이를 이 자료형에 넣지 않는다.
struct HeightMap {
  Eigen::MatrixXd heights;
  Eigen::ArrayXXi valid;
  Eigen::MatrixXd variance;  // 지도 메타데이터. 계획의 셀 사용 여부를 제한하지 않는다.
  Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  double resolution = 0.05;
  std::string frame = "odom";
};

enum class TerrainLayer { kGround, kVirtualFloor };

struct TerrainSettings {
  bool pipeline_enabled = true;
  int median_iterations = 1;
  double virtual_floor_sigma = 0.20;
  double virtual_floor_radius = 0.15;
  double occlusion_weight = 10.0;
  double curvature_weight = 0.001;
  double virtual_height_weight = 100.0;
  double body_alignment_weight = 100.0;
  double foot_radius = kDefaultFootRadius;  // 지면↔발 중심의 수직 오프셋, m.
  double smoothing_sigma = 0.05;  // m, Gaussian 표준편차.
  double contact_tolerance = 0.005;  // m.
  double ground_weight = 1e4;
  double edge_weight = 3.0;
  double collision_weight = 0.001;  // TAMOLS Table I.
  // 발 주변 후보 셀과 연속 착지점에 공통으로 적용하는 좌우 발 쌍의 Y 간격 비용.
  double foot_separation_weight = 0.0;
  double foot_separation_distance_y = 0.08;
  double foot_separation_scale = 0.01;
  double collision_distance = 0.20;  // 비용이 활성화되는 발 사이 거리, m.
};

class TerrainDomainError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// 지도 자체는 조회 가능하지만 점유 영역 제외로 초기 후보가 소진된 경우다.
class OccupiedFootholdError : public TerrainDomainError {
 public:
  using TerrainDomainError::TerrainDomainError;
};

struct TerrainSample {
  double height = 0.0;
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  double occlusion = 0.0;
  double curvature_cost = 0.0;
  double virtual_height = 0.0;
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
  TerrainSample Sample(const Eigen::Vector2d& xy,
                       TerrainLayer layer = TerrainLayer::kGround) const;
  // Sample은 지면, LandingHeight와 FindFoothold는 발 중심을 반환한다.
  double LandingHeight(const Eigen::Vector2d& xy,
                       TerrainLayer layer = TerrainLayer::kGround) const;
  TerrainResidual Residual(const Eigen::Vector3d& foot,
                           TerrainLayer layer = TerrainLayer::kGround) const;
  CollisionResidual Separation(const Eigen::Vector2d& delta) const;
  // delta의 몸통 Y 투영만 사용한다. 호출자가 FL/FR 또는 RL/RR 쌍을 지정한다.
  CollisionResidual FootSeparation(const Eigen::Vector2d& delta,
                                  double heading) const;
  bool IsQueryable(const Eigen::Vector2d& xy) const;
  // origin은 셀 중심이다. 점유 셀과 대각선을 포함한 주변 1칸을 제외한다.
  bool IsOccupiedCell(const Eigen::Vector2d& xy,
                      const SupportPositions& occupied) const;
  // direction은 월드 XY 단위 진행 방향, 정지 시 Zero이다.
  double FootholdCost(const Eigen::Vector2d& xy, const Eigen::Vector2d& nominal,
                      const Eigen::Vector2d& direction, double backward_weight,
                      TerrainLayer layer = TerrainLayer::kGround,
                      const SupportPositions& separation_supports = {}, double heading = 0.0) const;
  Eigen::Vector3d FindFoothold(const Eigen::Vector2d& nominal,
                             const Eigen::Vector2d& lower,
                             const Eigen::Vector2d& upper,
                             const Eigen::Vector2d& direction = Eigen::Vector2d::Zero(),
                             double backward_weight = 0.0,
                             const std::function<bool(const Eigen::Vector3d&)>& admissible = {},
                             TerrainLayer layer = TerrainLayer::kGround,
                             const SupportPositions& occupied = {}, double heading = 0.0,
                             const SupportPositions& separation_supports = {}) const;
  const HeightMap& map() const { return map_; }
  const HeightMap& processed_map() const { return processed_; }
  const Eigen::MatrixXd& smooth_heights() const { return smooth_; }
  const HeightMap& virtual_map() const { return virtual_; }
  const TerrainSettings& settings() const { return settings_; }

 private:
  HeightMap map_;
  TerrainSettings settings_;
  HeightMap processed_;
  HeightMap virtual_;
  Eigen::MatrixXd smooth_;
  Eigen::MatrixXd occlusion_;
  Eigen::MatrixXd curvature_cost_;
  Eigen::MatrixXd virtual_gradients_[2];
  Eigen::MatrixXd gradients_[4];  // 원본·평활 높이의 X/Y 기울기.
  // 원본 관측 마스크는 map_에 보존한다. 처리 지도 높이와 비용의 계산 가능 영역.
  // pipeline OFF는 기존 관측 전용 조회 계약을 유지한다.
  Eigen::ArrayXXi height_known_;
  Eigen::ArrayXXi usable_;
};
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_TERRAIN_H_
