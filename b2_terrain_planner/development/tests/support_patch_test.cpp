/**
 * @file support_patch_test.cpp
 * @brief 접촉 지도 보정의 조회 가능성·높이·범위·접촉 마스크를 검증한다.
 * @version 0.0.4
 * @update 2026-10-30: 보간 높이 조회와 원본 관측 마스크를 구분해 스윙·접촉 계약을 검증함.
 * @update 2026-10-29: 지면·발 중심 변환과 반경 누락·중복 방지 회귀를 검증함.
 */
#include "b2_terrain_planner/support_patch.h"
#include "b2_terrain_planner/map_datum.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace b2_terrain_planner;
void Check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void Run(double resolution) {
  HeightMap map;
  map.resolution = resolution;
  const int n = static_cast<int>(std::round(4.0 / resolution));
  map.origin = Eigen::Vector2d::Constant(-2.0 + 0.5 * resolution);
  map.heights = Eigen::MatrixXd::Constant(n, n, -0.032);
  map.valid = Eigen::ArrayXXi::Ones(n, n);
  map.variance = Eigen::MatrixXd::Constant(n, n, 0.001);
  std::array<Eigen::Vector3d, 4> feet{{{0.354, 0.210, 0.001},
      {0.353, -0.174, 0.0}, {-0.303, 0.209, 0.002}, {-0.303, -0.176, -0.001}}};
  // 앞발 아래와 먼 곳의 미관측 영역. 덧씌우기 밖의 unknown은 그대로 남아야 한다.
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      const double x = map.origin.x() + col * resolution;
      if (x > 0.0) map.valid(row, col) = 0;
    }
  }
  const auto original = map;
  TerrainSettings settings;
  const double half_width = SupportPatchHalfWidth(0.30, resolution, settings.smoothing_sigma);
  Check(ApplySupportPatches(&map, feet, {true, true, true, true}, half_width) > 0,
        "no cells patched");
  const TerrainModel terrain(map, settings);
  for (const auto& foot : feet) {
    Check(terrain.IsQueryable(foot.head<2>()), "support foot is not queryable");
    Check(std::abs(terrain.Sample(foot.head<2>()).height - (foot.z() - 0.032)) < 1e-10,
          "support height mismatch");
  }
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      const Eigen::Vector2d xy = map.origin + resolution * Eigen::Vector2d(col, row);
      bool covered = false;
      for (const auto& foot : feet) {
        if ((xy - foot.head<2>()).cwiseAbs().maxCoeff() <= half_width + 1e-12) covered = true;
      }
      if (!covered) {
        Check(map.heights(row, col) == original.heights(row, col) &&
              map.valid(row, col) == original.valid(row, col) &&
              map.variance(row, col) == original.variance(row, col), "outside changed");
      }
    }
  }
  auto no_contacts = original;
  Check(ApplySupportPatches(&no_contacts, feet, {false, false, false, false}, half_width) == 0,
        "noncontact feet patched terrain");
  auto single = original;
  ApplySupportPatches(&single, feet, {true, false, false, false}, half_width);
  Check(TerrainModel(single, settings).Sample(feet[1].head<2>()).occlusion > 0,
        "swing foot acquired a full support patch");
  feet[0] << 100, 100, 0;
  Check(ApplySupportPatches(&single, feet, {true, false, false, false}, half_width) == 0,
        "outside foot altered map");
}
void RunMemory(double resolution) {
  HeightMap raw;
  const int n = static_cast<int>(std::round(2.0 / resolution));
  raw.resolution = resolution;
  raw.origin = Eigen::Vector2d::Constant(-1.0 + 0.5 * resolution);
  raw.heights = Eigen::MatrixXd::Constant(n, n, -0.032);
  raw.valid = Eigen::ArrayXXi::Zero(n, n);
  raw.variance = Eigen::MatrixXd::Constant(n, n, 0.01);
  std::array<Eigen::Vector3d, 4> feet{};
  feet[0] << 0.013, 0.021, 0.12;
  const double width = SupportPatchHalfWidth(0.05, resolution, 0.5);
  Check(width == 0.05, "filter enlarged the contact footprint");
  SupportPatchMemory memory;
  memory.SyncLayout(raw);
  memory.Observe(feet, {true, false, false, false}, width);
  auto first = raw;
  const int count = memory.ApplyTo(&first);
  Check(count > 0, "small footprint was not recorded");
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      const Eigen::Vector2d xy = raw.origin + resolution * Eigen::Vector2d(col, row);
      const bool inside = (xy - feet[0].head<2>()).cwiseAbs().maxCoeff() <= width + 1e-12;
      if (inside) {
        Check(first.valid(row, col) && std::abs(first.heights(row, col) - 0.088) < 1e-12,
              "contact height not stored");
      } else {
        Check(first.valid(row, col) == raw.valid(row, col) &&
              first.heights(row, col) == raw.heights(row, col), "footprint expanded");
      }
    }
  }
  // 이륙한 발 높이를 저장하지 않으며 새 원본 지도가 와도 기존 접촉 높이를 유지한다.
  feet[0].z() = 0.8;
  memory.Observe(feet, {false, false, false, false}, width);
  memory.SyncLayout(raw);
  auto lifted = raw;
  Check(memory.ApplyTo(&lifted) == count && lifted.heights.isApprox(first.heights),
        "lifting or new raw map erased contact history");
  auto shifted = raw;
  shifted.origin += Eigen::Vector2d(2 * resolution, -resolution);
  memory.SyncLayout(shifted);
  Check(memory.ApplyTo(&shifted) == count, "rolling map lost overlapping contacts");
  for (int row = 0; row < n - 1; ++row) {
    for (int col = 2; col < n; ++col) {
      if (first.valid(row, col)) {
        Check(shifted.valid(row + 1, col - 2) &&
              shifted.heights(row + 1, col - 2) == first.heights(row, col),
              "contact moved in world coordinates with rolling map");
      }
    }
  }
  feet[0].z() = 0.22;
  memory.Observe(feet, {true, false, false, false}, width);
  auto landed = shifted;
  memory.ApplyTo(&landed);
  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      if (landed.valid(row, col)) Check(std::abs(landed.heights(row, col) - 0.188) < 1e-12, "recontact kept old height");
    }
  }
  auto far = raw;
  far.origin.x() += 10.0;
  memory.SyncLayout(far);
  Check(memory.ApplyTo(&far) == 0, "out-of-window contact retained");
  memory.SyncLayout(raw);
  auto returned = raw;
  Check(memory.ApplyTo(&returned) == 0, "discarded contacts resurrected");
  memory.Observe(feet, {true, false, false, false}, width);
  auto other_frame = raw;
  other_frame.frame = "new_odom";
  memory.SyncLayout(other_frame);
  Check(memory.ApplyTo(&other_frame) == 0, "old frame contacts reused");
  memory.SyncLayout(raw);
  memory.Observe(feet, {true, false, false, false}, width);
  memory.Reset();
  Check(memory.ApplyTo(&returned) == 0, "explicit reset retained contacts");
}
// datum → 접촉 기억 → 실제 착지 후보/비용을 함께 거친다. 원점과 반경이 달라도
// 평지는 평지로 남고 발 목표만 지면 위 한 반경에 있어야 한다.
void RunHeightContract() {
  for (double ground_z : {0.0, -0.28, 0.17}) {
    for (double radius : {0.032, 0.05}) {
      HeightMap map;
      map.origin = Eigen::Vector2d::Constant(-0.975);
      map.heights = Eigen::MatrixXd::Constant(40, 40, ground_z + 0.01);
      map.valid = Eigen::ArrayXXi::Ones(40, 40);
      map.variance = Eigen::MatrixXd::Zero(40, 40);
      ApplyDatumOffset(&map, -0.01);
      std::array<Eigen::Vector3d, 4> centers{};
      centers[0] << 0.0, 0.0, ground_z + radius;
      SupportPatchMemory memory;
      memory.SyncLayout(map);
      memory.Observe(centers, {true, false, false, false}, 0.05, 1.0, radius);
      Check(memory.ApplyTo(&map, 1.0, 2.0) > 0, "no ground patch applied");
      Check((map.heights.array() - ground_z).abs().maxCoeff() < 1e-12,
            "foot center created an artificial hill at the patch boundary");
      memory.ApplyTo(&map, 1.1, 2.0);
      Check((map.heights.array() - ground_z).abs().maxCoeff() < 1e-12,
            "reapplying a stored patch subtracted the radius twice");
      TerrainSettings settings;
      settings.foot_radius = radius;
      const TerrainModel terrain(map, settings);
      const Eigen::Vector2d xy = Eigen::Vector2d::Zero();
      const auto target = terrain.FindFoothold(xy, xy.array() - 0.1, xy.array() + 0.1);
      Check(std::abs(target.z() - ground_z - radius) < 1e-12,
            "landing candidate is not one radius above ground");
      Check(std::abs(terrain.LandingHeight(xy) - target.z()) < 1e-12,
            "warm-start height differs from the landing candidate");
      Check(terrain.Residual(centers[0]).values.norm() < 1e-10,
            "consistent flat contact has terrain or edge cost");
      Eigen::Vector3d ground_point = centers[0];
      ground_point.z() = ground_z;
      Check(std::abs(terrain.Residual(ground_point).values[0]) > 1.0,
            "a foot center on the ground incorrectly has zero landing cost");
    }
  }
}
}  // namespace

int main() {
  try {
    RunHeightContract();
    Run(0.05);
    Run(0.10);
    Run(0.02);
    RunMemory(0.02);
    RunMemory(0.05);
    RunMemory(0.10);
    std::cout << "PASS support patches: narrow footprint, retained contacts, recontact, rolling map, reset\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
