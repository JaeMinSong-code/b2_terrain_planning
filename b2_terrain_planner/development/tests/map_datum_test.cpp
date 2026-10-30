/**
 * @file map_datum_test.cpp
 * @brief datum 오프셋 추적(변화율 제한·median·유지)과 전역 수직 이동을 검증한다.
 * @version 0.0.1
 */
#include "b2_terrain_planner/map_datum.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {
using namespace b2_terrain_planner;

void Check(bool ok, const std::string& m) { if (!ok) throw std::runtime_error(m); }

void Run() {
  // 첫 측정도 점프 없이 0에서 ramp한다(측정 지연으로 쌓인 드리프트 포함).
  DatumOffsetState s;
  double off = UpdateDatumOffset(&s, {0.02, 0.018, 0.30}, 0.1, 0.01);  // 한 발 튐 → median 0.02
  Check(s.valid && std::abs(off - 0.001) < 1e-9, "init must ramp from zero, not jump");

  // 변화율 제한: 큰 목표(0.10)로도 스텝당 max_rate*dt=0.001만 움직인다.
  off = UpdateDatumOffset(&s, {0.10, 0.10, 0.10}, 0.1, 0.01);
  Check(std::abs(off - 0.002) < 1e-9, "rate limit not applied");

  // 지지 2개 미만(스윙 전환·깜빡임)은 직전 오프셋 유지.
  off = UpdateDatumOffset(&s, {0.50}, 0.1, 0.01);
  Check(std::abs(off - 0.002) < 1e-9, "sub-2 support should hold offset");
  off = UpdateDatumOffset(&s, {}, 0.1, 0.01);
  Check(std::abs(off - 0.002) < 1e-9, "empty should hold offset");

  // 드리프트 추종: 목표가 제한 안이면 정확히 도달한다(짝수 개 median=평균).
  off = UpdateDatumOffset(&s, {0.0025, 0.0025}, 0.1, 0.01);
  Check(std::abs(off - 0.0025) < 1e-9, "should reach target within rate limit");
  // 음수 방향도 대칭으로 제한된다.
  off = UpdateDatumOffset(&s, {-0.05, -0.05}, 0.1, 0.01);
  Check(std::abs(off - 0.0015) < 1e-9, "negative direction rate limit");

  // 초기화 전 지지 부족은 무효 유지(오프셋 0).
  DatumOffsetState s2;
  off = UpdateDatumOffset(&s2, {0.1}, 0.1, 0.01);
  Check(!s2.valid && off == 0.0, "pre-init with <2 support must stay invalid/zero");

  // 잘못된 dt·max_rate는 예외.
  bool threw = false;
  try { UpdateDatumOffset(&s, {0.0, 0.0}, 0.0, 0.01); } catch (const std::invalid_argument&) { threw = true; }
  Check(threw, "non-positive dt must throw");

  // 전역 수직 이동: 유효 셀만 offset만큼, 상대 모양(단차) 보존.
  HeightMap hm;
  hm.heights = Eigen::MatrixXd::Zero(4, 4);
  hm.variance = Eigen::MatrixXd::Zero(4, 4);
  hm.valid = Eigen::ArrayXXi::Ones(4, 4);
  hm.heights(1, 1) = 0.0; hm.heights(1, 2) = 0.1;  // 10cm 단차
  hm.valid(2, 2) = 0;                               // 미관측 셀 하나
  const double before_step = hm.heights(1, 2) - hm.heights(1, 1);
  const int n = ApplyDatumOffset(&hm, -0.03);
  Check(n == 15, "should shift exactly the 15 valid cells");
  Check(std::abs(hm.heights(1, 1) - (-0.03)) < 1e-12 &&
        std::abs(hm.heights(1, 2) - 0.07) < 1e-12, "offset not applied to valid cells");
  Check(std::abs((hm.heights(1, 2) - hm.heights(1, 1)) - before_step) < 1e-12,
        "relative terrain shape (step) must be preserved");
  Check(hm.heights(2, 2) == 0.0, "invalid cell must stay untouched");

  // offset 0은 무연산.
  Check(ApplyDatumOffset(&hm, 0.0) == 0, "zero offset must be a no-op");
}
}  // namespace

int main() {
  try {
    Run();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "map_datum_test failed: %s\n", e.what());
    return 1;
  }
  std::printf("map_datum_test passed\n");
  return 0;
}
