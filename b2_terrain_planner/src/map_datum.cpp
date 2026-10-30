/**
 * @file map_datum.cpp
 * @brief 관측 지도 datum 재정착: 변화율 제한 오프셋 추적과 전역 수직 이동.
 * @version 0.0.1
 */
#include "b2_terrain_planner/map_datum.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace b2_terrain_planner {

double UpdateDatumOffset(DatumOffsetState* state, const std::vector<double>& diffs,
                         double dt, double max_rate) {
  if (state == nullptr) throw std::invalid_argument("null datum state");
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(max_rate) || max_rate <= 0) {
    throw std::invalid_argument("datum dt and max_rate must be positive and finite");
  }
  std::vector<double> finite;
  finite.reserve(diffs.size());
  for (double d : diffs) if (std::isfinite(d)) finite.push_back(d);
  // 확인 지지가 2개 미만이면 직전 오프셋을 유지한다(초기화 전이면 무효 유지).
  if (finite.size() < 2) return state->offset;
  std::sort(finite.begin(), finite.end());
  const std::size_t m = finite.size() / 2;
  const double target = (finite.size() % 2) ? finite[m]
                                            : 0.5 * (finite[m - 1] + finite[m]);
  // 첫 측정에서도 점프하지 않는다. 측정 가능 시점까지 쌓인 드리프트도 0에서
  // 변화율 제한으로 따라잡아, 지도가 solve 사이에 크게 이동하지 않게 한다.
  state->valid = true;
  const double limit = max_rate * dt;
  state->offset += std::min(limit, std::max(-limit, target - state->offset));
  return state->offset;
}

int ApplyDatumOffset(HeightMap* map, double offset) {
  if (map == nullptr) throw std::invalid_argument("null terrain map");
  if (!std::isfinite(offset) || offset == 0.0) return 0;
  int shifted = 0;
  for (int row = 0; row < map->heights.rows(); ++row) {
    for (int col = 0; col < map->heights.cols(); ++col) {
      if (map->valid(row, col)) {
        map->heights(row, col) += offset;
        ++shifted;
      }
    }
  }
  return shifted;
}

}  // namespace b2_terrain_planner
