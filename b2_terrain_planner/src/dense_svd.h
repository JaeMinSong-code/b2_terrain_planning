/**
 * @file dense_svd.h
 * @brief SQP의 최소 노름 보정과 영공간을 동일한 절단 기준으로 계산한다.
 * @version 0.0.2
 * @update 2026-10-14: 고정 Jacobian 보정에서 재사용할 의사 역행렬을 제공함.
 * @update 2026-10-14: Jacobi와 LAPACK SVD의 비교 가능한 공통 인터페이스를 추가함.
 */
#ifndef B2_TERRAIN_PLANNER_SRC_DENSE_SVD_H_
#define B2_TERRAIN_PLANNER_SRC_DENSE_SVD_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <memory>

namespace b2_terrain_planner {
class DenseSvd {
 public:
  DenseSvd(const Eigen::MatrixXd& matrix, double threshold, bool full_v,
           bool lapack);
  Eigen::VectorXd Solve(const Eigen::VectorXd& rhs) const;
  Eigen::MatrixXd PseudoInverse() const;
  int rank() const { return rank_; }
  const Eigen::VectorXd& singular_values() const { return singular_values_; }
  const Eigen::MatrixXd& v() const { return v_; }

 private:
  Eigen::MatrixXd u_, v_;
  Eigen::VectorXd singular_values_;
  int rank_ = 0;
  std::unique_ptr<Eigen::JacobiSVD<Eigen::MatrixXd>> jacobi_;
};
}  // namespace b2_terrain_planner
#endif  // B2_TERRAIN_PLANNER_SRC_DENSE_SVD_H_
