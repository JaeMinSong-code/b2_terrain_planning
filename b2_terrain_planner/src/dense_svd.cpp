/**
 * @file dense_svd.cpp
 * @brief LAPACK DGESVD와 Eigen Jacobi SVD로 절단 최소제곱을 계산한다.
 * @version 0.0.2
 * @update 2026-10-14: 고정 Jacobian 보정에서 재사용할 의사 역행렬을 제공함.
 * @update 2026-10-14: 행렬 크기에 맞춘 LAPACK 작업 공간과 오류 검사를 추가함.
 */
#include "dense_svd.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/SVD>

// 시스템 LAPACK의 LP64 Fortran ABI. CMake가 동일한 정수 ABI를 요구한다.
extern "C" void dgesvd_(const char*, const char*, const int*, const int*, double*,
                        const int*, double*, double*, const int*, double*,
                        const int*, double*, const int*, int*);

namespace b2_terrain_planner {
DenseSvd::DenseSvd(const Eigen::MatrixXd& matrix, double threshold, bool full_v,
                   bool lapack) {
  if (matrix.rows() == 0 || matrix.cols() == 0 || !matrix.allFinite() ||
      !std::isfinite(threshold) || threshold < 0) {
    throw std::invalid_argument("invalid SVD input");
  }
  if (!lapack) {
    unsigned int options = Eigen::ComputeThinU | Eigen::ComputeThinV;
    if (full_v) options = Eigen::ComputeThinU | Eigen::ComputeFullV;
    jacobi_ = std::make_unique<Eigen::JacobiSVD<Eigen::MatrixXd>>(matrix, options);
    auto& svd = *jacobi_;
    svd.setThreshold(threshold);
    rank_ = svd.rank();
    u_ = svd.matrixU();
    v_ = svd.matrixV();
    singular_values_ = svd.singularValues();
    return;
  }
  const int m = matrix.rows(), n = matrix.cols(), k = std::min(m, n);
  int v_rows = k;
  char job_u = 'S', job_v = 'S';
  if (full_v) { v_rows = n; job_v = 'A'; }
  Eigen::MatrixXd a = matrix;
  Eigen::MatrixXd vt(v_rows, n);
  u_.resize(m, k);
  singular_values_.resize(k);
  int work_size = -1, info = 0;
  double optimal_size = 0;
  dgesvd_(&job_u, &job_v, &m, &n, a.data(), &m, singular_values_.data(),
          u_.data(), &m, vt.data(), &v_rows, &optimal_size, &work_size, &info);
  if (info != 0 || !std::isfinite(optimal_size) || optimal_size < 1 ||
      optimal_size > std::numeric_limits<int>::max()) {
    throw std::runtime_error("LAPACK SVD workspace query failed");
  }
  work_size = static_cast<int>(optimal_size);
  std::vector<double> work(work_size);
  dgesvd_(&job_u, &job_v, &m, &n, a.data(), &m, singular_values_.data(),
          u_.data(), &m, vt.data(), &v_rows, work.data(), &work_size, &info);
  if (info != 0) throw std::runtime_error("LAPACK SVD failed to converge");
  v_ = vt.transpose();
  const double cutoff = std::max(singular_values_[0] * threshold,
                                  std::numeric_limits<double>::min());
  while (rank_ < k && singular_values_[rank_] > cutoff) ++rank_;
}

Eigen::MatrixXd DenseSvd::PseudoInverse() const {
  if (rank_ == 0) return Eigen::MatrixXd::Zero(v_.rows(), u_.rows());
  if (jacobi_) return jacobi_->solve(Eigen::MatrixXd::Identity(u_.rows(), u_.rows()));
  return v_.leftCols(rank_) * singular_values_.head(rank_).cwiseInverse().asDiagonal() *
      u_.leftCols(rank_).transpose();
}

Eigen::VectorXd DenseSvd::Solve(const Eigen::VectorXd& rhs) const {
  if (rhs.size() != u_.rows()) throw std::invalid_argument("invalid SVD rhs");
  if (jacobi_) return jacobi_->solve(rhs);
  return v_.leftCols(rank_) *
      (u_.leftCols(rank_).transpose() * rhs).cwiseQuotient(singular_values_.head(rank_));
}
}  // namespace b2_terrain_planner
