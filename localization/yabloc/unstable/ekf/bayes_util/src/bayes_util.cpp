// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bayes_util/bayes_util.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace pcdless::bayes_util
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

template <typename T>
Eigen::Matrix<T, 2, 2> make_rotation_matrix(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

template <typename T>
Eigen::Matrix<T, 2, 2> make_spd_matrix(const T * const param)
{
  Eigen::Matrix<T, 2, 2> R = make_rotation_matrix(param[2]);

  Eigen::Matrix<T, 2, 2> singular;
  singular << param[0], T(0), T(0), param[1];

  return R * singular * R.transpose();
}

struct CostFunctor
{
  template <typename T>
  bool operator()(const T * const x, T * residual) const
  {
    Eigen::Matrix<T, 2, 2> A = make_spd_matrix(x);
    residual[0] = S_(0, 0) - A(0, 0);
    residual[1] = S_(0, 1) - A(0, 1);
    residual[2] = S_(1, 0) - A(1, 0);
    residual[3] = S_(1, 1) - A(1, 1);
    return true;
  }

  CostFunctor(const Eigen::Matrix2d & S) : S_(S) {}

  const Eigen::Matrix2d S_;
};

Eigen::Matrix2d approximate_by_spd(const Eigen::Matrix2d & target, bool verbose)
{
  Problem problem;
  CostFunction * cost_function =
    new AutoDiffCostFunction<CostFunctor, 4, 3>(new CostFunctor(target));
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  constexpr double epsilon = 0.04;
  problem.AddResidualBlock(cost_function, nullptr, x.data());
  problem.SetParameterLowerBound(x.data(), 0, epsilon);
  problem.SetParameterLowerBound(x.data(), 1, epsilon);

  Solver::Options options;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << x.transpose() << std::endl;
  }
  return make_spd_matrix(x.data());
}

Eigen::Matrix2f debayes_covariance(
  const Eigen::Matrix2f & prior_covariance, const Eigen::Matrix2f & post_covariance)
{
  std::cout << "prior_cov:" << std::endl;
  std::cout << prior_covariance << std::endl;
  std::cout << "post_cov:" << std::endl;
  std::cout << post_covariance << std::endl;

  // DEBUG:
  const Eigen::Matrix2f epsilon = 1e-4f * Eigen::Matrix2f::Identity();
  const Eigen::Matrix2f post_info = (post_covariance + epsilon).inverse();
  const Eigen::Matrix2f prior_info = (prior_covariance + epsilon).inverse();

  Eigen::Matrix2d likelihood_info_d = (post_info - prior_info).cast<double>();
  Eigen::Matrix2f likelihood_info = approximate_by_spd(likelihood_info_d).cast<float>();

  std::cout << "likelihood_cov:" << std::endl;
  std::cout << likelihood_info.inverse() << std::endl;

  std::cout << "debug post_cov:" << std::endl;
  std::cout << (likelihood_info + prior_info).inverse() << std::endl;

  return likelihood_info;
}

}  // namespace pcdless::bayes_util