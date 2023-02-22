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
  problem.AddResidualBlock(cost_function, nullptr, x.data());
  problem.SetParameterLowerBound(x.data(), 0, 0);
  problem.SetParameterLowerBound(x.data(), 1, 0);

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
  std::cout << "prior:" << std::endl;
  std::cout << prior_covariance << std::endl;
  std::cout << "post:" << std::endl;
  std::cout << post_covariance << std::endl;

  return post_covariance;
}

}  // namespace pcdless::bayes_util