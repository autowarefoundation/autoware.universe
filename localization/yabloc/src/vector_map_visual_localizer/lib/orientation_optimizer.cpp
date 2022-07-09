#include "imgproc/orientation_optimizer.hpp"

#include "imgproc/ceres_factor.hpp"

#include <sophus/geometry.hpp>

#include <ceres/ceres.h>

#include <vector>

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

namespace imgproc::opt
{

Sophus::SO3f optimizeOnce(const Sophus::SO3f & R, const Eigen::Vector3f & vp)
{
  Problem problem;
  LossFunction * loss = nullptr;

  Eigen::Vector4d q = R.unit_quaternion().coeffs().cast<double>();
  problem.AddParameterBlock(q.data(), 4, new ceres::EigenQuaternionParameterization());
  problem.AddResidualBlock(VanishPointFactor::create(vp), loss, q.data());

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();

  double cost = VanishPointFactor(vp.cast<double>())(q.data());
  std::cout << "final cost " << cost << " " << vp.transpose() << std::endl;

  return Sophus::SO3f(qf);
}

void sample()
{
  double x = 1, y = 1, r = 3;
  double m = std::sqrt(r);

  Problem problem;
  LossFunction * loss = nullptr;

  for (int i = 0; i < 4; i++) {
    double xx = 4 * std::cos(i * 6.28 / 4);
    double yy = 4 * std::sin(i * 6.28 / 4);
    problem.AddResidualBlock(DistanceFromCircleCost::create(xx, yy), loss, &x, &y, &m);
  }

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  r = m * m;
  std::cout << summary.BriefReport() << std::endl;
}
}  // namespace imgproc::opt