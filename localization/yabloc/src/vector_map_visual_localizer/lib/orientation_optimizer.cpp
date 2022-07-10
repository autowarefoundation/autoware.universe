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
  // std::cout << summary.FullReport() << std::endl;

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();

  // auto eval = [&vp](const Eigen::Quaternionf & q) -> float {
  //   const Eigen::Vector3f normal = q * Eigen::Vector3f::UnitZ();
  //   auto intersection = [&normal](
  //                         const Eigen::Vector3f & s, const Eigen::Vector3f & t) ->
  //                         Eigen::Vector3f {
  //     float lambda = -normal.dot(s) / (normal.dot(t) + 1e-6f);
  //     return s + lambda * t;
  //   };
  //   Eigen::Vector3f pl = intersection({-1, 0, 1}, Eigen::Vector3f::UnitY());
  //   Eigen::Vector3f pr = intersection({1, 0, 1}, Eigen::Vector3f::UnitY());
  //   Eigen::Vector3f s(-normal.z() / normal.x(), 0, 1);
  //   Eigen::Vector3f t = pl - pr;
  //   Eigen::Vector3f n(t.y(), -t.x(), 0);
  //   float distance = n.normalized().dot(pl - vp);
  //   return std::abs(distance);
  // };
  // std::cout << "before: " << eval(R.unit_quaternion()) << std::endl;
  // std::cout << "after: " << eval(qf) << std::endl;

  return Sophus::SO3f(qf);
}

Sophus::SO3f optimizeOnce(
  const Sophus::SO3f & R, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical)
{
  Problem problem;
  LossFunction * loss = nullptr;

  Eigen::Vector4d q = R.unit_quaternion().coeffs().cast<double>();
  problem.AddParameterBlock(q.data(), 4, new ceres::EigenQuaternionParameterization());
  problem.AddResidualBlock(VanishPointFactor::create(vp), loss, q.data());
  problem.AddResidualBlock(HorizonFactor::create(vertical), loss, q.data());

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
  // std::cout << summary.FullReport() << std::endl;

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();

  return Sophus::SO3f(qf);
}
}  // namespace imgproc::opt