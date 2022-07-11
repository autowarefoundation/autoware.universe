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
int Vertex::index_max = 0;

Sophus::SO3f Optimizer::optimize(
  const Sophus::SO3f & dR, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical,
  const Sophus::SO3f & initial_R)
{
  if (vertices_.empty()) {
    Vertex::Ptr v = std::make_shared<Vertex>(initial_R.unit_quaternion(), vp, dR);
    vertices_.push_back(v);
    return dR;
  }

  Sophus::SO3f last_R = vertices_.back()->so3f();
  Vertex::Ptr v = std::make_shared<Vertex>((last_R * dR).unit_quaternion(), vp, dR);
  vertices_.push_back(v);

  ceres::LossFunction * loss = new ceres::CauchyLoss(0.1f);

  // Build opmization problem
  Problem problem;
  for (int i = 0; i < vertices_.size(); i++) {
    Vertex::Ptr & v = vertices_.at(i);
    problem.AddParameterBlock(v->q_.data(), 4, new ceres::EigenQuaternionParameterization());
    if (i != (vertices_.size() - 1)) {
      problem.AddResidualBlock(VanishPointFactor::create(vp), loss, v->q_.data());
      problem.AddResidualBlock(HorizonFactor::create(vertical), nullptr, v->q_.data());
    }

    if (i == 0) continue;

    Vertex::Ptr & v_prev = vertices_.at(i - 1);
    problem.AddResidualBlock(
      ImuFactor::create(v->dR_, 100), nullptr, v_prev->q_.data(), v->q_.data());
  }
  // Fix first vertix
  // problem.SetParameterBlockConstant(vertices_.front()->q_.data());

  // Solve opmization problem
  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;

  // DEBUG
  {
    auto vn = *std::prev(vertices_.end(), 1);
    auto vm = *std::prev(vertices_.end(), 2);
    std::cout << "Rn: " << vn->so3f().unit_quaternion().coeffs().transpose() << std::endl;
    std::cout << "Rm: " << vm->so3f().unit_quaternion().coeffs().transpose() << std::endl;
    std::cout << "dR: " << dR.unit_quaternion().coeffs().transpose() << std::endl;
  }

  return vertices_.back()->so3f();
}

Sophus::SO3f optimizeOnce(const Sophus::SO3f & R, const Eigen::Vector3f & vp)
{
  Problem problem;

  Eigen::Vector4d q = R.unit_quaternion().coeffs().cast<double>();
  problem.AddParameterBlock(q.data(), 4, new ceres::EigenQuaternionParameterization());
  problem.AddResidualBlock(VanishPointFactor::create(vp), nullptr, q.data());

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();

  // DEBUG:
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

  Eigen::Vector4d q = R.unit_quaternion().coeffs().cast<double>();
  problem.AddParameterBlock(q.data(), 4, new ceres::EigenQuaternionParameterization());
  problem.AddResidualBlock(VanishPointFactor::create(vp), nullptr, q.data());
  problem.AddResidualBlock(HorizonFactor::create(vertical), nullptr, q.data());

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();
  return Sophus::SO3f(qf);
}
}  // namespace imgproc::opt