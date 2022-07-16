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

std::vector<Sophus::SO3f> Optimizer::allRotations() const
{
  std::vector<Sophus::SO3f> rotations;
  for (int i = 0; i < vertices_.size(); i++) {
    const Vertex::Ptr & v = vertices_.at(i);
    rotations.push_back(v->so3f());
  }
  return rotations;
}

Sophus::SO3f Optimizer::optimize(
  const Sophus::SO3f & dR, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical,
  const Sophus::SO3f & initial_R)
{
  // Push back a new vertex into ring buffer
  if (vertices_.empty()) {
    Vertex::Ptr v = std::make_shared<Vertex>(initial_R.unit_quaternion(), vp, dR);
    vertices_.push_back(v);
    return dR;
  }
  Sophus::SO3f last_R = vertices_.back()->so3f();
  Vertex::Ptr v = std::make_shared<Vertex>((last_R * dR).unit_quaternion(), vp, dR);
  vertices_.push_back(v);

  // Build opmization problem
  Problem problem;
  ceres::LossFunction * loss = nullptr;
  for (int i = 0; i < vertices_.size(); i++) {
    Vertex::Ptr & v = vertices_.at(i);
    problem.AddParameterBlock(v->q_.data(), 4, new ceres::EigenQuaternionParameterization());
    // vp-cost
    problem.AddResidualBlock(VanishPointFactor::create(vp), loss, v->q_.data());
    // horizontal-coost
    problem.AddResidualBlock(HorizonFactor::create(vertical), loss, v->q_.data());

    if (i == 0) continue;

    // imu-cost
    Vertex::Ptr & v_prev = vertices_.at(i - 1);
    problem.AddResidualBlock(
      ImuFactor::create(v->dR_, 10), nullptr, v_prev->q_.data(), v->q_.data());
  }

  std::cout << "=== BEFORE ===" << std::endl;
  printEvaluation();

  // TODO: Fix first vertix
  // I guess we should fix it
  problem.SetParameterBlockConstant(vertices_.front()->q_.data());

  // Solve opmization problem
  Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Save after-loss infomation
  std::cout << "=== AFTER ===" << std::endl;
  printEvaluation();
  std::cout << summary.BriefReport() << std::endl;

  // DEBUG
  // {
  //   auto vn = *std::prev(vertices_.end(), 1);
  //   auto vm = *std::prev(vertices_.end(), 2);
  //   // std::cout << "Rn: " << vn->so3f().unit_quaternion().coeffs().transpose() << std::endl;
  //   // std::cout << "Rm: " << vm->so3f().unit_quaternion().coeffs().transpose() << std::endl;
  //   Eigen::Quaternionf dq = (vm->so3f().inverse() * vn->so3f()).unit_quaternion();
  //   std::cout << "ddR: " << dq.coeffs().transpose() << std::endl;
  //   std::cout << " dR: " << dR.unit_quaternion().coeffs().transpose() << std::endl;
  //   std::cout << "|dq| " << (dq.inverse() * dR.unit_quaternion()).vec().norm() << std::endl;
  //   std::cout << " Rn: " << vn->so3f().unit_quaternion().coeffs().transpose() << std::endl;
  // }

  return vertices_.back()->so3f();
}

void Optimizer::printEvaluation() const
{
  // Evaluate before status
  auto vp0 = vertices_.front();
  float cost_vp0 = VanishPointFactor::eval(vp0->vp_, vp0->q_.data());
  float cost_hz0 = HorizonFactor::eval(Eigen::Vector2f::UnitY(), vp0->q_.data());

  if (vertices_.size() < 2) return;
  auto vp1 = vertices_.at(1);
  float cost_vp1 = VanishPointFactor::eval(vp1->vp_, vp1->q_.data());
  float cost_hz1 = HorizonFactor::eval(Eigen::Vector2f::UnitY(), vp1->q_.data());

  float cost_imu = ImuFactor::eval(vp1->dR_, vp0->q_.data(), vp1->q_.data());

  std::cout << "vp0 " << cost_vp0 << std::endl;
  std::cout << "vp1 " << cost_vp1 << std::endl;
  std::cout << "hz0 " << cost_hz0 << std::endl;
  std::cout << "hz1 " << cost_hz1 << std::endl;
  std::cout << "imu " << cost_imu << std::endl;
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