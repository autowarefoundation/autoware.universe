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
Optimizer::Optimizer(rclcpp::Node * node, bool verbose)
: verbose_(verbose),
  vertices_(node->declare_parameter<int>("opt.max_vertex_cnt", 5)),
  imu_factor_gain_(node->declare_parameter<float>("opt.imu_factor_gain", 1.0f)),
  vp_factor_gain_(node->declare_parameter<float>("opt.vp_factor_gain", 1.0f)),
  hz_factor_gain_(node->declare_parameter<float>("opt.hz_factor_gain", 1.0f))
{
}

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
  const Sophus::SO3f & dR, const OptVector3f & vp, const Eigen::Vector2f & vertical,
  const Sophus::SO3f & initial_R)
{
  // Store the first vertex and return ASAP
  if (vertices_.empty()) {
    Vertex::Ptr v = std::make_shared<Vertex>(initial_R.unit_quaternion(), vp, dR);
    vertices_.push_back(v);
    return initial_R;
  }

  // Push back a new vertex into ring buffer
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
    if (v->vp_.has_value())
      problem.AddResidualBlock(
        VanishPointFactor::create(v->vp_.value(), vp_factor_gain_), loss, v->q_.data());
    // horizontal-coost
    problem.AddResidualBlock(HorizonFactor::create(vertical, hz_factor_gain_), loss, v->q_.data());

    if (i == 0) continue;

    // imu-cost
    Vertex::Ptr & v_prev = vertices_.at(i - 1);
    problem.AddResidualBlock(
      ImuFactor::create(v->dR_, imu_factor_gain_), nullptr, v_prev->q_.data(), v->q_.data());
  }

  if (verbose_) {
    std::cout << "=== BEFORE ===" << std::endl;
    printEvaluation();
  }

  // Fix first vertix (I believe we should anchor it)
  problem.SetParameterBlockConstant(vertices_.front()->q_.data());

  // Solve opmization problem
  Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  // Save after-loss infomation
  if (verbose_) {
    std::cout << "=== AFTER ===" << std::endl;
    printEvaluation();
    std::cout << summary.BriefReport() << std::endl;
  }

  return vertices_.back()->so3f();
}

void Optimizer::printEvaluation() const
{
  // Evaluate before status
  auto vp0 = vertices_.front();
  float cost_vp0 = 0;
  if (vp0->vp_.has_value()) cost_vp0 = VanishPointFactor::eval(vp0->vp_.value(), vp0->q_.data());
  float cost_hz0 = HorizonFactor::eval(Eigen::Vector2f::UnitY(), vp0->q_.data());

  if (vertices_.size() < 2) return;
  auto vp1 = vertices_.at(1);
  float cost_vp1 = 0;
  if (vp1->vp_.has_value()) cost_vp1 = VanishPointFactor::eval(vp1->vp_.value(), vp0->q_.data());
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
  problem.AddResidualBlock(VanishPointFactor::create(vp, 1.0), nullptr, q.data());

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
  problem.AddResidualBlock(VanishPointFactor::create(vp, 1.0), nullptr, q.data());
  problem.AddResidualBlock(HorizonFactor::create(vertical, 1.0), nullptr, q.data());

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  Eigen::Quaternionf qf;
  qf.coeffs() = q.cast<float>();
  return Sophus::SO3f(qf);
}

Sophus::SO3f extractNominalRotation(const Sophus::SO3f & R_base, const Sophus::SO3f & R)
{
  // DEBUG:
  static bool first = true;

  Eigen::Vector3f tmp_rx = R_base.matrix().col(0);
  Eigen::Vector3f rz = R.matrix().col(2);
  Eigen::Vector3f ry = (rz.cross(tmp_rx)).normalized();
  Eigen::Vector3f rx = ry.cross(rz);

  Eigen::Matrix3f tmp;
  tmp.col(0) = rx;
  tmp.col(1) = ry;
  tmp.col(2) = rz;
  Sophus::SO3f normalized(tmp);

  if (first) {
    std::cout << "static tf" << std::endl;
    std::cout << R_base.matrix() << std::endl;
    std::cout << "optimized tf" << std::endl;
    std::cout << R.matrix() << std::endl;
    std::cout << "normalized" << std::endl;
    std::cout << normalized.matrix() << std::endl;
    first = false;
  }

  // NOTE:
  // I do not know which one is correct.
  // I guess C is true one
  auto A = R_base.inverse() * normalized;
  auto B = R_base * normalized.inverse();
  auto C = normalized * R_base.inverse();  // = B.transpose()
  auto D = normalized.inverse() * R_base;  // = A.transpose()

  return C;
}

}  // namespace imgproc::opt