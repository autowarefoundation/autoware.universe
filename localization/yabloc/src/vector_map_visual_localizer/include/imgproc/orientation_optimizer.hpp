#pragma once
#include "imgproc/opt_factor.hpp"

#include <rclcpp/node.hpp>
#include <sophus/geometry.hpp>

#include <boost/circular_buffer.hpp>

namespace imgproc::opt
{
Sophus::SO3f optimizeOnce(const Sophus::SO3f & R, const Eigen::Vector3f & vp);
Sophus::SO3f optimizeOnce(
  const Sophus::SO3f & R, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical);

Sophus::SO3f extractNominalRotation(const Sophus::SO3f & R_base, const Sophus::SO3f & R);

class Optimizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using OptVector3f = std::optional<Eigen::Vector3f>;
  using Ptr = std::shared_ptr<Optimizer>;

  Optimizer(rclcpp::Node * node, bool verbose = false);

  Sophus::SO3f optimize(
    const Sophus::SO3f & dR, const OptVector3f & vp, const Eigen::Vector2f & vertical,
    const Sophus::SO3f & initial_R);

  std::vector<Sophus::SO3f> allRotations() const;

private:
  const bool verbose_;
  boost::circular_buffer<Vertex::Ptr> vertices_;
  const float imu_factor_gain_;
  const float vp_factor_gain_;
  const float hz_factor_gain_;

  void printEvaluation() const;
};

}  // namespace imgproc::opt