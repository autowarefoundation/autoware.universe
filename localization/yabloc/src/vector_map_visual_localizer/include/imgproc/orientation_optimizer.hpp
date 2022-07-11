#pragma once
#include "imgproc/opt_factor.hpp"

#include <sophus/geometry.hpp>

#include <boost/circular_buffer.hpp>

namespace imgproc::opt
{
Sophus::SO3f optimizeOnce(const Sophus::SO3f & R, const Eigen::Vector3f & vp);
Sophus::SO3f optimizeOnce(
  const Sophus::SO3f & R, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical);

class Optimizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Optimizer(int max_vertex_size = 5) : vertices_(max_vertex_size) {}

  Sophus::SO3f optimize(
    const Sophus::SO3f & dR, const Eigen::Vector3f & vp, const Eigen::Vector2f & vertical,
    const Sophus::SO3f & initial_R);

  std::vector<Sophus::SO3f> allRotations() const;

private:
  boost::circular_buffer<Vertex::Ptr> vertices_;
};

}  // namespace imgproc::opt