#pragma once
#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

namespace pcdless::ground_server
{
class MovingAveraging
{
public:
  MovingAveraging() : buffer_(50) {}

  Eigen::Vector3f update(const Eigen::Vector3f & normal)
  {
    buffer_.push_back(normal);

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const Eigen::Vector3f & v : buffer_) mean += v;
    mean /= buffer_.size();

    return mean.normalized();
  }

private:
  boost::circular_buffer<Eigen::Vector3f> buffer_;
};
}  // namespace pcdless::ground_server
