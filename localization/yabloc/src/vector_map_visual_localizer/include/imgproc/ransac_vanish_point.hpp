#pragma once
#include <eigen3/Eigen/StdVector>
#include <lsd/lsd.hpp>

namespace imgproc
{
class RansacVanishPoint
{
public:
  using Vec3Vec = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;

  RansacVanishPoint();

  cv::Point2f operator()(const cv::Mat & image);

private:
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd_{nullptr};

  Eigen::Vector2f estimateVanishPoint(
    const Vec3Vec & horizontals, const Vec3Vec & mid_and_theta, const cv::Mat & image) const;
};
}  // namespace imgproc