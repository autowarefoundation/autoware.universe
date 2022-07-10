#pragma once
#include <eigen3/Eigen/StdVector>
#include <lsd/lsd.hpp>

#include <random>

namespace imgproc
{
class RansacVanishPoint
{
public:
  using Vec3Vec = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;

  RansacVanishPoint();

  cv::Point2f operator()(const cv::Mat & image);
  cv::Point2f estimate(const cv::Mat & line_segments);

private:
  mutable std::random_device seed_gen_;
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd_{nullptr};

  Eigen::Vector2f estimateVerticalLine(const Vec3Vec & verticals) const;
  Eigen::Vector2f estimateVanishPoint(
    const Vec3Vec & horizontals, const Vec3Vec & mid_and_theta) const;
};
}  // namespace imgproc