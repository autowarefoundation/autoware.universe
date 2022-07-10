#pragma once
#include <eigen3/Eigen/StdVector>
#include <lsd/lsd.hpp>

#include <random>

namespace imgproc
{
struct LineSegment
{
  LineSegment(const cv::Mat & line)
  {
    from_ << line.at<float>(0), line.at<float>(1);
    to_ << line.at<float>(2), line.at<float>(3);

    Eigen::Vector2f tangent = from_ - to_;
    Eigen::Vector2f normal(tangent(1), -tangent(0));
    abc_ << normal(0), normal(1), -normal.dot(from_);

    float dot_horizontal = abc_.dot(Eigen::Vector3f::UnitY());
    float dot_vertical = abc_.dot(Eigen::Vector3f::UnitX());

    is_horizontal_ = std::abs(dot_vertical) * 0.7 < std::abs(dot_horizontal);

    mid_ = (from_ + to_) / 2;
    theta_ = std::atan2(tangent.y(), tangent.x());
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float theta_;
  Eigen::Vector2f mid_;
  Eigen::Vector2f from_, to_;
  Eigen::Vector3f abc_;
  bool is_horizontal_;
};

class RansacVanishPoint
{
public:
  using Vec3Vec = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
  using SegmentVec = std::vector<LineSegment>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RansacVanishPoint();

  cv::Point2f operator()(const cv::Mat & image);
  cv::Point2f estimate(const cv::Mat & line_segments);

  void drawActiveLines(const cv::Mat & image) const;

private:
  mutable std::random_device seed_gen_;
  SegmentVec last_horizontals_;
  std::vector<int> last_inlier_horizontal_indices_;

  cv::Ptr<cv::lsd::LineSegmentDetector> lsd_{nullptr};

  Eigen::Vector2f estimateVanishPoint(const SegmentVec & horizontals);
};
}  // namespace imgproc