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

    const float cos45deg = std::sqrt(2.f) / 2.f;
    const Eigen::Vector3f unit_d1 = Eigen::Vector3f(cos45deg, cos45deg, 0);
    const Eigen::Vector3f unit_d2 = Eigen::Vector3f(-cos45deg, cos45deg, 0);

    float dot_diagonal1 = std::abs(abc_.dot(unit_d1));
    float dot_diagonal2 = std::abs(abc_.dot(unit_d2));
    float dot_diagonal = std::max(dot_diagonal1, dot_diagonal2);

    float dot_horizontal = std::abs(abc_.dot(Eigen::Vector3f::UnitY()));
    float dot_vertical = std::abs(abc_.dot(Eigen::Vector3f::UnitX()));

    is_diagonal_ = (dot_diagonal > 0.9f * dot_horizontal) & (dot_diagonal > 0.9f * dot_vertical);

    mid_ = (from_ + to_) / 2.f;
    theta_ = std::atan2(tangent.y(), tangent.x());
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float theta_;
  Eigen::Vector2f mid_;
  Eigen::Vector2f from_, to_;
  Eigen::Vector3f abc_;
  bool is_diagonal_;
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
  const int max_iteration_ = 500;
  const int sample_count_ = 4;  // must be equal or larther than 2
  const float inlier_ratio_ = 0.2;
  const float error_threshold_ = std::cos(5 * 3.14 / 180);  // 5 [deg]

  mutable std::random_device seed_gen_;
  SegmentVec last_horizontals_;
  std::vector<int> last_inlier_horizontal_indices_;

  cv::Ptr<cv::lsd::LineSegmentDetector> lsd_{nullptr};

  Eigen::Vector2f estimateVanishPoint(const SegmentVec & horizontals);
};
}  // namespace imgproc