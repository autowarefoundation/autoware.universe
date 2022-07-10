#include "imgproc/ransac_vanish_point.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <iostream>

namespace imgproc
{
RansacVanishPoint::RansacVanishPoint()
{
  using cv::lsd::createLineSegmentDetector;
  lsd_ = createLineSegmentDetector(cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

void RansacVanishPoint::drawActiveLines(const cv::Mat & image) const
{
  for (int index : last_inlier_horizontal_indices_) {
    const auto & segment = last_horizontals_.at(index);
    cv::Point2i from(segment.from_.x(), segment.from_.y());
    cv::Point2i to(segment.to_.x(), segment.to_.y());
    cv::line(image, from, to, cv::Scalar(0, 0, 255), 2);
  }
}

cv::Point2f RansacVanishPoint::estimate(const cv::Mat & line_segments)
{
  SegmentVec horizontals, verticals;

  auto toCvPoint = [](const Eigen::Vector2f & v) { return cv::Point(v.x(), v.y()); };

  for (int i = 0; i < line_segments.rows; i++) {
    LineSegment segment(line_segments.row(i));
    if (segment.is_horizontal_) {
      horizontals.push_back(segment);
    } else {
      verticals.push_back(segment);
    }
  }

  Eigen::Vector2f vanish = estimateVanishPoint(horizontals);
  last_horizontals_ = horizontals;

  return cv::Point2f(vanish.x(), vanish.y());
}

cv::Point2f RansacVanishPoint::operator()(const cv::Mat & image)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat line_segments;
  lsd_->detect(gray_image, line_segments);
  return estimate(line_segments);
}

Eigen::Vector2f RansacVanishPoint::estimateVanishPoint(const SegmentVec & horizontals)
{
  constexpr int max_iteration = 500;
  constexpr int sample_count = 4;  // must be equal or larther than 2
  constexpr float inlier_ratio = 0.2;
  const float error_threshold = std::cos(2 * 3.14 / 180);  // [deg]

  std::mt19937 engine{seed_gen_()};

  Eigen::Vector2f best_candidate;
  float best_residual = std::numeric_limits<float>::max();

  const int N = horizontals.size();

  for (int itr = 0; itr < max_iteration; itr++) {
    SegmentVec samples;
    std::sample(
      horizontals.begin(), horizontals.end(), std::back_inserter(samples), sample_count, engine);

    Eigen::MatrixXf A_sample(sample_count, 2);
    Eigen::MatrixXf b_sample(sample_count, 1);
    for (int i = 0; i < sample_count; i++) {
      A_sample(i, 0) = samples.at(i).abc_.x();
      A_sample(i, 1) = samples.at(i).abc_.y();
      b_sample(i) = -samples.at(i).abc_.z();
    }
    Eigen::VectorXf candidate = A_sample.householderQr().solve(b_sample);
    Eigen::Vector2f candidate2(candidate.x(), candidate.y());

    int inliers_count = 0;
    SegmentVec inliers;
    std::vector<int> inlier_indices;
    for (int i = 0; i < N; i++) {
      const LineSegment & segment = horizontals.at(i);
      const Eigen::Vector2f mid = segment.mid_;
      const float theta = segment.theta_;

      Eigen::Vector2f tangent(std::cos(theta), std::sin(theta));
      Eigen::Vector2f target = (candidate2 - mid);
      tangent.normalize();
      target.normalize();
      if (std::abs(target.dot(tangent)) > error_threshold) {
        inliers.push_back(segment);
        inlier_indices.push_back(i);
      }
    }

    if (inliers.size() < inlier_ratio * N) continue;

    Eigen::MatrixXf A_inlier(inliers.size(), 2);
    Eigen::MatrixXf b_inlier(inliers.size(), 1);
    for (int i = 0; i < inliers.size(); i++) {
      A_inlier(i, 0) = inliers.at(i).abc_.x();
      A_inlier(i, 1) = inliers.at(i).abc_.y();
      b_inlier(i) = -inliers.at(i).abc_.z();
    }
    Eigen::VectorXf new_candidate = A_inlier.householderQr().solve(b_inlier);
    float residual = (A_inlier * new_candidate - b_inlier).squaredNorm() / inliers.size();

    if (residual < best_residual) {
      best_residual = residual;
      best_candidate = new_candidate;
      last_inlier_horizontal_indices_ = inlier_indices;
    }
  }

  return best_candidate;
}

}  // namespace imgproc