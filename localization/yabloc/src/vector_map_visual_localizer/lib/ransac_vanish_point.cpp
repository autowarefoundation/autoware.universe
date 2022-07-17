#include "imgproc/ransac_vanish_point.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <iostream>

namespace imgproc
{
RansacVanishPoint::RansacVanishPoint(rclcpp::Node * node)
: param_{
    node->declare_parameter<int>("max_iteration", 500),
    node->declare_parameter<int>("sample_count", 4),
    node->declare_parameter<float>("inlier_ratio", 0.2f),
    node->declare_parameter<float>("error_threshold", 0.996f),
    node->declare_parameter<float>("lsd_sigma", 1.5f)}
{
  using cv::lsd::createLineSegmentDetector;
  lsd_ = createLineSegmentDetector(
    cv::lsd::LSD_REFINE_STD, 0.8, param_.lsd_sigma_, 2.0, 22.5, 0, 0.7, 1024);
}

void RansacVanishPoint::drawActiveLines(const cv::Mat & image) const
{
  cv::Mat overlay = cv::Mat::zeros(image.size(), CV_8UC3);

  auto drawWholeLineSegments = [&overlay](SegmentVec line_segments) -> void {
    for (const LineSegment & ls : line_segments) {
      cv::Point2i from(ls.from_.x(), ls.from_.y());
      cv::Point2i to(ls.to_.x(), ls.to_.y());
      auto color = (ls.is_diagonal_) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 165, 255);
      cv::line(overlay, from, to, color, 1);
    }
  };
  drawWholeLineSegments(last_diagonal_);
  drawWholeLineSegments(last_perpendicular_);

  cv::addWeighted(image, 0.8, overlay, 0.5, 1.0, image);
  for (int index : last_inlier_horizontal_indices_) {
    const auto & segment = last_diagonal_.at(index);
    cv::Point2i from(segment.from_.x(), segment.from_.y());
    cv::Point2i to(segment.to_.x(), segment.to_.y());
    cv::line(image, from, to, cv::Scalar(0, 0, 255), 2);
  }
}

std::optional<cv::Point2f> RansacVanishPoint::estimate(const cv::Mat & line_segments)
{
  SegmentVec perpendicular, diagonal;

  for (int i = 0; i < line_segments.rows; i++) {
    LineSegment segment(line_segments.row(i));
    if (segment.is_diagonal_) {
      diagonal.push_back(segment);
    } else {
      perpendicular.push_back(segment);
    }
  }

  last_diagonal_ = diagonal;
  last_perpendicular_ = perpendicular;

  return estimateVanishPoint(diagonal);
}

std::optional<cv::Point2f> RansacVanishPoint::operator()(const cv::Mat & image)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat line_segments;
  lsd_->detect(gray_image, line_segments);
  return estimate(line_segments);
}

std::optional<cv::Point2f> RansacVanishPoint::estimateVanishPoint(const SegmentVec & diagonals)
{
  std::mt19937 engine{seed_gen_()};

  Eigen::Vector2f best_candidate = Eigen::Vector2f::Zero();
  float best_residual = std::numeric_limits<float>::max();

  const int N = diagonals.size();

  last_inlier_horizontal_indices_.clear();

  for (int itr = 0; itr < param_.max_iteration_; itr++) {
    SegmentVec samples;
    std::sample(
      diagonals.begin(), diagonals.end(), std::back_inserter(samples), param_.sample_count_,
      engine);

    Eigen::MatrixXf A_sample(param_.sample_count_, 2);
    Eigen::MatrixXf b_sample(param_.sample_count_, 1);
    for (int cnt = 0; cnt < param_.sample_count_; cnt++) {
      A_sample(cnt, 0) = samples.at(cnt).abc_.x();
      A_sample(cnt, 1) = samples.at(cnt).abc_.y();
      b_sample(cnt) = -samples.at(cnt).abc_.z();
    }
    Eigen::VectorXf candidate = A_sample.householderQr().solve(b_sample);
    Eigen::Vector2f candidate2(candidate.x(), candidate.y());

    SegmentVec inliers;
    std::vector<int> inlier_indices;
    for (int i = 0; i < N; i++) {
      const LineSegment & segment = diagonals.at(i);
      const Eigen::Vector2f mid = segment.mid_;
      const float theta = segment.theta_;

      Eigen::Vector2f tangent(std::cos(theta), std::sin(theta));
      Eigen::Vector2f target = (candidate2 - mid);
      tangent.normalize();
      target.normalize();
      if (std::abs(target.dot(tangent)) > param_.error_threshold_) {
        inliers.push_back(segment);
        inlier_indices.push_back(i);
      }
    }

    if (inliers.size() < param_.inlier_ratio_ * N) continue;

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
  if (last_inlier_horizontal_indices_.empty()) return std::nullopt;

  return cv::Point2f(best_candidate.x(), best_candidate.y());
}

}  // namespace imgproc