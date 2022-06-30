#include "imgproc/ransac_vanish_point.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <iostream>
#include <random>

namespace imgproc
{
RansacVanishPoint::RansacVanishPoint()
{
  using cv::lsd::createLineSegmentDetector;
  lsd_ = createLineSegmentDetector(cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

cv::Point2f RansacVanishPoint::operator()(const cv::Mat & image)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat line_segments;
  lsd_->detect(gray_image, line_segments);
  // lsd_->drawSegments(gray_image, line_segments);

  Vec3Vec lines;
  Vec3Vec horizontal, vertical;
  Vec3Vec mid_and_theta;

  auto toCvPoint = [](const Eigen::Vector2f & v) { return cv::Point(v.x(), v.y()); };

  for (int i = 0; i < line_segments.rows; i++) {
    cv::Mat xy_xy = line_segments.row(i);
    Eigen::Vector2f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1);
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3);

    Eigen::Vector2f tangent = xy1 - xy2;
    Eigen::Vector2f normal(tangent(1), -tangent(0));
    Eigen::Vector3f abc;
    abc << normal(0), normal(1), -normal.dot(xy1);
    lines.push_back(abc);

    float dot_horizontal = abc.dot(Eigen::Vector3f::UnitY());
    float dot_vertical = abc.dot(Eigen::Vector3f::UnitX());
    bool is_horizontal = std::abs(dot_vertical) < std::abs(dot_horizontal);

    cv::Scalar color;
    if (is_horizontal) {
      horizontal.push_back(abc);
      Eigen::Vector2f mid = (xy1 + xy2) / 2;
      float theta = std::atan2(tangent.y(), tangent.x());
      mid_and_theta.push_back({mid.x(), mid.y(), theta});
      color = cv::Scalar(0, 0, 255);
    } else {
      vertical.push_back(abc);
      color = cv::Scalar(255, 0, 0);
    }
    cv::line(image, toCvPoint(xy1), toCvPoint(xy2), color, 1, cv::LINE_8);
  }

  Eigen::Vector2f vanish = estimateVanishPoint(horizontal, mid_and_theta, image);

  return cv::Point2f(vanish.x(), vanish.y());
}

Eigen::Vector2f RansacVanishPoint::estimateVanishPoint(
  const Vec3Vec & horizontals, const Vec3Vec & mid_and_theta, const cv::Mat & image) const
{
  assert(mid_and_theta.size() != horizontals.size());

  constexpr int max_iteration = 500;
  constexpr int sample_count = 4;  // must be equal or larther than 2
  constexpr float inlier_ratio = 0.2;
  const float error_threshold = std::cos(2 * 3.14 / 180);  // 5[deg]

  using CoeffMatrix = Eigen::Matrix<float, sample_count, 2>;
  using CoeffVector = Eigen::Matrix<float, sample_count, 1>;

  std::random_device seed_gen;
  std::mt19937 engine{seed_gen()};

  Eigen::Vector2f best_candidate;
  float best_residual;

  const int N = horizontals.size();

  auto drawLine = [image](const Eigen::Vector3f & abc) -> void {
    const int W = image.cols;
    cv::Point2i p1(0, -abc.z() / (abc.y() + 1e-4f));
    cv::Point2i p2(W, (-abc.x() * W - abc.z()) / (abc.y() + 1e-4f));
    cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 1);
  };

  int enough_inlier_samples = 0;

  for (int itr = 0; itr < max_iteration; itr++) {
    Vec3Vec samples;
    std::sample(
      horizontals.begin(), horizontals.end(), std::back_inserter(samples), sample_count, engine);

    CoeffMatrix A;
    CoeffVector b;
    for (int i = 0; i < sample_count; i++) {
      A(i, 0) = samples.at(i).x();
      A(i, 1) = samples.at(i).y();
      b(i) = -samples.at(i).z();

      // if (itr == 0) drawLine(samples.at(i));
    }
    Eigen::VectorXf candidate = A.householderQr().solve(b);
    Eigen::Vector2f candidate2(candidate.x(), candidate.y());
    Eigen::Vector3f candidate3(candidate.x(), candidate.y(), 1);

    int inliers_count = 0;
    Vec3Vec inliers;
    for (int i = 0; i < N; i++) {
      const Eigen::Vector2f mid = mid_and_theta.at(i).topRows(2);
      const float theta = mid_and_theta.at(i).z();

      Eigen::Vector2f tangent(std::cos(theta), std::sin(theta));
      Eigen::Vector2f target = (candidate2 - mid);
      tangent.normalize();
      target.normalize();
      if (std::abs(target.dot(tangent)) > error_threshold) {
        inliers.push_back(horizontals.at(i));
      }
    }

    if (inliers.size() < inlier_ratio * N) continue;
    enough_inlier_samples++;

    {
      Eigen::MatrixXf A(inliers.size(), 2);
      Eigen::MatrixXf b(inliers.size(), 1);
      for (int i = 0; i < inliers.size(); i++) {
        A(i, 0) = inliers.at(i).x();
        A(i, 1) = inliers.at(i).y();
        b(i) = -inliers.at(i).z();
      }
      Eigen::VectorXf new_candidate = A.householderQr().solve(b);
      float residual = (A * new_candidate - b).squaredNorm() / inliers.size();

      if (residual < best_residual) {
        best_residual = residual;
        best_candidate = new_candidate;
      }
    }

    best_candidate = candidate;
  }

  std::cout << " enough " << enough_inlier_samples << " " << N << std::endl;
  return best_candidate;
}

}  // namespace imgproc