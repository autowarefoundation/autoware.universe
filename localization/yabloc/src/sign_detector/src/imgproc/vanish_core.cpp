#include "common/util.hpp"
#include "imgproc/vanish.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <random>

namespace impgroc
{
VanishPoint::VanishPoint() : Node("vanish_point")
{
  using std::placeholders::_1;
  auto cb_image = std::bind(&VanishPoint::callbackImage, this, _1);
  sub_image_ =
    create_subscription<Image>("/sensing/camera/traffic_light/image_raw/compressed", 10, cb_image);

  using cv::lsd::createLineSegmentDetector;
  lsd_ = createLineSegmentDetector(cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

void VanishPoint::callbackImage(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
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
  cv::circle(image, toCvPoint(vanish), 20, cv::Scalar(0, 255, 0), -1, cv::LINE_8);

  cv::imshow("lsd", image);
  cv::waitKey(1);
  RCLCPP_INFO_STREAM(get_logger(), "nice vanish point");
}

Eigen::Vector2f VanishPoint::estimateVanishPoint(
  const Vec3Vec & horizontals, const Vec3Vec & mid_and_theta, const cv::Mat & image) const
{
  assert(mid_and_theta.size() != horizontals.size());

  constexpr int max_iteration = 100;
  constexpr int sample_count = 6;  // must be equal or larther than 2
  const float error_threshold = std::cos(5 * 3.14 / 180);

  using CoeffMatrix = Eigen::Matrix<float, sample_count, 2>;
  using CoeffVector = Eigen::Matrix<float, sample_count, 1>;

  std::random_device seed_gen;
  std::mt19937 engine{seed_gen()};
  Eigen::Vector2f best_candidate;
  float best_inliers_count = 0;

  const int N = horizontals.size();

  auto drawLine = [image](const Eigen::Vector3f & abc) -> void {
    const int W = image.cols;
    cv::Point2i p1(0, -abc.z() / (abc.y() + 1e-4f));
    cv::Point2i p2(W, (-abc.x() * W - abc.z()) / (abc.y() + 1e-4f));
    cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 1);
  };

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

      if (itr == 0) drawLine(samples.at(i));
    }
    Eigen::VectorXf candidate = A.householderQr().solve(b);
    Eigen::Vector2f candidate2(candidate.x(), candidate.y());
    Eigen::Vector3f candidate3(candidate.x(), candidate.y(), 1);

    int inliers_count = 0;
    for (int i = 0; i < N; i++) {
      const Eigen::Vector2f mid = mid_and_theta.at(i).topRows(2);
      const float theta = mid_and_theta.at(i).z();

      Eigen::Vector2f tangent(std::cos(theta), std::sin(theta));
      Eigen::Vector2f target = (candidate2 - mid);
      tangent.normalize();
      target.normalize();
      if (std::abs(target.dot(tangent)) < error_threshold) inliers_count++;
    }

    if (best_inliers_count < inliers_count) {
      best_inliers_count = inliers_count;
      best_candidate = candidate;
    }
  }

  RCLCPP_INFO_STREAM(
    get_logger(), "best_inliers_count " << best_inliers_count << " " << best_candidate.transpose());
  return best_candidate;
}

}  // namespace impgroc