#include "common/util.hpp"
#include "imgproc/vanish.hpp"

#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

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

  using Vec3Vec = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
  Vec3Vec lines;
  Vec3Vec horizontal, vertical;

  auto toCvPoint = [](const Eigen::Vector2f & v) { return cv::Point(v.x(), v.y()); };

  for (int i = 0; i < line_segments.rows; i++) {
    cv::Mat xy_xy = line_segments.row(i);
    Eigen::Vector2f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1);
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3);

    Eigen::Vector2f normal = xy1 - xy2;
    Eigen::Vector3f abc;
    abc << normal(1), -normal(0), -normal.dot(xy1);
    lines.push_back(abc);

    float dot_vertical = abc.dot(Eigen::Vector3f::UnitY());
    float dot_horizontal = abc.dot(Eigen::Vector3f::UnitX());
    bool is_vertical = std::abs(dot_vertical) < std::abs(dot_horizontal);

    cv::Scalar color;
    if (is_vertical) {
      vertical.push_back(abc);
      color = cv::Scalar(0, 0, 255);
    } else {
      horizontal.push_back(abc);
      color = cv::Scalar(255, 0, 0);
    }
    cv::line(image, toCvPoint(xy1), toCvPoint(xy2), color, 2, cv::LINE_8);
  }

  cv::imshow("lsd", image);
  cv::waitKey(1);
  RCLCPP_INFO_STREAM(get_logger(), "nice vanish point");
}

}  // namespace impgroc