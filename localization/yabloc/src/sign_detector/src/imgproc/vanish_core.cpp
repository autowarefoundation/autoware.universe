#include "common/util.hpp"
#include "imgproc/vanish.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
VanishPoint::VanishPoint() : Node("vanish_point")
{
  using std::placeholders::_1;
  auto cb_image = std::bind(&VanishPoint::callbackImage, this, _1);
  auto cb_imu = std::bind(&VanishPoint::callbackImu, this, _1);

  sub_imu_ = create_subscription<Imu>("/sensing/imu/tamagawa/imu_raw", 10, cb_imu);
  sub_image_ =
    create_subscription<Image>("/sensing/camera/traffic_light/image_raw/compressed", 10, cb_image);
  sub_info_ = create_subscription<CameraInfo>(
    "/sensing/camera/traffic_light/camera_info", 10,
    [this](const CameraInfo & msg) -> void { this->info_ = msg; });
}

void VanishPoint::callbackImu(const Imu & msg) {}

void VanishPoint::callbackImage(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);

  cv::Point2f vanish = ransac_vanish_point_(image);
  auto toCvPoint = [](const Eigen::Vector2f & v) { return cv::Point(v.x(), v.y()); };

  cv::circle(image, vanish, 5, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
  cv::circle(image, vanish, 20, cv::Scalar(0, 255, 0), 2, cv::LINE_8);

  cv::imshow("lsd", image);
  cv::waitKey(1);
  RCLCPP_INFO_STREAM(get_logger(), "nice vanish point " << vanish);
}

}  // namespace imgproc