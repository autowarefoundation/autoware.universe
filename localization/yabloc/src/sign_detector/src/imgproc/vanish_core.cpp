#include "common/util.hpp"
#include "imgproc/vanish.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
VanishPoint::VanishPoint() : Node("vanish_point"), tf_subscriber_(get_clock())
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

void VanishPoint::callbackImu(const Imu & msg) { imu_buffer_.push_back(msg); }

void VanishPoint::integral(const rclcpp::Time & image_stamp)
{
  auto opt_ex =
    tf_subscriber_("traffic_light_left_camera/camera_optical_link", "tamagawa/imu_link");
  if (!opt_ex.has_value()) return;
  Sophus::SO3f ex_rot(opt_ex->rotation());

  while (!imu_buffer_.empty()) {
    Imu msg = imu_buffer_.front();
    rclcpp::Time stamp = msg.header.stamp;
    if (stamp > image_stamp) break;

    if (!last_imu_stamp_.has_value()) {
      last_imu_stamp_ = stamp;
      continue;
    }
    // from here
    const float dt = (stamp - last_imu_stamp_.value()).seconds();

    Eigen::Vector3f w;
    w << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    rotation_ *= Sophus::SO3f::exp(ex_rot * w * dt);

    // until here
    imu_buffer_.pop_front();
    last_imu_stamp_ = stamp;
  }
}

void VanishPoint::callbackImage(const Image & msg)
{
  auto opt_imu_ex =
    tf_subscriber_("traffic_light_left_camera/camera_optical_link", "tamagawa/imu_link");
  auto opt_camera_ex = tf_subscriber_("traffic_light_left_camera/camera_optical_link", "base_link");

  if (!opt_imu_ex.has_value()) return;
  if (!opt_camera_ex.has_value()) return;
  if (!info_.has_value()) return;

  integral(msg.header.stamp);

  cv::Mat image = util::decompress2CvMat(msg);
  cv::Point2f vanish = ransac_vanish_point_(image);

  // Measurement update of Kalman Filter

  // Visualize estimated vanishing point
  Sophus::SO3f rot = Sophus::SO3f(opt_camera_ex->rotation()) * rotation_;
  Eigen::Vector3f normal = rot * Eigen::Vector3f::UnitZ();

  const int W = info_->width;
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Matrix3f Kinv = K.inverse();
  Eigen::Vector3f abc = Kinv.transpose() * normal;
  float v1 = -abc.z() / abc.y();
  float v2 = (-abc.x() * W - abc.z()) / abc.y();
  std::cout << v1 << " " << v2 << std::endl;
  cv::line(image, cv::Point(v1, 0), cv::Point2i(v2, W), cv::Scalar(0, 255, 0), 1);

  // Pure measurement vanishing point
  cv::circle(image, vanish, 5, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
  cv::circle(image, vanish, 20, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
  cv::imshow("lsd", image);
  cv::waitKey(1);

  RCLCPP_INFO_STREAM(get_logger(), "nice vanish point " << vanish);
}

}  // namespace imgproc