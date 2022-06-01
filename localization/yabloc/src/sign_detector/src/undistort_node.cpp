#include "sign_detector/timer.hpp"
#include "sign_detector/util.hpp"

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class UndistortNode : public rclcpp::Node
{
public:
  UndistortNode() : Node("undistort"), WIDTH(declare_parameter("width", 800))
  {
    rclcpp::QoS qos = rclcpp::QoS(10);
    // rclcpp::QoS qos = rclcpp::QoS(10).durability_volatile().best_effort();

    sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/sensing/camera/traffic_light/image_raw/compressed", qos,
      std::bind(&UndistortNode::imageCallback, this, std::placeholders::_1));
    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/sensing/camera/traffic_light/camera_info", qos,
      std::bind(&UndistortNode::infoCallback, this, std::placeholders::_1));

    pub_info_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("/sensing/camera/undistorted/camera_info", 10);
    pub_image_ =
      create_publisher<sensor_msgs::msg::Image>("/sensing/camera/undistorted/image_raw", 10);
  }

private:
  const int WIDTH;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  std::optional<sensor_msgs::msg::CameraInfo> info_{std::nullopt}, scaled_info_{std::nullopt};

  void imageCallback(const sensor_msgs::msg::CompressedImage & msg)
  {
    Timer timer;
    cv::Mat image = decompress2CvMat(msg);
    cv::Size size = image.size();

    if (!info_.has_value()) return;

    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
    cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void *)(info_->d.data()));
    cv::Mat undistorted;
    cv::undistort(image, undistorted, K, D, K);
    image = undistorted;

    const float SCALE = 1.0f * WIDTH / size.width;
    const int HEIGHT = SCALE * size.height;
    cv::resize(image, image, cv::Size(WIDTH, HEIGHT));

    publishImage(*pub_image_, image, msg.header.stamp);

    if (!scaled_info_.has_value()) buildScaledCameraInfo(K, SCALE);
    pub_info_->publish(scaled_info_.value());

    RCLCPP_INFO_STREAM(get_logger(), "decompress: " << timer.microSeconds() / 1000.f << "[ms]");
  }

  void infoCallback(const sensor_msgs::msg::CameraInfo & msg) { info_ = msg; }

  void buildScaledCameraInfo(const cv::Mat & K, float scale)
  {
    scaled_info_ = sensor_msgs::msg::CameraInfo{};
    cv::Mat k = scale * K;
    scaled_info_->k.at(0) = k.at<double>(0, 0);
    scaled_info_->k.at(2) = k.at<double>(0, 2);
    scaled_info_->k.at(4) = k.at<double>(1, 1);
    scaled_info_->k.at(5) = k.at<double>(1, 2);
    scaled_info_->k.at(8) = 1;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UndistortNode>());
  rclcpp::shutdown();
  return 0;
}
