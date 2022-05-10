#include "sign_detector/util.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>


class OptFlowNode : public rclcpp::Node
{
public:
  OptFlowNode(const std::string& image_topic, const std::string& info_topic) : Node("optflow_image")
  {
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, 10, std::bind(&OptFlowNode::imageCallback, this, std::placeholders::_1));
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10, std::bind(&OptFlowNode::infoCallback, this, std::placeholders::_1));
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/flow_image", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  std::optional<sensor_msgs::msg::CameraInfo> info_;

  void infoCallback(const sensor_msgs::msg::CameraInfo& msg)
  {
    info_ = msg;
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg)
  {
    sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);
    cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
    if (!info_.has_value()) return;
    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void*)(info_->k.data()));
    cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void*)(info_->d.data()));
    cv::Mat undistorted;
    cv::undistort(image, undistorted, K, D, K);
    image = undistorted;

    cv::Size size = image.size();
    const int WIDTH = 800;
    const float SCALE = 1.0f * WIDTH / size.width;
    int HEIGHT = SCALE * size.height;
    cv::resize(image, image, cv::Size(WIDTH, HEIGHT));

    // TODO:

    publishImage(image, this->get_clock()->now());
  }

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp)
  {
    cv_bridge::CvImage raw_image;
    raw_image.header.stamp = stamp;
    raw_image.header.frame_id = "map";
    raw_image.encoding = "bgr8";
    raw_image.image = image;
    pub_image_->publish(*raw_image.toImageMsg());
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const std::string image_topic = "/sensing/camera/traffic_light/image_raw/compressed";
  const std::string info_topic = "/sensing/camera/traffic_light/camera_info";

  rclcpp::spin(std::make_shared<OptFlowNode>(image_topic, info_topic));
  rclcpp::shutdown();
  return 0;
}