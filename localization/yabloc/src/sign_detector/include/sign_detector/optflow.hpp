#pragma once
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/video.hpp>
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
  std::optional<sensor_msgs::msg::CameraInfo> info_{std::nullopt};
  std::optional<cv::Mat> last_gray_image{std::nullopt};
  std::vector<cv::Point2f> p0;
  std::optional<cv::Mat> scaled_intrinsic{std::nullopt};

  void infoCallback(const sensor_msgs::msg::CameraInfo& msg)
  {
    info_ = msg;
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg);

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp);

  void computeRotation(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2);

  void trackOptFlow(const cv::Mat& src_image);
};