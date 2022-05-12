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

  void infoCallback(const sensor_msgs::msg::CameraInfo& msg)
  {
    info_ = msg;
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg);

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp);


  void trackOptFlow(const cv::Mat& src_image)
  {
    cv::Mat show_image = src_image.clone();
    cv::Mat gray_image;
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);

    if (!last_gray_image.has_value()) {
      cv::goodFeaturesToTrack(gray_image, p0, 100, 0.3, 7, cv::Mat{}, 7, false, 0.04);
      last_gray_image = gray_image;
      std::cout << " p0: " << p0.size() << std::endl;
      return;
    }

    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 10, 0.03);
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> p1;
    cv::calcOpticalFlowPyrLK(last_gray_image.value(), gray_image, p0, p1, status, err, cv::Size(30, 30), 2, criteria);

    std::cout << " p1: " << p1.size() << std::endl;
    std::vector<cv::Point2f> good_new;
    for (uint i = 0; i < p0.size(); i++) {
      if (status[i] == 1) {
        cv::line(show_image, p1[i], p0[i], cv::Scalar(0, 255, 255), 2);
        cv::circle(show_image, p1[i], 5, cv::Scalar(0, 255, 255), -1);
      }
    }
    cv::imshow("optflow", show_image);
    cv::waitKey(1);

    p0 = p1;
    last_gray_image = gray_image;
  }
};