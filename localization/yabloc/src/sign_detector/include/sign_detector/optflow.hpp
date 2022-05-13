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
    constexpr bool MUTUAL_CHECK = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "tracked points: " << p0.size());

    cv::Mat show_image = src_image.clone();
    cv::Mat gray_image;
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);

    // ================================================
    // First find good features
    if (!last_gray_image.has_value()) {
      cv::goodFeaturesToTrack(gray_image, p0, 100, 0.3, 7, cv::Mat{}, 7, false, 0.04);
      last_gray_image = gray_image;
      return;
    }

    // ================================================
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 10, 0.03);
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> p1;
    cv::calcOpticalFlowPyrLK(last_gray_image.value(), gray_image, p0, p1, status, err, cv::Size(30, 30), 2, criteria);

    std::vector<cv::Point2f> enable_p0;
    std::vector<cv::Point2f> enable_p1;
    for (uint i = 0; i < p0.size(); i++) {
      if (status[i] == 1) {
        enable_p0.push_back(p0[i]);
        enable_p1.push_back(p1[i]);
      }
    }


    if (MUTUAL_CHECK) {
      std::vector<uchar> reverse_status;
      std::vector<float> reverse_err;
      std::vector<cv::Point2f> reverse_p0;
      cv::calcOpticalFlowPyrLK(gray_image, last_gray_image.value(), enable_p1, reverse_p0, reverse_status, reverse_err, cv::Size(30, 30), 2, criteria);
      std::vector<cv::Point2f> more_enable_p0;
      std::vector<cv::Point2f> more_enable_p1;
      for (uint i = 0; i < enable_p1.size(); i++) {
        if (reverse_status[i] == 1) {
          cv::Point2f d = enable_p0[i] - reverse_p0[i];
          if (d.x * d.x + d.y * d.y < 10) {
            more_enable_p0.push_back(enable_p0[i]);
            more_enable_p1.push_back(enable_p1[i]);
          }
        }
      }
      enable_p0 = more_enable_p0;
      enable_p1 = more_enable_p1;
    }

    for (uint i = 0; i < enable_p1.size(); i++) {
      cv::line(show_image, enable_p1[i], enable_p0[i], cv::Scalar(0, 0, 255), 1);
      cv::circle(show_image, enable_p1[i], 4, cv::Scalar(0, 255, 255), 1);
    }
    cv::imshow("optflow", show_image);
    cv::waitKey(1);

    // ================================================
    // Pickup additional good features
    {
      if (enable_p1.size() < 50) {
        cv::Mat mask = cv::Mat::ones(gray_image.size(), CV_8UC1);
        for (const auto p : enable_p1) cv::circle(mask, p, 20, cv::Scalar::all(0), -1);

        std::vector<cv::Point2f> additional_p1;
        cv::goodFeaturesToTrack(gray_image, additional_p1, 50, 0.3, 7, mask, 7, false, 0.04);
        std::copy(additional_p1.begin(), additional_p1.end(), std::back_inserter(enable_p1));
      }
    }


    p0 = enable_p1;
    last_gray_image = gray_image;
  }
};