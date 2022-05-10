#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <lsd/lsd.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <optional>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/core/eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LineSegmentDetector(const std::string& image_topic, const std::string& info_topic) : Node("line_detector"), info_(std::nullopt), line_thick_(this->declare_parameter<int>("line_thick", 1))
  {
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, 10, std::bind(&LineSegmentDetector::imageCallback, this, std::placeholders::_1));
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10, std::bind(&LineSegmentDetector::infoCallback, this, std::placeholders::_1));
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);
    pub_image_lsd_ = this->create_publisher<sensor_msgs::msg::Image>("/lsd_image", 10);
    lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    image_size_ = this->declare_parameter<int>("image_size", 800);
    max_range_ = this->declare_parameter<float>("max_range", 20.f);
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};

  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_, pub_image_lsd_;
  const int line_thick_;
  int image_size_;
  float max_range_;

  void projectEdgeOnPlane(const cv::Mat& lines, const cv::Mat& K_cv, const rclcpp::Time& stamp) const;

  void listenExtrinsicTf(const std::string& frame_id);

  void infoCallback(const sensor_msgs::msg::CameraInfo& msg)
  {
    info_ = msg;
    listenExtrinsicTf(info_->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg) const;

  sensor_msgs::msg::Image::ConstSharedPtr decompressImage(const sensor_msgs::msg::CompressedImage& compressed_img) const;
};