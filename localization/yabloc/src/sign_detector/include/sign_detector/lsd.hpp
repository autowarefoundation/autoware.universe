#pragma once
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <lsd/lsd.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/hfs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ximgproc/segmentation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <optional>

class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LineSegmentDetector(const std::string & image_topic, const std::string & info_topic)
  : Node("line_detector"),
    info_(std::nullopt),
    line_thick_(declare_parameter<int>("line_thick", 1)),
    image_size_(declare_parameter<int>("image_size", 800)),
    max_range_(declare_parameter<float>("max_range", 20.f)),
    length_threshold_(declare_parameter<float>("length_threshold", 2.0f)),
    dilate_size_(declare_parameter<int>("dilate_size", 5))
  {
    // Subscriber
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic, rclcpp::QoS(10).durability_volatile().best_effort(),
      std::bind(&LineSegmentDetector::imageCallback, this, std::placeholders::_1));
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::QoS(10).durability_volatile().best_effort(),
      std::bind(&LineSegmentDetector::infoCallback, this, std::placeholders::_1));

    // Publisher
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/projected_image", 10);
    pub_image_lsd_ = this->create_publisher<sensor_msgs::msg::Image>("/lsd_image", 10);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lsd_cloud", 10);

    lsd = cv::lsd::createLineSegmentDetector(
      cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_, pub_image_lsd_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  const int line_thick_;
  const int image_size_;
  const float max_range_;
  const float length_threshold_;
  cv::Ptr<cv::hfs::HfsSegment> segmentator;
  cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation> gs;
  const int dilate_size_;

  void directLineSegment(const cv::Mat & image, const cv::Mat & lines) const;

  void projectEdgeOnPlane(
    const cv::Mat & lines, const cv::Mat & K_cv, const rclcpp::Time & stamp,
    const cv::Mat & mask) const;

  void listenExtrinsicTf(const std::string & frame_id);

  void infoCallback(const sensor_msgs::msg::CameraInfo & msg)
  {
    info_ = msg;
    listenExtrinsicTf(info_->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage & msg);
  void publishCloud(
    const pcl::PointCloud<pcl::PointNormal> & cloud, const rclcpp::Time & stamp) const;
  void publishImage(const cv::Mat & image, const rclcpp::Time & stamp) const;

  cv::Mat segmentationHfs(const cv::Mat & image);
  cv::Mat segmentationGraph(const cv::Mat & image);
};