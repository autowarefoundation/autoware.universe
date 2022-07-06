#pragma once
#include "common/static_tf_subscriber.hpp"

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <lsd/lsd.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ximgproc/segmentation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <memory>
#include <optional>

namespace imgproc
{
class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  LineSegmentDetector();

private:
  const int dilate_size_;
  const int line_thick_;
  const int image_size_;
  const float max_range_;

  common::StaticTfSubscriber tf_subscriber_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_, pub_image_lsd_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;

  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  std::optional<sensor_msgs::msg::CameraInfo> info_{std::nullopt};
  cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation> gs;

  void directLineSegment(const cv::Mat & image, const cv::Mat & lines) const;

  void projectEdgeOnPlane(
    const cv::Mat & lines, const cv::Mat & K_cv, const rclcpp::Time & stamp,
    const cv::Mat & mask) const;

  void infoCallback(const sensor_msgs::msg::CameraInfo & msg)
  {
    info_ = msg;
    camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
  }

  void imageCallback(const sensor_msgs::msg::Image & msg);
  void execute(const cv::Mat & image, const rclcpp::Time & stamp);

  cv::Point toCvPoint(const Eigen::Vector3f & v) const;
  cv::Mat segmentationGraph(const cv::Mat & image);
};
}  // namespace imgproc