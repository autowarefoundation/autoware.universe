#pragma once

#include "common/gamma_converter.hpp"
#include "common/ground_plane.hpp"
#include "common/static_tf_subscriber.hpp"
#include "common/synchro_subscriber.hpp"
#include "validation/refine_optimizer.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace validation
{
class RefineOptimizer : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  using Float32Array = std_msgs::msg::Float32MultiArray;
  using String = std_msgs::msg::String;

  RefineOptimizer();

protected:
  common::StaticTfSubscriber tf_subscriber_;
  GroundPlane ground_plane_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_ground_plane_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;

  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<Sophus::SE3f> camera_extrinsic_{std::nullopt};
  LineSegments ll2_cloud_;
  boost::circular_buffer<PoseStamped> pose_buffer_;
  SynchroSubscriber<Image, PointCloud2>::SharedPtr sub_synchro_;

  const int pixel_interval_;
  const bool show_grad_image_;
  GammaConverter gamma_converter_{5.0};
  RefineConfig opt_config_;

  void infoCallback(const CameraInfo & msg);

  void imageAndLsdCallback(const Image & image, const PointCloud2 & msg);

  LineSegments extractNaerLineSegments(
    const Sophus::SE3f & pose, const LineSegments & linesegments);

  cv::Mat makeCostMap(LineSegments & lsd);

  pcl::PointCloud<pcl::PointXYZ> sampleUniformlyOnImage(
    const Sophus::SE3f & pose, const LineSegments & segments);

  void drawOverlayPoints(
    cv::Mat & image, const Sophus::SE3f & pose_affine,
    const pcl::PointCloud<pcl::PointXYZ> & points, const cv::Scalar & color);
  void drawOverlayLineSegments(
    cv::Mat & image, const Sophus::SE3f & pose_affine, const LineSegments & linesegments,
    const cv::Scalar & color);
};
}  // namespace validation