#pragma once

#include "common/gamma_converter.hpp"
#include "common/ground_plane.hpp"
#include "common/static_tf_subscriber.hpp"
#include "common/synchro_subscriber.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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

  RefineOptimizer();

protected:
  common::StaticTfSubscriber tf_subscriber_;
  GroundPlane ground_plane_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_ground_plane_;

  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  LineSegments ll2_cloud_;
  boost::circular_buffer<PoseStamped> pose_buffer_;
  SynchroSubscriber<Image, PointCloud2>::SharedPtr sub_synchro_;

  GammaConverter gamma_converter_{5.0};

  void infoCallback(const CameraInfo & msg);

  void imageAndLsdCallback(const Image & image, const PointCloud2 & msg);

  LineSegments extractNaerLineSegments(const Pose & pose, const LineSegments & linesegments);

  cv::Mat makeCostMap(LineSegments & lsd);

  cv::Mat drawOverlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp);
  void drawOverlayLineSegments(
    cv::Mat & image, const Pose & pose, const LineSegments & linesegments);
};
}  // namespace validation