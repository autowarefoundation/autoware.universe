#pragma once
#include "common/ground_plane.hpp"
#include "common/static_tf_subscriber.hpp"
#include "common/timer.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace validation
{
class Overlay : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Marker = visualization_msgs::msg::Marker;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  using Float32Array = std_msgs::msg::Float32MultiArray;

  Overlay();

private:
  common::StaticTfSubscriber tf_subscriber_;
  common::GroundPlane ground_plane_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_sign_board_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_ground_plane_;

  rclcpp::Publisher<Marker>::SharedPtr pub_vis_;

  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  LineSegments ll2_cloud_, sign_board_;
  boost::circular_buffer<PoseStamped> pose_buffer_;

  void infoCallback(const CameraInfo & msg);
  void imageCallback(const Image & msg);
  void lsdCallback(const PointCloud2 & msg);

  LineSegments extractNaerLineSegments(const Pose & pose);

  void drawOverlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp);
  void drawOverlaySignBoard(cv::Mat & image, const Pose & pose);
  void drawOverlayRoadMark(cv::Mat & image, const Pose & pose);

  void makeVisMarker(const LineSegments & ls, const Pose & pose, const rclcpp::Time & stamp);

  Eigen::Affine3f poseConsideringSlope(const Eigen::Affine3f & pose) const;
};
}  // namespace validation