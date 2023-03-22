#pragma once
#include "camera_pose_initializer/lane_image.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ground_msgs/srv/ground.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace pcdless
{
class CameraPoseInitializer : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Marker = visualization_msgs::msg::Marker;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Ground = ground_msgs::srv::Ground;
  using Image = sensor_msgs::msg::Image;
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const cv::Point2i &)>;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

  CameraPoseInitializer();

private:
  const Eigen::Vector2d cov_xx_yy_;
  LaneImage::SharedPtr lane_image_{nullptr};
  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  rclcpp::Client<Ground>::SharedPtr ground_cli_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  std::optional<Image::ConstSharedPtr> latest_image_msg_{std::nullopt};
  ProjectFunc project_func_ = nullptr;

  cv::Point2i to_cv_point(const Eigen::Vector3f & v) const
  {
    const float image_size_ = 800;
    const float max_range_ = 20;

    cv::Point pt;
    pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
    pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
    return pt;
  }
  void on_ll2(const PointCloud2 & msg);
  void on_map(const HADMapBin & msg);
  void on_initial_pose(const PoseCovStamped & initialpose);

  cv::Mat create_vectormap_image(const Eigen::Vector3f & position);
  cv::Mat project_image();
  bool estimate_pose(const Eigen::Vector3f & position, Eigen::Vector3f & tangent);

  bool define_project_func();

  void publish_rectified_initial_pose(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent, const rclcpp::Time & stamp);
};
}  // namespace pcdless