#pragma once
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ground_msgs/srv/ground.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace pcdless
{
class CameraPoseInitializer : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Marker = visualization_msgs::msg::Marker;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Ground = ground_msgs::srv::Ground;

  CameraPoseInitializer();

private:
  const Eigen::Vector2d cov_xx_yy_;

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  rclcpp::Client<Ground>::SharedPtr ground_cli_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  void on_map(const HADMapBin & msg);
  void on_initial_pose(const PoseCovStamped & initialpose);

  void publish_rectified_initial_pose(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
    const PoseCovStamped & raw_initialpose);
};
}  // namespace pcdless