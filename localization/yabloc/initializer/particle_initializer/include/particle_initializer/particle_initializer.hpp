#pragma once
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace pcdless::modularized_particle_filter
{
class ParticleInitializer : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Marker = visualization_msgs::msg::Marker;
  using SetBool = std_srvs::srv::SetBool;

  ParticleInitializer();

private:
  const Eigen::Vector2d cov_xx_yy_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_initialpose_ekf_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  rclcpp::Client<SetBool>::SharedPtr client_ekf_trigger_;

  void on_initial_pose(const PoseCovStamped & initialpose);

  void publish_range_marker(const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent);
  void publish_rectified_initial_pose(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
    const PoseCovStamped & raw_initialpose);
};
}  // namespace pcdless::modularized_particle_filter