#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::ekf_corrector
{
class GnssEkfCorrector : public rclcpp::Node
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using Float32 = std_msgs::msg::Float32;

  GnssEkfCorrector();

private:
  const bool ignore_less_than_float_;

  rclcpp::Subscription<NavPVT>::SharedPtr sub_ublox_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<Float32>::SharedPtr height_sub_;
  Float32 latest_height_;

  void on_ublox(const NavPVT::ConstSharedPtr ublox_msg);
  void publish_marker(const Eigen::Vector3f & position, bool fixed);
};
}  // namespace pcdless::ekf_corrector
