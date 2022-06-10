#pragma once
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace trajectory
{
class TwistEstimator : public rclcpp::Node
{
public:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using Imu = sensor_msgs::msg::Imu;
  using NavPVT = ublox_msgs::msg::NavPVT;

  TwistEstimator();

private:
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;

  rclcpp::Subscription<NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;

  void callbackImu(const Imu & msg);
  void callbackTwistStamped(const TwistStamped & msg);
  void callbackNavPVT(const NavPVT & msg);
  void callbackNavSatFix(const NavSatFix & msg);
};
}  // namespace trajectory