#pragma once
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace vml_common
{
rclcpp::Time ubloxTime2Stamp(const ublox_msgs::msg::NavPVT & msg);
ublox_msgs::msg::NavPVT stamp2UbloxTime(const builtin_interfaces::msg::Time & stamp);
}  // namespace vml_common