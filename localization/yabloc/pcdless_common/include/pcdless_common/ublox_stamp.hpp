#pragma once
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::common
{
rclcpp::Time ublox_time_to_stamp(const ublox_msgs::msg::NavPVT & msg);
ublox_msgs::msg::NavPVT stamp_to_ublox_time(const builtin_interfaces::msg::Time & stamp);

}  // namespace pcdless::common