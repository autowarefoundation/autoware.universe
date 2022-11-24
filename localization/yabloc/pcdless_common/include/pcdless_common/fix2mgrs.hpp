#pragma once
#include <Eigen/Core>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::common
{
Eigen::Vector3d ublox_to_mgrs(const ublox_msgs::msg::NavPVT & msg);

Eigen::Vector3d fix_to_mgrs(const sensor_msgs::msg::NavSatFix & msg);
}  // namespace pcdless::common