#pragma once
#include <Eigen/Core>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace pcdless::common
{
Eigen::Vector3d fix_to_mgrs(const sensor_msgs::msg::NavSatFix & msg);
}