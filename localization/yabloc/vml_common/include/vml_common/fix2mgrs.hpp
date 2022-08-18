#pragma once
#include <Eigen/StdVector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace vml_common
{
Eigen::Vector3d fix2Mgrs(const sensor_msgs::msg::NavSatFix & msg);
}