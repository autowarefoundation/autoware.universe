#pragma once
#include <Eigen/StdVector>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

inline Eigen::Vector3d fix2Mgrs(const sensor_msgs::msg::NavSatFix& msg)
{
  using namespace GeographicLib;
  double x, y;
  int zone;
  bool northp;
  std::string mgrs;
  UTMUPS::Forward(msg.latitude, msg.longitude, zone, northp, x, y);
  MGRS::Forward(zone, northp, x, y, msg.latitude, 6, mgrs);

  double local_x = std::stoi(mgrs.substr(5, 6)) * 0.1;
  double local_y = std::stoi(mgrs.substr(11, 6)) * 0.1;
  return {local_x, local_y, 0};
}