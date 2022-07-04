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

  const int DIGIT = 7;
  MGRS::Forward(zone, northp, x, y, msg.latitude, DIGIT, mgrs);

  double local_x = std::stoi(mgrs.substr(5, DIGIT)) * std::pow(10, 5 - DIGIT);
  double local_y = std::stoi(mgrs.substr(5 + DIGIT, DIGIT)) * std::pow(10, 5 - DIGIT);
  return {local_x, local_y, 0};
}