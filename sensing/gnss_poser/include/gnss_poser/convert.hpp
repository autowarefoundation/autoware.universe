// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef GNSS_POSER__CONVERT_HPP_
#define GNSS_POSER__CONVERT_HPP_

#include "gnss_poser/gnss_stat.hpp"

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <geo_pos_conv/geo_pos_conv.hpp>

// #include <rclcpp/logging.hpp>

// #include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "gnss_poser/msg.hpp"
#include <string>

namespace gnss_poser
{
enum class MGRSPrecision {
  _10_KIRO_METER = 1,
  _1_KIRO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};
// EllipsoidHeight:height above ellipsoid
// OrthometricHeight:height above geoid
double EllipsoidHeight2OrthometricHeight(const DoraNavSatFix & nav_sat_fix_msg)
{
  double OrthometricHeight{0.0};
  try 
  {
    GeographicLib::Geoid egm2008("egm2008-1");
    OrthometricHeight = egm2008.ConvertHeight(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
      GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  } 
  catch (const GeographicLib::GeographicErr & err) 
  {
    std::cout << "Failed to convert Height from Ellipsoid to Orthometric" << err.what()<< std::endl;
  }
  return OrthometricHeight;
}
GNSSStat NavSatFix2LocalCartesianWGS84(
  const DoraNavSatFix & nav_sat_fix_msg,
  DoraNavSatFix nav_sat_fix_origin_)
{
  GNSSStat local_cartesian;
  local_cartesian.coordinate_system = CoordinateSystem::LOCAL_CARTESIAN_WGS84;
  
      std::cout << "NavSatFix2LocalCartesianWGS84 System" << std::endl;
  try {
    GeographicLib::LocalCartesian localCartesian_origin(
      nav_sat_fix_origin_.latitude, nav_sat_fix_origin_.longitude, nav_sat_fix_origin_.altitude);
    localCartesian_origin.Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
      local_cartesian.x, local_cartesian.y, local_cartesian.z);

    local_cartesian.latitude = nav_sat_fix_msg.latitude;
    local_cartesian.longitude = nav_sat_fix_msg.longitude;
    local_cartesian.altitude = nav_sat_fix_msg.altitude;
  } catch (const GeographicLib::GeographicErr & err) {
    std::cout << "Failed to convert NavSatFix to LocalCartesian" << err.what()<< std::endl;
 
    // RCLCPP_ERROR_STREAM(logger, "Failed to convert NavSatFix to LocalCartesian" << err.what());
  }
  return local_cartesian;
}
GNSSStat NavSatFix2UTM(const DoraNavSatFix & nav_sat_fix_msg)
{
  GNSSStat utm;
  utm.coordinate_system = CoordinateSystem::UTM;
  try {
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm.zone, utm.northup, utm.x, utm.y);

    utm.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg);

    utm.latitude = nav_sat_fix_msg.latitude;
    utm.longitude = nav_sat_fix_msg.longitude;
    utm.altitude = nav_sat_fix_msg.altitude;
  } catch (const GeographicLib::GeographicErr & err) {
    std::cout << "Failed to convert from LLH to UTM: " << err.what()<< std::endl;
  }
  return utm;
}
GNSSStat NavSatFix2LocalCartesianUTM(
  const DoraNavSatFix & nav_sat_fix_msg,
  DoraNavSatFix nav_sat_fix_origin)
{
  GNSSStat utm_local;
  utm_local.coordinate_system = CoordinateSystem::UTM;
  try {
    // origin of the local coordinate system in global frame
    GNSSStat utm_origin;
    utm_origin.coordinate_system = CoordinateSystem::UTM;
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_origin.latitude, nav_sat_fix_origin.longitude, utm_origin.zone,
      utm_origin.northup, utm_origin.x, utm_origin.y);
    utm_origin.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_origin);
    // individual coordinates of global coordinate system
    double global_x = 0.0;
    double global_y = 0.0;
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm_origin.zone, utm_origin.northup,
      global_x, global_y);
    utm_local.latitude = nav_sat_fix_msg.latitude;
    utm_local.longitude = nav_sat_fix_msg.longitude;
    utm_local.altitude = nav_sat_fix_msg.altitude;
    // individual coordinates of local coordinate system
    utm_local.x = global_x - utm_origin.x;
    utm_local.y = global_y - utm_origin.y;
    utm_local.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg) - utm_origin.z;
  } catch (const GeographicLib::GeographicErr & err) {
    std::cout << "Failed to convert from LLH to UTM in local coordinates " << err.what()<< std::endl;
  }
  return utm_local;
}
GNSSStat UTM2MGRS(
  const GNSSStat & utm, const MGRSPrecision & precision)
{
  constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

  GNSSStat mgrs = utm;
  mgrs.coordinate_system = CoordinateSystem::MGRS;
  try {
    std::string mgrs_code;
    GeographicLib::MGRS::Forward(
      utm.zone, utm.northup, utm.x, utm.y, utm.latitude, static_cast<int>(precision), mgrs_code);
    mgrs.zone = std::stod(mgrs_code.substr(0, GZD_ID_size));
    mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
             std::pow(
               10, static_cast<int>(MGRSPrecision::_1_METER) -
                     static_cast<int>(precision));  // set unit as [m]
    mgrs.y = std::stod(mgrs_code.substr(
               GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
             std::pow(
               10, static_cast<int>(MGRSPrecision::_1_METER) -
                     static_cast<int>(precision));  // set unit as [m]
    mgrs.z = utm.z;                                 // TODO(ryo.watanabe)
  } catch (const GeographicLib::GeographicErr & err) {
    std::cout << "Failed to convert from UTM to MGRS" << err.what()<< std::endl;
  }
  return mgrs;
}

GNSSStat NavSatFix2MGRS(
  const DoraNavSatFix & nav_sat_fix_msg, const MGRSPrecision & precision)
{
  const auto utm = NavSatFix2UTM(nav_sat_fix_msg);
  const auto mgrs = UTM2MGRS(utm, precision);
  return mgrs;
}

GNSSStat NavSatFix2PLANE(
  const DoraNavSatFix & nav_sat_fix_msg, const int & plane_zone)
{
  GNSSStat plane;
  plane.coordinate_system = CoordinateSystem::PLANE;
  geo_pos_conv geo;
  geo.set_plane(plane_zone);
  geo.llh_to_xyz(nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude);
  plane.x = geo.y();
  plane.y = geo.x();
  plane.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg);
  return plane;
}
}  // namespace gnss_poser

#endif  // GNSS_POSER__CONVERT_HPP_
