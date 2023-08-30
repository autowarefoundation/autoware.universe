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

#include <rclcpp/logging.hpp>
#include "tier4_geography_utils/height.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>

namespace gnss_poser
{
enum class MGRSPrecision {
  _10_KILO_METER = 1,
  _1_KILO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};

GNSSStat NavSatFix2UTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const rclcpp::Logger & logger,
  int height_system);

GNSSStat NavSatFix2LocalCartesianUTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg,
  sensor_msgs::msg::NavSatFix nav_sat_fix_origin, const rclcpp::Logger & logger, int height_system);

GNSSStat UTM2MGRS(
  const GNSSStat & utm, const MGRSPrecision & precision, const rclcpp::Logger & logger);

GNSSStat NavSatFix2MGRS(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const MGRSPrecision & precision,
  const rclcpp::Logger & logger, int height_system);
}  // namespace gnss_poser

#endif  // GNSS_POSER__CONVERT_HPP_
