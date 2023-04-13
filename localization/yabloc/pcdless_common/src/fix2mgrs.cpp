// Copyright 2023 TIER IV, Inc.
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

#include "pcdless_common/fix2mgrs.hpp"

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace pcdless::common
{
Eigen::Vector3d ublox_to_mgrs(const ublox_msgs::msg::NavPVT & msg)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = msg.lat * 1e-7f;
  fix.longitude = msg.lon * 1e-7f;
  fix.altitude = msg.height * 1e-3f;
  return fix_to_mgrs(fix);
}

Eigen::Vector3d fix_to_mgrs(const sensor_msgs::msg::NavSatFix & msg)
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
}  // namespace pcdless::common