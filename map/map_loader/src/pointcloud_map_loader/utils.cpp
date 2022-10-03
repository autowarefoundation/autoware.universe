// Copyright 2022 The Autoware Contributors
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

#include "utils.hpp"

#include <fmt/format.h>

bool sphereAndBoxOverlapExists(
  const geometry_msgs::msg::Point position, const double radius, const pcl::PointXYZ position_min,
  const pcl::PointXYZ position_max)
{
  if (
    (position_min.x - radius <= position.x && position.x <= position_max.x + radius &&
     position_min.y <= position.y && position.y <= position_max.y && position_min.z <= position.z &&
     position.z <= position_max.z) ||
    (position_min.x <= position.x && position.x <= position_max.x &&
     position_min.y - radius <= position.y && position.y <= position_max.y + radius &&
     position_min.z <= position.z && position.z <= position_max.z) ||
    (position_min.x <= position.x && position.x <= position_max.x && position_min.y <= position.y &&
     position.y <= position_max.y && position_min.z - radius <= position.z &&
     position.z <= position_max.z + radius)) {
    return true;
  }
  double r2 = std::pow(radius, 2.0);
  double minx2 = std::pow(position.x - position_min.x, 2.0);
  double maxx2 = std::pow(position.x - position_max.x, 2.0);
  double miny2 = std::pow(position.y - position_min.y, 2.0);
  double maxy2 = std::pow(position.y - position_max.y, 2.0);
  double minz2 = std::pow(position.z - position_min.z, 2.0);
  double maxz2 = std::pow(position.z - position_max.z, 2.0);
  if (
    minx2 + miny2 + minz2 <= r2 || minx2 + miny2 + maxz2 <= r2 || minx2 + maxy2 + minz2 <= r2 ||
    minx2 + maxy2 + maxz2 <= r2 || maxx2 + miny2 + minz2 <= r2 || maxx2 + miny2 + maxz2 <= r2 ||
    maxx2 + maxy2 + minz2 <= r2 || maxx2 + maxy2 + maxz2 <= r2) {
    return true;
  }
  return false;
}

bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area, const PCDFileMetadata metadata)
{
  // Currently, the area load only supports spherical area
  geometry_msgs::msg::Point position = area.center;
  double radius = area.radius;
  bool res = sphereAndBoxOverlapExists(position, radius, metadata.min, metadata.max);
  return res;
}
