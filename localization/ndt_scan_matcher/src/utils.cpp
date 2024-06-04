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

#include <ndt_scan_matcher/metadata.hpp>
#include <ndt_scan_matcher/utils.hpp>

#include <fmt/format.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace loc
{

bool cylinderAndBoxOverlapExists(
  const double center_x, const double center_y, const double radius,
  const pcl::PointXYZ box_min_point, const pcl::PointXYZ box_max_point)
{
  // Collision detection with x-y plane (circular base of the cylinder)
  if (
    box_min_point.x - radius <= center_x && center_x <= box_max_point.x + radius &&
    box_min_point.y - radius <= center_y && center_y <= box_max_point.y + radius) {
    return true;
  }

  // Collision detection with box edges
  const double dx0 = center_x - box_min_point.x;
  const double dx1 = center_x - box_max_point.x;
  const double dy0 = center_y - box_min_point.y;
  const double dy1 = center_y - box_max_point.y;

  if (
    std::hypot(dx0, dy0) <= radius || std::hypot(dx1, dy0) <= radius ||
    std::hypot(dx0, dy1) <= radius || std::hypot(dx1, dy1) <= radius) {
    return true;
  }

  return false;
}

// Return the indices of the segments contained in a specified area
void queryContainedSegmentIdx(
  float center_x, float center_y, float radius, const PCDMetadata & m,
  std::list<SegmentIndex> & map_id)
{
  double lower_x = center_x - radius, lower_y = center_y - radius;
  double upper_x = center_x + radius, upper_y = center_y + radius;

  // Compute the boundaries of the segment indices
  auto lb = m.coorToSegmentIndex(lower_x, lower_y);
  auto ub = m.coorToSegmentIndex(upper_x, upper_y);

  // Loop on the candidate boundaries
  for (float min_x = lb.x; min_x <= ub.x; min_x += m.res_x_) {
    for (float min_y = lb.y; min_y <= ub.y; min_y += m.res_y_) {
      pcl::PointXYZ lbox, ubox;

      lbox.x = min_x;
      lbox.y = min_y;
      ubox.x = min_x + m.res_x_;
      ubox.y = min_y + m.res_y_;

      if (cylinderAndBoxOverlapExists(center_x, center_y, radius, lbox, ubox)) {
        map_id.push_back(m.coorToSegmentIndex(lbox.x, lbox.y));
      }
    }
  }
}

}  // namespace loc
