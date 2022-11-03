// Copyright 2019 Autoware Foundation
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

#include "utility_functions.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <unordered_set>
#include <utility>

bool exists(const std::unordered_set<lanelet::Id> & set, const lanelet::Id & id)
{
  return set.find(id) != set.end();
}

tier4_autoware_utils::LinearRing2d createVehicleFootprint(
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  using tier4_autoware_utils::LinearRing2d;
  using tier4_autoware_utils::Point2d;

  const auto & i = vehicle_info;

  LinearRing2d footprint;
  footprint.push_back(Point2d{i.max_longitudinal_offset_m, i.max_lateral_offset_m});
  footprint.push_back(Point2d{i.max_longitudinal_offset_m, i.min_lateral_offset_m});
  footprint.push_back(Point2d{i.min_longitudinal_offset_m, i.min_lateral_offset_m});
  footprint.push_back(Point2d{i.min_longitudinal_offset_m, i.max_lateral_offset_m});

  return footprint;
}
void set_color(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

lanelet::ConstLanelet combine_lanelets(const lanelet::ConstLanelets & lanelets)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  lanelet::Points3d centers;
  for (const auto & llt : lanelets) {
    for (const auto & pt : llt.leftBound()) {
      lefts.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.rightBound()) {
      rights.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.centerline()) {
      centers.push_back(lanelet::Point3d(pt));
    }
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  const auto center_line = lanelet::LineString3d(lanelet::InvalId, centers);
  auto combined_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  combined_lanelet.setCenterline(center_line);
  return std::move(combined_lanelet);
}
