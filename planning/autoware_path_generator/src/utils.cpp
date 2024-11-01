// Copyright 2024 TIER IV, Inc.
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

#include "autoware/path_generator/utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
namespace utils
{
std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> getWaypointGroups(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::LaneletMap & lanelet_map)
{
  std::vector<std::pair<lanelet::ConstPoints3d, std::pair<double, double>>> waypoint_groups;

  double arc_length_offset = 0.;
  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      arc_length_offset += lanelet::utils::getLaneletLength2d(lanelet);
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);
    const auto get_interval_bound =
      [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
        const auto arc_coordinates =
          lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point));
        return arc_length_offset + arc_coordinates.length +
               lateral_distance_factor * std::abs(arc_coordinates.distance);
      };

    constexpr auto new_group_threshold = 1.0;
    constexpr auto lateral_distance_factor = 10.0;
    if (
      waypoint_groups.empty() ||
      lanelet::geometry::distance2d(waypoint_groups.back().first.back(), waypoints.front()) >
        new_group_threshold) {
      waypoint_groups.emplace_back().second.first =
        get_interval_bound(waypoints.front(), -lateral_distance_factor);
    }
    waypoint_groups.back().second.second =
      get_interval_bound(waypoints.back(), lateral_distance_factor);

    for (const auto & waypoint : waypoints) {
      waypoint_groups.back().first.push_back(waypoint);
    }

    arc_length_offset += lanelet::utils::getLaneletLength2d(lanelet);
  }

  return waypoint_groups;
}

void removeOverlappingPoints(PathWithLaneId & path)
{
  auto & filtered_path_end = path.points.front();
  for (auto it = std::next(path.points.begin()); it != path.points.end();) {
    constexpr auto min_interval = 0.001;
    if (
      autoware::universe_utils::calcDistance3d(filtered_path_end.point, it->point) < min_interval) {
      filtered_path_end.lane_ids.push_back(it->lane_ids.front());
      filtered_path_end.point.longitudinal_velocity_mps = std::min(
        it->point.longitudinal_velocity_mps, filtered_path_end.point.longitudinal_velocity_mps);
      it = path.points.erase(it);
    } else {
      filtered_path_end = *it;
      ++it;
    }
  }
}
}  // namespace utils
}  // namespace autoware::path_generator
