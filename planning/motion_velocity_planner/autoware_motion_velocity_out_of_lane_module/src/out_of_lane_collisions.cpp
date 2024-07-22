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

#include "out_of_lane_collisions.hpp"

#include <algorithm>
#include <limits>

namespace autoware::motion_velocity_planner::out_of_lane
{
void calculate_object_time_collisions(
  OutOfLaneData & out_of_lane_data, const CollisionChecker & ego_trajectory_collision_checker,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects)
{
  for (const auto & object : objects) {
    // TODO(Maxime): do not calculate intersections with all the trajectory, only check the out of
    // lane areas
    const auto time_collisions =
      calculate_time_collisions_along_trajectory(ego_trajectory_collision_checker, object);
    for (auto & out_of_lane_point : out_of_lane_data.outside_points) {
      for (const auto & [t, points] : time_collisions[out_of_lane_point.trajectory_index]) {
        auto & collision_points = out_of_lane_point.time_collisions[t];
        collision_points.insert(collision_points.end(), points.begin(), points.end());
      }
    }
  }
}

void calculate_collisions_to_avoid(
  OutOfLaneData & out_of_lane_data, const EgoData & ego_data, const PlannerParam & params)
{
  for (auto & out_of_lane_point : out_of_lane_data.outside_points) {
    auto min_time = std::numeric_limits<double>::infinity();
    auto max_time = -std::numeric_limits<double>::infinity();
    for (const auto & [t, points] : out_of_lane_point.time_collisions) {
      for (const auto & out_of_lane_area : out_of_lane_point.outside_rings) {
        if (!boost::geometry::disjoint(out_of_lane_area, points)) {
          min_time = std::min(t, min_time);
          max_time = std::max(t, max_time);
          break;
        }
      }
    }
    if (min_time <= max_time) {
      out_of_lane_point.min_object_arrival_time = min_time;
      out_of_lane_point.max_object_arrival_time = max_time;
      const auto & ego_time =
        rclcpp::Duration(
          ego_data.trajectory_points[out_of_lane_point.trajectory_index].time_from_start)
          .seconds();
      if (ego_time >= min_time && ego_time <= max_time) {
        out_of_lane_point.ttc = 0.0;
      } else {
        out_of_lane_point.ttc =
          std::min(std::abs(ego_time - min_time), std::abs(ego_time - max_time));
      }
    }
  }
  for (auto & p : out_of_lane_data.outside_points) {
    p.to_avoid = params.mode == "ttc" ? (p.ttc && p.ttc <= params.ttc_threshold)
                                      : (p.min_object_arrival_time &&
                                         p.min_object_arrival_time <= params.time_threshold);
  }
}

void calculate_out_of_lane_areas(OutOfLaneData & out_of_lane_data, const EgoData & ego_data)
{
  for (auto i = 0UL; i < ego_data.trajectory_footprints.size(); ++i) {
    const auto & footprint = ego_data.trajectory_footprints[i];
    if (!boost::geometry::within(footprint, ego_data.drivable_lane_polygons)) {
      out_of_lane::OutOfLanePoint p;
      p.trajectory_index = i + ego_data.first_trajectory_idx;
      out_of_lane::Polygons out_of_lane_polygons;
      boost::geometry::difference(footprint, ego_data.drivable_lane_polygons, out_of_lane_polygons);
      for (const auto & area : out_of_lane_polygons) {
        p.outside_rings.push_back(area.outer);
      }
      out_of_lane_data.outside_points.push_back(p);
    }
  }
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
