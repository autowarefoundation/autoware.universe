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

#include "calculate_slowdown_points.hpp"

#include "footprint.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::motion_velocity_planner::out_of_lane
{

std::optional<geometry_msgs::msg::Pose> calculate_last_in_lane_pose(
  const EgoData & ego_data, const OutOfLanePoint & out_of_lane_point,
  const autoware::universe_utils::Polygon2d & footprint, const PlannerParam & params)
{
  const auto max_arc_length = motion_utils::calcSignedArcLength(
    ego_data.trajectory_points, 0UL, out_of_lane_point.trajectory_index);
  const auto min_arc_length = motion_utils::calcSignedArcLength(
                                ego_data.trajectory_points, 0UL, ego_data.first_trajectory_idx) +
                              ego_data.longitudinal_offset_to_first_trajectory_index +
                              ego_data.min_stop_distance;
  for (auto l = max_arc_length - params.precision; l >= min_arc_length; l -= params.precision) {
    const auto interpolated_pose =
      autoware::motion_utils::calcInterpolatedPose(ego_data.trajectory_points, l);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_pose);
    const auto is_inside_ego_lane =
      boost::geometry::disjoint(interpolated_footprint, ego_data.drivable_lane_polygons);
    if (is_inside_ego_lane) return interpolated_pose;
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose> calculate_slowdown_point(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data, PlannerParam params)
{
  params.extra_front_offset += params.lon_dist_buffer;
  params.extra_right_offset += params.lat_dist_buffer;
  params.extra_left_offset += params.lat_dist_buffer;
  const auto base_footprint = make_base_footprint(params);

  // search for the first slowdown decision for which a stop point can be inserted
  for (const auto & out_of_lane_point : out_of_lane_data.outside_points) {
    const auto last_in_lane_pose =
      calculate_last_in_lane_pose(ego_data, out_of_lane_point, base_footprint, params);
    if (last_in_lane_pose) return last_in_lane_pose;
  }
  return std::nullopt;
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
