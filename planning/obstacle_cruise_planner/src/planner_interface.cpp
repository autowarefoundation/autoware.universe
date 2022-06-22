// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_cruise_planner/planner_interface.hpp"

Trajectory PlannerInterface::insertStopPointToTrajectory(
  const ObstacleCruisePlannerData & planner_data)
{
  const auto nearest_segment_idx = tier4_autoware_utils::findNearestSegmentIndex(
    planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!nearest_segment_idx) {
    return planner_data.traj;
  }

  // Get Closest Behavior Stop Distance
  const double traj_length = tier4_autoware_utils::calcArcLength(planner_data.traj.points);
  const double dist_to_segment = tier4_autoware_utils::calcSignedArcLength(
    planner_data.traj.points, 0, *nearest_segment_idx + 1);
  const auto closest_forward_stop_dist = tier4_autoware_utils::calcDistanceToForwardStopPoint(
    planner_data.traj.points, *nearest_segment_idx + 1);
  const double closest_behavior_stop_dist =
    closest_forward_stop_dist
      ? std::min(dist_to_segment + closest_forward_stop_dist.get(), traj_length)
      : traj_length;

  // Get Closest Obstacle Stop Distance
  double closest_obstacle_stop_dist = closest_behavior_stop_dist;
  const double offset = vehicle_info_.max_longitudinal_offset_m + min_behavior_stop_margin_;
  for (const auto & obstacle : planner_data.target_obstacles) {
    // Ignore obstacle that is not required to stop
    if (!isStopRequired(obstacle)) {
      continue;
    }

    const double stop_dist = tier4_autoware_utils::calcSignedArcLength(
                               planner_data.traj.points, 0, obstacle.collision_point) -
                             offset;
    closest_obstacle_stop_dist = std::clamp(stop_dist, 0.0, closest_obstacle_stop_dist);
  }

  // If behavior stop point is ahead of the closest_obstacle_stop point within a certain margin
  // we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const double stop_dist_diff = closest_behavior_stop_dist - closest_obstacle_stop_dist;
  if (0 < stop_dist_diff && stop_dist_diff < longitudinal_info_.safe_distance_margin) {
    closest_obstacle_stop_dist = closest_behavior_stop_dist;
  }

  const double closest_stop_dist = std::min(closest_behavior_stop_dist, closest_obstacle_stop_dist);

  return obstacle_cruise_utils::insertStopPoint(planner_data.traj, closest_stop_dist);
}
