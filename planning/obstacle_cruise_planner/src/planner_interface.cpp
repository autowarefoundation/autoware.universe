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
  const double ego_offset = std::max(
    0.0, tier4_autoware_utils::calcSignedArcLength(
           planner_data.traj.points, 0, planner_data.current_pose.position));
  double closest_stop_dist = obstacle_cruise_utils::calcDistanceFromEgoPoseToStopPoint(
                               planner_data.traj, planner_data.current_pose) +
                             ego_offset;

  const double offset = vehicle_info_.max_longitudinal_offset_m + min_behavior_stop_margin_;

  for (const auto & obstacle : planner_data.target_obstacles) {
    // Ignore obstacle that is not required to stop
    if (!isStopRequired(obstacle)) {
      continue;
    }

    const double stop_dist = tier4_autoware_utils::calcSignedArcLength(
                               planner_data.traj.points, 0, obstacle.collision_point) -
                             offset;
    closest_stop_dist = std::clamp(stop_dist, 0.0, closest_stop_dist);
  }

  return obstacle_cruise_utils::insertStopPoint(planner_data.traj, closest_stop_dist);
}
