// Copyright 2024 Autoware Foundation
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

#include "autoware/motion_velocity_planner_common/ttc_utils.hpp"

namespace autoware::motion_velocity_planner
{
/// @brief calculate the time distance of a predicted object at each ego trajectory point
/// @param ego_collision_checker
/// @param predicted_object
/// @return
CollisionTimeRanges calculate_object_to_trajectory_time_ranges(
  const CollisionChecker & ego_collision_checker,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  std::vector<std::optional<CollisionTimeRange>> time_ranges(
    ego_collision_checker.trajectory_size());
  for (const auto & object_path : predicted_object.kinematics.predicted_paths) {
    const auto time_step = rclcpp::Duration(object_path.time_step).seconds();
    auto t = 0.0;
    for (const auto & path_pose : object_path.path) {
      const auto object_footprint =
        autoware::universe_utils::toPolygon2d(path_pose, predicted_object.shape);
      const auto collisions = ego_collision_checker.get_collisions(object_footprint);
      for (const auto & collision : collisions) {
        if (!time_ranges[collision.trajectory_index].has_value()) {
          time_ranges[collision.trajectory_index].emplace(t);
        } else {
          time_ranges[collision.trajectory_index]->extend(t);
        }
      }
      t += time_step;
    }
  }
  return time_ranges;
}

/// @brief calculate time to collisions for each ego trajectory point
/// @param ego_collision_checker collision checker for the ego trajectory
/// @param ego_trajectory ego trajectory with accurate time_from_start values
/// @param predicted_objects predicted object to check for collisions
std::vector<std::optional<CollisionTimeRange>> calculate_collision_time_ranges_along_trajectory(
  const CollisionChecker & ego_collision_checker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  std::vector<std::optional<CollisionTimeRange>> time_range_along_trajectory(ego_trajectory.size());
  const auto object_to_trajectory_time_ranges =
    calculate_object_to_trajectory_time_ranges(ego_collision_checker, predicted_object);
  for (auto i = 0UL; i < object_to_trajectory_time_ranges.size(); ++i) {
    const auto & time_range = object_to_trajectory_time_ranges[i];
    if (time_range.has_value()) {
      const auto ego_t = rclcpp::Duration(ego_trajectory[i].time_from_start).seconds();
      time_range_along_trajectory[i].emplace(time_range->from_s - ego_t, time_range->to_s - ego_t);
    }
  }
  return time_range_along_trajectory;
}

std::vector<TimeCollisions> calculate_time_collisions_along_trajectory(
  const CollisionChecker & ego_collision_checker,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  std::vector<TimeCollisions> time_collisions_along_trajectory(
    ego_collision_checker.trajectory_size());
  for (const auto & object_path : predicted_object.kinematics.predicted_paths) {
    const auto time_step = rclcpp::Duration(object_path.time_step).seconds();
    auto t = 0.0;
    for (const auto & path_pose : object_path.path) {
      const auto object_footprint =
        autoware::universe_utils::toPolygon2d(path_pose, predicted_object.shape);
      const auto collisions = ego_collision_checker.get_collisions(object_footprint);
      for (const auto & collision : collisions) {
        auto & time_collision = time_collisions_along_trajectory[collision.trajectory_index];
        auto & points = time_collision[t];
        points.insert(
          points.end(), collision.collision_points.begin(), collision.collision_points.end());
      }
      t += time_step;
    }
  }
  return time_collisions_along_trajectory;
}

}  // namespace autoware::motion_velocity_planner
