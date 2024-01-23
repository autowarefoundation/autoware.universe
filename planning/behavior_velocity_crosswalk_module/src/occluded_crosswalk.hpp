// Copyright 2024 Tier IV, Inc.
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

#ifndef OCCLUDED_CROSSWALK_HPP_
#define OCCLUDED_CROSSWALK_HPP_

#include "scene_crosswalk.hpp"

#include <grid_map_core/GridMap.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <optional>
#include <vector>

namespace behavior_velocity_planner
{
/// @brief check if the gridmap is occluded at the given index
/// @param [in] grid_map input grid map
/// @param [in] min_nb_of_cells minimum number of occluded cells needed to detect an occlusion (as
/// side of a square centered at the index)
/// @param [in] idx target index in the grid map
/// @param [in] params parameters
/// @return true if the index is occluded
bool is_occluded(
  const grid_map::GridMap & grid_map, const int min_nb_of_cells, const grid_map::Index idx,
  const behavior_velocity_planner::CrosswalkModule::PlannerParam & params);

/// @brief interpolate a point beyond the end of the given segment
/// @param [in] segment input segment
/// @param [in] extra_distance desired distance beyond the end of the segment
/// @return interpolated point beyond the end of the segment
lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicSegment2d & segment, const double extra_distance);

/// @brief check if the crosswalk should be ignored
/// @param crosswalk_lanelet lanelet of the crosswalk
/// @param ignore_with_traffic_light if true, ignore the crosswalk if it has a traffic light
/// @return true if the crosswalk should be ignored
bool is_crosswalk_ignored(
  const lanelet::ConstLanelet & crosswalk_lanelet, const bool ignore_with_traffic_light);

/// @brief check if the crosswalk is occluded
/// @param crosswalk_lanelet lanelet of the crosswalk
/// @param occupancy_grid occupancy grid with the occlusion information
/// @param path_intersection intersection between the crosswalk and the ego path
/// @param detection_range range away from the crosswalk until occlusions are considered
/// @param dynamic_objects dynamic objects
/// @param params parameters
/// @return true if the crosswalk is occluded
bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Point & path_intersection, const double detection_range,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & dynamic_objects,
  const behavior_velocity_planner::CrosswalkModule::PlannerParam & params);

/// @brief calculate the distance away from the crosswalk that should be checked for occlusions
/// @param occluded_objects_velocity assumed velocity of the objects coming out of occlusions
/// @param dist_ego_to_crosswalk distance between ego and the crosswalk
/// @param ego_velocity current velocity of ego
double calculate_detection_range(
  const double occluded_object_velocity, const double dist_ego_to_crosswalk,
  const double ego_velocity);

/// @brief update timers so that the slowdown activates if the initial time is older than the buffer
/// @param initial_time initial time
/// @param most_recent_slowdown_time time to set only if initial_time is older than the buffer
/// @param buffer [s] time buffer
void update_occlusion_timers(
  std::optional<rclcpp::Time> & initial_time,
  std::optional<rclcpp::Time> & most_recent_slowdown_time, const rclcpp::Time & now,
  const double buffer);
}  // namespace behavior_velocity_planner

#endif  // OCCLUDED_CROSSWALK_HPP_
