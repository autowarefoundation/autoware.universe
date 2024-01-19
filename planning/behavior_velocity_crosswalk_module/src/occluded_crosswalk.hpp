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

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

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
/// @param [in] p1 first point of the segment
/// @param [in] p2 second point of the segment
/// @param [in] extra_distance distance beyond p2 of the interpolated point
/// @return interpolated point beyond p2
lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2, const double extra_distance);

/// @brief check if the crosswalk is occluded
/// @param crosswalk_lanelet lanelet of the crosswalk
/// @param occupancy_grid occupancy grid
/// @param path_intersection intersection between the crosswalk and the ego path
/// @param params parameters
/// @return true if the crosswalk is occluded
bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Point & path_intersection,
  const behavior_velocity_planner::CrosswalkModule::PlannerParam & params);
}  // namespace behavior_velocity_planner

#endif  // OCCLUDED_CROSSWALK_HPP_
