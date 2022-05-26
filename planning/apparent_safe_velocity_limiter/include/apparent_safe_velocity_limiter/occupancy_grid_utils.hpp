// Copyright 2022 Tier IV, Inc.
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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_

#include "apparent_safe_velocity_limiter/collision_distance.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <grid_map_core/GridMap.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace apparent_safe_velocity_limiter
{

/// @brief mask gridmap cells that are inside the given polygons
/// @param[in, out] grid_map the grid map to modify
/// @param[in] polygons the polygons to mask from the grid map
void maskPolygons(grid_map::GridMap & grid_map, const multipolygon_t & polygons);

/// @brief apply a threshold to the grid map
/// @param[in, out] grid_map the grid map to modify
/// @param[in] threshold cells above this value are set to the max value, the other are set to 0
void threshold(grid_map::GridMap & grid_map, const float threshold);

/// @brief apply a threshold to the grid map
/// @param[in, out] cv_image opencv image to modify
/// @param[in] num_iter optional parameter for the number of iteration performed for noise removal
void denoise(cv::Mat & cv_image, const int num_iter = 2);

/// @brief extract from an occupancy grid the lines representing static obstacles
/// @param[in] occupancy_grid input occupancy grid
/// @param[in] polygon_in_masks occupancy grid cells inside the polygon are masked away
/// @param[in] polygon_out_masks occupancy grid cells outisde the polygon are masked away
/// @param[in] occupied_threshold threshold to use for identifying obstacles in the occupancy grid
/// @return multiple linestrings each representing an obstacle
multilinestring_t extractStaticObstaclePolygons(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, const multipolygon_t & polygon_in_masks,
  const polygon_t & polygon_out_masks, const int8_t occupied_threshold);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_
