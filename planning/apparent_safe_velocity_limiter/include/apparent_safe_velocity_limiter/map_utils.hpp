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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_

#include "apparent_safe_velocity_limiter/obstacles.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{

/// @brief Extract from the lanelet map static obstacles that are along the given route
/// @details Relevent lanelets are selected as listed from the route message.
/// The left and right linestring of each relevent lanelet is determined to be an obstacle if
/// its tag matches one of the given tags, or its id matches one of the given ids
/// @param[in] lanelet_map lanelet map
/// @param[in] route route to select relevent lanelets
/// @param[in] tags tags to identify obstacle linestrings
/// @param[in] obstacle_ids ids to identify obstacle linestrings
/// @return the extracted obstacles
Obstacles extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_auto_planning_msgs::msg::HADMapRoute & route,
  const std::vector<std::string> & tags, const std::vector<int64_t> & obstacle_ids);

/// @brief Determine if the given linestring is an obstacle
/// @param[in] ls linestring to check
/// @param[in] tags obstacle tags
/// @param[in] ids obstacle ids
/// @return true if the linestring is an obstacle
bool isObstacle(
  const lanelet::ConstLineString2d & ls, const std::vector<std::string> & tags,
  const std::vector<int64_t> & ids);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_
