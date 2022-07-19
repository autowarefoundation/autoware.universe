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

#include "apparent_safe_velocity_limiter/types.hpp"

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{
multilinestring_t extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_auto_planning_msgs::msg::HADMapRoute & route,
  const std::vector<std::string> & tags);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__MAP_UTILS_HPP_
