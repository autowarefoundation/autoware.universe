
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

#include "apparent_safe_velocity_limiter/map_utils.hpp"
#include "lanelet2_core/primitives/LineString.h"

#include <lanelet2_core/Attribute.h>

#include <algorithm>

namespace apparent_safe_velocity_limiter
{
Obstacles extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_auto_planning_msgs::msg::HADMapRoute & route,
  const std::vector<std::string> & tags, const std::vector<int64_t> & obstacle_ids)
{
  Obstacles obstacles;
  lanelet::Ids ids;
  for (const auto & segment : route.segments) {
    ids.push_back(segment.preferred_primitive_id);
    for (const auto & primitive : segment.primitives) ids.push_back(primitive.id);
  }
  for (const auto & id : ids) {
    const auto lanelet = lanelet_map.laneletLayer.find(id);
    if (lanelet == lanelet_map.laneletLayer.end()) continue;
    for (const auto linestring : {lanelet->leftBound2d(), lanelet->rightBound2d()}) {
      if (isObstacle(linestring, tags, obstacle_ids)) {
        linestring_t ls;
        for (const auto & p : linestring) ls.push_back(point_t{p.x(), p.y()});
        obstacles.push_back(ls);
      }
    }
  }
  return obstacles;
}

bool isObstacle(
  const lanelet::ConstLineString2d & ls, const std::vector<std::string> & tags,
  const std::vector<int64_t> & ids)
{
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, "");
  return type != "" && (std::find(tags.begin(), tags.end(), type) != tags.end() ||
                        std::find(ids.begin(), ids.end(), ls.id()) != ids.end());
}
}  // namespace apparent_safe_velocity_limiter
