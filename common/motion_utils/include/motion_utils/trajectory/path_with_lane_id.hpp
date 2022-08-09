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

#ifndef MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
#define MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_

#include "tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp"

#include <boost/optional.hpp>

#include <utility>

namespace motion_utils
{
inline boost::optional<std::pair<size_t, size_t>> getPathIndexRangeWithLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t lane_id)
{
  size_t start_idx = 0;  // NOTE: to prevent from maybe-uninitialized error
  size_t end_idx = 0;    // NOTE: to prevent from maybe-uninitialized error

  bool found_first_idx = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & p = path.points.at(i);
    for (const auto & id : p.lane_ids) {
      if (id == lane_id) {
        if (!found_first_idx) {
          start_idx = i;
          found_first_idx = true;
        }
        end_idx = i;
      }
    }
  }

  if (found_first_idx) {
    return std::make_pair(start_idx, end_idx);
  }

  return {};
}
}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
