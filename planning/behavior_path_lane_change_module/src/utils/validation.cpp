// Copyright 2024 TIER IV, Inc.
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

#include "behavior_path_lane_change_module/utils/validation.hpp"

#include "behavior_path_lane_change_module/utils/calculation.hpp"

namespace behavior_path_planner::utils::lane_change::validation
{
using calculation::calc_min_lane_change_length;
bool is_near_end_of_current_lanes(const CommonDataPtr & common_data, const double threshold)
{
  const auto & current_lanes = common_data->lanes.current;
  const auto min_lc_length =
    calc_min_lane_change_length(common_data, current_lanes, Direction::NONE);

  const auto dist_to_end =
    std::max(0.0, calculation::calc_ego_remaining_distance_in_current_lanes(common_data)) -
    min_lc_length;

  return dist_to_end < threshold;
}
}  // namespace behavior_path_planner::utils::lane_change::validation
