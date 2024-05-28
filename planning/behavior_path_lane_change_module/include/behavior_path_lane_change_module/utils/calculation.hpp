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

#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_

#include "behavior_path_lane_change_module/utils/data_structs.hpp"

namespace behavior_path_planner::utils::lane_change::calculation
{
using data::lane_change::CommonDataPtr;
using data::lane_change::LCParamPtr;
using data::lane_change::RouteHandlerPtr;
using route_handler::Direction;

double calc_min_lane_change_length(
  const LCParamPtr & lc_params, const std::vector<double> & shift_intervals);

double calc_min_lane_change_length(
  const CommonDataPtr & common_data, const lanelet::ConstLanelets & lanes,
  Direction direction = Direction::NONE);

double calc_ego_remaining_distance_in_current_lanes(const CommonDataPtr & common_data);
}  // namespace behavior_path_planner::utils::lane_change::calculation

#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_
