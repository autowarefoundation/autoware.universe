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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/structs/path.hpp"

#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
using behavior_path_planner::LaneChangePath;
using behavior_path_planner::lane_change::CommonDataPtr;

bool get_prepare_segment(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path,
  const LaneChangePhaseMetrics prep_metric, PathWithLaneId & prepare_segment);

LaneChangePath get_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangePhaseMetrics & prep_metric,
  const LaneChangePhaseMetrics & lc_metric, const PathWithLaneId & prep_segment,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, const Pose & lc_start_pose,
  const double shift_length);

std::optional<LaneChangePath> construct_candidate_path(
  const LaneChangeInfo & lane_change_info, const PathWithLaneId & prepare_segment,
  const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

std::optional<LaneChangePath> calcTerminalLaneChangePath(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & prev_module_path);
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
