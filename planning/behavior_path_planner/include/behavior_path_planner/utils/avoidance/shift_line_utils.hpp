// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__SHIFT_LINE_UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__SHIFT_LINE_UTILS_HPP_

#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::shift_line_utils
{

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using ArcLength = std::vector<double>;

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2);

std::vector<size_t> calcParentIds(
  const AvoidLineArray & parent_candidates, const AvoidLine & child);

void alignShiftLinesOrder(
  AvoidLineArray & shift_lines, const double current_shift,
  const bool recalculate_start_length = true);

void fillAdditionalInfoFromPoint(
  const PathWithLaneId & path, const size_t ego_idx, AvoidLineArray & shift_lines);

void fillAdditionalInfoFromLongitudinal(
  const PathWithLaneId & path, const size_t ego_idx, AvoidLineArray & shift_lines);

void fillAdditionalInfo(const PathWithLaneId & path, AvoidLine & shift_line);

void fillAdditionalInfo(const PathWithLaneId & path, AvoidLineArray & shift_lines);

void quantizeShiftLine(AvoidLineArray & shift_lines, const double interval);

void trimSmallShiftLine(AvoidLineArray & shift_lines, const double shift_diff_thres);

void trimSimilarGradShiftLine(AvoidLineArray & shift_lines, const double threshold);

AvoidLine toAvoidLine(const ShiftLine & shift_line);

ShiftedPath toShiftedPath(const PathWithLaneId & path);

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points);

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines);
}  // namespace behavior_path_planner::shift_line_utils

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__SHIFT_LINE_UTILS_HPP_
