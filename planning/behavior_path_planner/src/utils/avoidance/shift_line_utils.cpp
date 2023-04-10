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

#include "behavior_path_planner/utils/avoidance/shift_line_utils.hpp"

#include "behavior_path_planner/utils/avoidance/util.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::shift_line_utils
{

using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2)
{
  std::set<size_t> id_set{ids1.begin(), ids1.end()};
  for (const auto id : ids2) {
    id_set.insert(id);
  }
  const auto v = std::vector<size_t>{id_set.begin(), id_set.end()};
  return v;
}

std::vector<size_t> calcParentIds(const AvoidLineArray & parent_candidates, const AvoidLine & child)
{
  // Get the ID of the original AP whose transition area overlaps with the given AP,
  // and set it to the parent id.
  std::set<uint64_t> ids;
  for (const auto & al : parent_candidates) {
    const auto p_s = al.start_longitudinal;
    const auto p_e = al.end_longitudinal;
    const auto has_overlap = !(p_e < child.start_longitudinal || child.end_longitudinal < p_s);

    if (!has_overlap) {
      continue;
    }

    // Id the shift is overlapped, insert the shift point. Additionally, the shift which refers
    // to the same object id (created by the same object) will be set.
    //
    // Why? : think that there are two shifts, avoiding and .
    // If you register only the avoiding shift, the return-to-center shift will not be generated
    // when you get too close to or over the obstacle. The return-shift can be handled with
    // addReturnShift(), but it maybe reasonable to register the return-to-center shift for the
    // object at the same time as registering the avoidance shift to remove the complexity of the
    // addReturnShift().
    for (const auto & al_local : parent_candidates) {
      if (al_local.object.object.object_id == al.object.object.object_id) {
        ids.insert(al_local.id);
      }
    }
  }
  return std::vector<size_t>(ids.begin(), ids.end());
}

void alignShiftLinesOrder(
  AvoidLineArray & shift_lines, const double current_shift, const bool recalculate_start_length)
{
  if (shift_lines.empty()) {
    return;
  }

  // sort shift points from front to back.
  std::sort(shift_lines.begin(), shift_lines.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative length
  // NOTE: the input shift point must not have conflict range. Otherwise relative
  // length value will be broken.
  if (recalculate_start_length) {
    shift_lines.front().start_shift_length = current_shift;
    for (size_t i = 1; i < shift_lines.size(); ++i) {
      shift_lines.at(i).start_shift_length = shift_lines.at(i - 1).end_shift_length;
    }
  }
}

void fillAdditionalInfoFromPoint(
  const PathWithLaneId & path, const size_t ego_idx, AvoidLineArray & shift_lines)
{
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto dist_path_front_to_ego = calcSignedArcLength(path.points, 0, ego_idx);

  // calc longitudinal
  for (auto & sl : shift_lines) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arclength.at(sl.start_idx) - dist_path_front_to_ego;
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arclength.at(sl.end_idx) - dist_path_front_to_ego;
  }
}

void fillAdditionalInfoFromLongitudinal(
  const PathWithLaneId & path, const size_t ego_idx, AvoidLineArray & shift_lines)
{
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto dist_path_front_to_ego = calcSignedArcLength(path.points, 0, ego_idx);

  for (auto & sl : shift_lines) {
    sl.start_idx =
      findPathIndexFromArclength(arclength, sl.start_longitudinal + dist_path_front_to_ego);
    sl.start = path.points.at(sl.start_idx).point.pose;
    sl.end_idx =
      findPathIndexFromArclength(arclength, sl.end_longitudinal + dist_path_front_to_ego);
    sl.end = path.points.at(sl.end_idx).point.pose;
  }
}

void fillAdditionalInfo(const PathWithLaneId & path, AvoidLine & shift_line)
{
  const auto arclength = util::calcPathArcLengthArray(path);

  shift_line.start_idx = findNearestIndex(path.points, shift_line.start.position);
  shift_line.start_longitudinal = arclength.at(shift_line.start_idx);
  shift_line.end_idx = findNearestIndex(path.points, shift_line.end.position);
  shift_line.end_longitudinal = arclength.at(shift_line.end_idx);
}

void fillAdditionalInfo(const PathWithLaneId & path, AvoidLineArray & shift_lines)
{
  const auto arclength = util::calcPathArcLengthArray(path);

  // calc longitudinal
  for (auto & sl : shift_lines) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arclength.at(sl.start_idx);
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arclength.at(sl.end_idx);
  }
}

void quantizeShiftLine(AvoidLineArray & shift_lines, const double interval)
{
  if (interval < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sl : shift_lines) {
    sl.end_shift_length = std::round(sl.end_shift_length / interval) * interval;
  }
}

void trimSmallShiftLine(AvoidLineArray & shift_lines, const double shift_diff_thres)
{
  AvoidLineArray shift_lines_orig = shift_lines;
  shift_lines.clear();

  shift_lines.push_back(shift_lines_orig.front());  // Take the first one anyway (think later)

  for (size_t i = 1; i < shift_lines_orig.size(); ++i) {
    auto sl_now = shift_lines_orig.at(i);
    const auto sl_prev = shift_lines.back();
    const auto shift_diff = sl_now.end_shift_length - sl_prev.end_shift_length;

    auto sl_modified = sl_now;

    // remove the shift point if the length is almost same as the previous one.
    if (std::abs(shift_diff) < shift_diff_thres) {
      sl_modified.end_shift_length = sl_prev.end_shift_length;
      sl_modified.start_shift_length = sl_prev.end_shift_length;
    }

    shift_lines.push_back(sl_modified);
  }
}

void trimSimilarGradShiftLine(
  AvoidLineArray & avoid_lines, const double change_shift_dist_threshold)
{
  AvoidLineArray avoid_lines_orig = avoid_lines;
  avoid_lines.clear();

  avoid_lines.push_back(avoid_lines_orig.front());  // Take the first one anyway (think later)

  // Save the points being merged. When merging consecutively, also check previously merged points.
  AvoidLineArray being_merged_points;

  for (size_t i = 1; i < avoid_lines_orig.size(); ++i) {
    const auto al_now = avoid_lines_orig.at(i);
    const auto al_prev = avoid_lines.back();

    being_merged_points.push_back(al_prev);  // This point is about to be merged.

    auto combined_al = al_prev;
    setEndData(
      combined_al, al_now.end_shift_length, al_now.end, al_now.end_idx, al_now.end_longitudinal);
    combined_al.parent_ids = concatParentIds(combined_al.parent_ids, al_prev.parent_ids);

    const auto has_large_length_change = [&]() {
      for (const auto & original : being_merged_points) {
        const auto longitudinal = original.end_longitudinal - combined_al.start_longitudinal;
        const auto new_length =
          combined_al.getGradient() * longitudinal + combined_al.start_shift_length;

        if (std::abs(new_length - original.end_shift_length) > change_shift_dist_threshold) {
          return true;
        }
      }
      return false;
    }();

    if (has_large_length_change) {
      // If this point is merged with the previous points, it makes a large changes.
      // Do not merge this.
      avoid_lines.push_back(al_now);
      being_merged_points.clear();
    } else {
      avoid_lines.back() = combined_al;  // Update the last points by merging the current point
      being_merged_points.push_back(al_prev);
    }
  }
}

AvoidLine toAvoidLine(const ShiftLine & shift_line)
{
  AvoidLine ret{};
  ret.start = shift_line.start;
  ret.start_idx = shift_line.start_idx;
  ret.start_shift_length = shift_line.start_shift_length;
  ret.end = shift_line.end;
  ret.end_idx = shift_line.end_idx;
  ret.end_shift_length = shift_line.end_shift_length;

  return ret;
}

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points)
{
  ShiftLineArray shift_lines;
  for (const auto & ap : avoid_points) {
    shift_lines.push_back(ap);
  }
  return shift_lines;
}

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines)
{
  // TODO(Horibe) parametrize
  const auto isSimilar = [](const AvoidLine & a, const AvoidLine & b) {
    using tier4_autoware_utils::calcDistance2d;
    if (calcDistance2d(a.start, b.start) > 1.0) {
      return false;
    }
    if (calcDistance2d(a.end, b.end) > 1.0) {
      return false;
    }
    if (std::abs(a.end_shift_length - b.end_shift_length) > 0.5) {
      return false;
    }
    return true;
  };
  const auto hasSameObjectId = [](const auto & a, const auto & b) {
    return a.object.object.object_id == b.object.object.object_id;
  };

  auto combined = base_lines;  // initialized
  for (const auto & added_line : added_lines) {
    bool skip = false;

    for (const auto & base_line : base_lines) {
      if (hasSameObjectId(added_line, base_line) && isSimilar(added_line, base_line)) {
        skip = true;
        break;
      }
    }
    if (!skip) {
      combined.push_back(added_line);
    }
  }

  return combined;
}
}  // namespace behavior_path_planner::shift_line_utils
