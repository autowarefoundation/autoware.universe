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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__REPLAN_CHECKER_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__REPLAN_CHECKER_HPP_

#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <boost/geometry.hpp>

#include <vector>

namespace drivable_area_expansion
{
class ReplanChecker
{
private:
  linestring_t prev_path_ls_{};
  polygon_t prev_expanded_drivable_area_{};

public:
  /// @brief set the previous path and its expanded drivable area
  /// @param path previous path
  void set_previous(const PathWithLaneId & path)
  {
    prev_path_ls_.clear();
    for (const auto & p : path.points)
      prev_path_ls_.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
    boost::geometry::clear(prev_expanded_drivable_area_);
    for (const auto & p : path.left_bound)
      prev_expanded_drivable_area_.outer().emplace_back(p.x, p.y);
    for (auto it = path.right_bound.rbegin(); it != path.right_bound.rend(); ++it)
      prev_expanded_drivable_area_.outer().emplace_back(it->x, it->y);
    if (!boost::geometry::is_empty(prev_expanded_drivable_area_))
      prev_expanded_drivable_area_.outer().push_back(prev_expanded_drivable_area_.outer().front());
  }

  /// @brief get the previous expanded drivable area
  /// @return the previous expanded drivable area
  polygon_t get_previous_expanded_drivable_area() { return prev_expanded_drivable_area_; }

  /// @brief reset the previous path and expanded drivable area
  void reset()
  {
    boost::geometry::clear(prev_path_ls_);
    boost::geometry::clear(prev_expanded_drivable_area_);
  }

  /// @brief calculate the index of the input path from which replanning is necessary (starting from
  /// ego pose)
  /// @param [in] path input path
  /// @param [in] ego_index path index before the current ego pose
  /// @param [in] max_deviation [m] replan index will be the first path point that deviates by more
  /// than this distance
  /// @return path index from which to replan
  size_t calculate_replan_index(
    const PathWithLaneId & path, const size_t ego_index, const double max_deviation) const
  {
    linestring_t path_ls;
    path_ls.reserve(path.points.size());
    for (const auto & p : path.points)
      path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
    // full replan if no prev path or if end of the previous path does not match with the new path
    if (
      prev_path_ls_.empty() ||
      boost::geometry::distance(prev_path_ls_.back(), path_ls) > max_deviation)
      return 0;

    for (size_t i = ego_index; i < path_ls.size(); ++i) {
      const auto & point = path_ls[i];
      const auto deviation_distance = boost::geometry::distance(point, prev_path_ls_);
      if (deviation_distance > max_deviation) {
        return i;
      }
    }
    return path_ls.size();
  }
};
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__REPLAN_CHECKER_HPP_
