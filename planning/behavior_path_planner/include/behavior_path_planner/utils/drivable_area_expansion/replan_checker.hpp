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
namespace
{
linestring_t to_linestring(const PathWithLaneId & path)
{
  linestring_t ls;
  ls.reserve(path.points.size());
  for (const auto & p : path.points)
    ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
  return ls;
}
linestring_t to_linestring(const std::vector<Point> & points)
{
  linestring_t ls;
  ls.reserve(points.size());
  for (const auto & p : points) ls.emplace_back(p.x, p.y);
  return ls;
}
size_t first_deviating_index(
  const linestring_t & ls, const linestring_t & prev_ls, const double max_deviation)
{
  if (prev_ls.empty()) return 0;
  for (size_t i = 0; i < ls.size(); ++i) {
    const auto & point = ls[i];
    const auto deviation_distance = boost::geometry::distance(point, prev_ls);
    if (deviation_distance > max_deviation) return i;
  }
  return ls.size();
};
};  // namespace

class ReplanChecker
{
private:
  linestring_t prev_path_ls_{};
  linestring_t prev_left_bound_ls_{};
  linestring_t prev_right_bound_ls_{};
  polygon_t prev_expanded_drivable_area_{};

public:
  void set_previous(const PathWithLaneId & path, const polygon_t & prev_expanded_drivable_area)
  {
    prev_path_ls_.clear();
    prev_left_bound_ls_.clear();
    prev_right_bound_ls_.clear();
    for (const auto & p : path.points)
      prev_path_ls_.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
    for (const auto & p : path.left_bound) prev_left_bound_ls_.emplace_back(p.x, p.y);
    for (const auto & p : path.right_bound) prev_right_bound_ls_.emplace_back(p.x, p.y);
    prev_expanded_drivable_area_ = prev_expanded_drivable_area;
  }

  polygon_t get_previous_expanded_drivable_area() { return prev_expanded_drivable_area_; }

  void reset()
  {
    boost::geometry::clear(prev_path_ls_);
    boost::geometry::clear(prev_left_bound_ls_);
    boost::geometry::clear(prev_right_bound_ls_);
    boost::geometry::clear(prev_expanded_drivable_area_);
  }

  size_t calculate_replan_index(const PathWithLaneId & path, const double max_deviation) const
  {
    const auto path_ls = to_linestring(path);
    const auto left_bound_ls = to_linestring(path.left_bound);
    const auto right_bound_ls = to_linestring(path.right_bound);
    return std::min(
      {first_deviating_index(path_ls, prev_path_ls_, max_deviation),
       first_deviating_index(left_bound_ls, prev_left_bound_ls_, max_deviation),
       first_deviating_index(right_bound_ls, prev_right_bound_ls_, max_deviation)});
  }
};
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__REPLAN_CHECKER_HPP_
