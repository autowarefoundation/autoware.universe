// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__TYPES_HPP_
#define SCENE_MODULE__OUT_OF_LANE__TYPES_HPP_

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane_utils
{
/// @brief parameters for the "out of lane" module
struct PlannerParam
{
  std::string mode;  // mode used to consider a conflict with an object

  double time_threshold;        // [s](mode="threshold") objects time threshold
  double intervals_ego_buffer;  // [s](mode="intervals") buffer to extend the ego time range
  double intervals_obj_buffer;  // [s](mode="intervals") buffer to extend the objects time range
  double ttc_threshold;  // [s](mode="ttc") threshold on time to collision between ego and an object

  double objects_min_vel;  //  # [m/s] objects lower than this velocity will be ignored

  double overlap_extra_length;  // [m] extra length to add around an overlap range
  double overlap_min_dist;      // [m] min distance inside another lane to consider an overlap
  // action to insert in the path if an object causes a conflict at an overlap
  double dist_buffer;
  double max_decel;
  double slow_velocity;
  double slow_dist_threshold;
  double stop_dist_threshold;
  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance
};

/// @brief range along the path where ego overlaps another lane
struct OverlapRange
{
  lanelet::ConstLanelet lane;
  size_t entering_path_idx{};
  size_t exiting_path_idx{};
  lanelet::BasicPoint2d entering_point;  // pose of the overlapping point closest along the lane
  lanelet::BasicPoint2d exiting_point;   // pose of the overlapping point furthest along the lane
  double inside_distance{};              // [m] how much ego footprint enters the lane
};
using OverlapRanges = std::vector<OverlapRange>;

/// @brief action taken by the "out of lane" module
struct Slowdown
{
  size_t target_path_idx{};             // we want to slowdown before this path index
  double velocity{};                    // desired slow down velocity
  lanelet::ConstLanelet lane_to_avoid;  // we want to slowdown before entering this lane
};
/// @brief bound of an overlap range (either the first, or last bound)
struct RangeBound
{
  size_t index;
  lanelet::BasicPoint2d point;
  double arc_length;
  double inside_distance;
};
/// @brief representation of a lane and its current overlap range
struct OtherLane
{
  bool range_is_open = false;
  RangeBound first_range_bound{};
  RangeBound last_range_bound{};  // TODO(Maxime): if we look ahead we do not need this
  lanelet::ConstLanelet lanelet;
  lanelet::BasicPolygon2d polygon;

  explicit OtherLane(lanelet::ConstLanelet ll) : lanelet(std::move(ll))
  {
    polygon = lanelet.polygon2d().basicPolygon();
  }

  [[nodiscard]] OverlapRange close_range()
  {
    OverlapRange range;
    range.lane = lanelet;
    range.entering_path_idx = first_range_bound.index;
    range.entering_point = first_range_bound.point;
    range.exiting_path_idx = last_range_bound.index;
    range.exiting_point = last_range_bound.point;
    range.inside_distance =
      std::max(first_range_bound.inside_distance, last_range_bound.inside_distance);
    range_is_open = false;
    last_range_bound = {};
    return range;
  }
};

struct EgoInfo
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path;
  size_t first_path_idx;
  double velocity;  // [m/s]
  geometry_msgs::msg::Pose pose;
};

}  // namespace behavior_velocity_planner::out_of_lane_utils

#endif  // SCENE_MODULE__OUT_OF_LANE__TYPES_HPP_
