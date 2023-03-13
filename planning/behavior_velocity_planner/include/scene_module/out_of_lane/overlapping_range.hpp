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

#ifndef SCENE_MODULE__OUT_OF_LANE__OVERLAPPING_RANGE_HPP_
#define SCENE_MODULE__OUT_OF_LANE__OVERLAPPING_RANGE_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/io/svg/svg_mapper.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <vector>

namespace behavior_velocity_planner
{
namespace out_of_lane_utils
{
inline OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params)
{
  std::vector<OtherLane> other_lanes;
  for (const auto & lanelet : lanelets) other_lanes.emplace_back(lanelet);

  OverlapRanges ranges;

  std::ofstream svg("/tmp/debug.svg");
  boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 800, 800);

  for (const auto & path_footprint : path_footprints) mapper.add(path_footprint);

  for (auto i = 0UL; i < path_footprints.size(); ++i) {
    auto path_footprint = path_footprints[i];
    mapper.map(path_footprint, "opacity:0.5;fill-opacity:0.0;fill:red;stroke:red;stroke-width:1");
    for (auto & other_lane : other_lanes) {
      const auto & left_bound = other_lane.lanelet.leftBound2d().basicLineString();
      const auto & right_bound = other_lane.lanelet.rightBound2d().basicLineString();
      const auto overlap_left = boost::geometry::intersects(path_footprint, left_bound);
      const auto overlap_right = boost::geometry::intersects(path_footprint, right_bound);
      auto inside_dist = 0.0;
      lanelet::BasicPoint2d min_overlap_point;
      lanelet::BasicPoint2d max_overlap_point;
      auto min_arc_length = std::numeric_limits<double>::infinity();
      auto max_arc_length = 0.0;
      lanelet::BasicPolygons2d overlapping_polygons;
      if (overlap_left || overlap_right)  // TODO(Maxime): special case when both are overlapped
        boost::geometry::intersection(path_footprint, other_lane.polygon, overlapping_polygons);
      for (const auto & overlapping_polygon : overlapping_polygons) {
        mapper.map(
          overlapping_polygon, "opacity:0.5;fill-opacity:0.2;fill:red;stroke:red;stroke-width:1");
        for (const auto & point : overlapping_polygon) {
          if (overlap_left && overlap_right)
            inside_dist = 0.0;  // TODO(Maxime): boost::geometry::distance(left_bound, right_bound)
          else if (overlap_left)
            inside_dist = std::max(inside_dist, boost::geometry::distance(point, left_bound));
          else if (overlap_right)
            inside_dist = std::max(inside_dist, boost::geometry::distance(point, right_bound));
          geometry_msgs::msg::Pose p;
          p.position.x = point.x();
          p.position.y = point.y();
          const auto length = lanelet::utils::getArcCoordinates(path_lanelets, p).length;
          if (length > max_arc_length) {
            max_arc_length = length;
            max_overlap_point = point;
          }
          if (length < min_arc_length) {
            min_arc_length = length;
            min_overlap_point = point;
          }
        }
      }
      const auto has_overlap = inside_dist > params.overlap_min_dist;
      if (has_overlap) {  // open/update the range
        if (!other_lane.range_is_open) {
          other_lane.first_range_bound.index = i;
          other_lane.first_range_bound.point = min_overlap_point;
          other_lane.first_range_bound.arc_length = min_arc_length - params.overlap_extra_length;
          other_lane.first_range_bound.inside_distance = inside_dist;
          other_lane.range_is_open = true;
        }
        other_lane.last_range_bound.index = i;
        // TODO(Maxime): the last max_overlap_point does not go to go to the range
        other_lane.last_range_bound.point = max_overlap_point;
        mapper.map(max_overlap_point, "opacity:0.5;stroke:green;stroke-width:1", 2);
        other_lane.last_range_bound.arc_length = max_arc_length + params.overlap_extra_length;
        other_lane.last_range_bound.inside_distance = inside_dist;
      } else if (other_lane.range_is_open) {  // !has_overlap: close the range
        ranges.push_back(other_lane.closeRange());
        mapper.map(ranges.back().entering_point, "opacity:0.5;stroke:blue;stroke-width:1", 2);
        mapper.map(ranges.back().exiting_point, "opacity:0.5;stroke:green;stroke-width:1", 2);
      }
    }
  }
  // close all open ranges
  for (auto & other_lane : other_lanes)
    if (other_lane.range_is_open) {
      ranges.push_back(other_lane.closeRange());
      mapper.map(ranges.back().entering_point, "opacity:0.5;stroke:blue;stroke-width:1", 2);
      mapper.map(ranges.back().exiting_point, "opacity:0.5;stroke:green;stroke-width:1", 2);
    }
  return ranges;
}

}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__OVERLAPPING_RANGE_HPP_
