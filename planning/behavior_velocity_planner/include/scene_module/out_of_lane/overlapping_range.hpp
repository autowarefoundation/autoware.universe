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

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <limits>
#include <vector>

namespace behavior_velocity_planner::out_of_lane_utils
{

struct Overlap
{
  double inside_distance = 0.0;  ///!< distance inside the overlap
  double min_arc_length = std::numeric_limits<double>::infinity();
  double max_arc_length = 0.0;
  lanelet::BasicPoint2d min_overlap_point{};  ///!< point with min arc length
  lanelet::BasicPoint2d max_overlap_point{};  ///!< point with max arc length
};

Overlap calculate_overlap(
  const lanelet::BasicPolygon2d & path_footprint, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelet & lanelet);

OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelet & lanelet,
  const PlannerParam & params);

OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params);

}  // namespace behavior_velocity_planner::out_of_lane_utils

#endif  // SCENE_MODULE__OUT_OF_LANE__OVERLAPPING_RANGE_HPP_
