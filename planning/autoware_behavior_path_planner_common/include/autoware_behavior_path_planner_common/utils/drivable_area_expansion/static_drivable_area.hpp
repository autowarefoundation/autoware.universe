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

#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__STATIC_DRIVABLE_AREA_HPP_  // NOLINT
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__STATIC_DRIVABLE_AREA_HPP_  // NOLINT

#include <autoware_behavior_path_planner_common/utils/utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner::utils
{
using drivable_area_expansion::DrivableAreaExpansionParameters;

std::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes);

std::vector<DrivableLanes> cutOverlappedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes);

std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & current_lanes);

std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & shoulder_lanes);

std::vector<DrivableLanes> getNonOverlappingExpandedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const DrivableAreaExpansionParameters & parameters);
void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const bool enable_expanding_freespace_areas,
  const std::shared_ptr<const PlannerData> planner_data, const bool is_driving_forward = true);

void generateDrivableArea(
  PathWithLaneId & path, const double vehicle_length, const double offset,
  const bool is_driving_forward = true);

/**
 * @brief Expand the borders of the given lanelets
 * @param [in] drivable_lanes lanelets to expand
 * @param [in] left_bound_offset [m] expansion distance of the left bound
 * @param [in] right_bound_offset [m] expansion distance of the right bound
 * @param [in] types_to_skip linestring types that will not be expanded
 * @return expanded lanelets
 */
std::vector<DrivableLanes> expandLanelets(
  const std::vector<DrivableLanes> & drivable_lanes, const double left_bound_offset,
  const double right_bound_offset, const std::vector<std::string> & types_to_skip = {});

void extractObstaclesFromDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableAreaInfo::Obstacle> & obstacles);

std::pair<std::vector<lanelet::ConstPoint3d>, bool> getBoundWithFreeSpaceAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::vector<lanelet::ConstPoint3d> & other_side_bound,
  const std::shared_ptr<const PlannerData> planner_data, const bool is_left);

std::vector<lanelet::ConstPoint3d> getBoundWithHatchedRoadMarkings(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<RouteHandler> & route_handler);

std::vector<lanelet::ConstPoint3d> getBoundWithIntersectionAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left);

std::vector<geometry_msgs::msg::Point> calcBound(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<DrivableLanes> & drivable_lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const bool enable_expanding_freespace_areas, const bool is_left,
  const bool is_driving_forward = true);

std::vector<geometry_msgs::msg::Point> postProcess(
  const std::vector<geometry_msgs::msg::Point> & original_bound, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left,
  const bool is_driving_forward = true);

DrivableAreaInfo combineDrivableAreaInfo(
  const DrivableAreaInfo & drivable_area_info1, const DrivableAreaInfo & drivable_area_info2);

lanelet::ConstLanelets combineLanelets(
  const lanelet::ConstLanelets & base_lanes, const lanelet::ConstLanelets & added_lanes);

std::vector<DrivableLanes> combineDrivableLanes(
  const std::vector<DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<DrivableLanes> & new_drivable_lanes_vec);

}  // namespace autoware::behavior_path_planner::utils

// clang-format off
#endif  // AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__STATIC_DRIVABLE_AREA_HPP_  // NOLINT
// clang-format on
