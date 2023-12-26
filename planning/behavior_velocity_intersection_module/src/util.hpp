// Copyright 2020 Tier IV, Inc.
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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "util_type.hpp"

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace util
{

std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold);

bool hasLaneIds(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p,
  const std::set<lanelet::Id> & ids);
std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids);

std::optional<size_t> getFirstPointInsidePolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval, const lanelet::CompoundPolygon3d & polygon,
  const bool search_forward = true);

/**
 * @brief check if ego is over the target_idx. If the index is same, compare the exact pose
 * @param path path
 * @param closest_idx ego's closest index on the path
 * @param current_pose ego's exact pose
 * @return true if ego is over the target_idx
 */
bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx);

bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx);

/*
lanelet::ConstLanelets extendedAdjacentDirectionLanes(
lanelet::LaneletMapConstPtr map, const lanelet::routing::RoutingGraphPtr routing_graph,
lanelet::ConstLanelet lane);
*/

std::optional<tier4_autoware_utils::Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr);

bool hasAssociatedTrafficLight(lanelet::ConstLanelet lane);

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
  const rclcpp::Logger logger);

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state);

std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length);

}  // namespace util
}  // namespace behavior_velocity_planner

#endif  // UTIL_HPP_
