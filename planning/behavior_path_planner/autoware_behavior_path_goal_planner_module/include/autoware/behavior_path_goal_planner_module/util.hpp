// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_candidate.hpp"
#include "autoware/behavior_path_goal_planner_module/pull_over_planner/pull_over_planner_base.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::goal_planner_utils
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using Shape = autoware_perception_msgs::msg::Shape;
using Polygon2d = autoware::universe_utils::Polygon2d;

lanelet::ConstLanelets getPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance);

lanelet::ConstLanelets generateBetweenEgoAndExpandedPullOverLanes(
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side,
  const geometry_msgs::msg::Pose ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double outer_road_offset,
  const double inner_road_offset);

/*
 * @brief generate polygon to extract objects
 * @param pull_over_lanes pull over lanes
 * @param left_side left side or right side
 * @param outer_offset outer offset from pull over lane boundary
 * @param inner_offset inner offset from pull over lane boundary
 * @return polygon to extract objects
 */
std::optional<Polygon2d> generateObjectExtractionPolygon(
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side, const double outer_offset,
  const double inner_offset);

PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside);

double calcLateralDeviationBetweenPaths(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path);
bool isReferencePath(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path,
  const double lateral_deviation_thresh);

std::optional<PathWithLaneId> cropPath(const PathWithLaneId & path, const Pose & end_pose);
PathWithLaneId cropForwardPoints(
  const PathWithLaneId & path, const size_t target_seg_idx, const double forward_length);

/**
 * @brief extend target_path by extend_length
 * @param target_path original target path to extend
 * @param reference_path reference path to extend
 * @param extend_length length to extend
 * @param remove_connected_zero_velocity flag to remove zero velocity if the last point of
 *                                       target_path has zero velocity
 * @return extended path
 */
PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const double extend_length, const bool remove_connected_zero_velocity);
/**
 * @brief extend target_path to extend_pose
 * @param target_path original target path to extend
 * @param reference_path reference path to extend
 * @param extend_pose pose to extend
 * @param remove_connected_zero_velocity flag to remove zero velocity if the last point of
 *                                       target_path has zero velocity
 * @return extended path
 */
PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const Pose & extend_pose, const bool remove_connected_zero_velocity);

std::vector<Polygon2d> createPathFootPrints(
  const PathWithLaneId & path, const double base_to_front, const double base_to_rear,
  const double width);

/**
 * @brief check if footprint intersects with given areas
 */
bool isIntersectingAreas(
  const LinearRing2d & footprint, const std::vector<lanelet::BasicPolygon2d> & areas);

/**
 * @brief check if footprint is within one of the areas
 */
bool isWithinAreas(
  const LinearRing2d & footprint, const std::vector<lanelet::BasicPolygon2d> & areas);

/**
 * @brief query BusStopArea polygons associated with given lanes
 */
std::vector<lanelet::BasicPolygon2d> getBusStopAreaPolygons(const lanelet::ConstLanelets & lanes);

bool checkObjectsCollision(
  const PathWithLaneId & path, const std::vector<double> & curvatures,
  const PredictedObjects & static_target_objects, const PredictedObjects & dynamic_target_objects,
  const BehaviorPathPlannerParameters & behavior_path_parameters,
  const double collision_check_margin, const bool extract_static_objects,
  const double maximum_deceleration,
  const double object_recognition_collision_check_max_extra_stopping_margin,
  std::vector<Polygon2d> & ego_polygons_expanded, const bool update_debug_data = false);

// debug
MarkerArray createPullOverAreaMarkerArray(
  const autoware::universe_utils::MultiPolygon2d area_polygons,
  const std_msgs::msg::Header & header, const std_msgs::msg::ColorRGBA & color, const double z);
MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createGoalCandidatesMarkerArray(
  const GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color);
MarkerArray createLaneletPolygonMarkerArray(
  const lanelet::CompoundPolygon3d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createNumObjectsToAvoidTextsMarkerArray(
  const GoalCandidates & goal_candidates, std::string && ns,
  const std_msgs::msg::ColorRGBA & color);
std::string makePathPriorityDebugMessage(
  const std::vector<size_t> & sorted_path_indices,
  const std::vector<PullOverPath> & pull_over_path_candidates,
  const std::map<size_t, size_t> & goal_id_to_index, const GoalCandidates & goal_candidates,
  const std::map<size_t, double> & path_id_to_rough_margin_map,
  const std::function<bool(const PullOverPath &)> & isSoftMargin,
  const std::function<bool(const PullOverPath &)> & isHighCurvature);
/**
 * @brief combine two points
 * @param points lane points
 * @param points_next next lane points
 * @return combined points
 */
lanelet::Points3d combineLanePoints(
  const lanelet::Points3d & points, const lanelet::Points3d & points_next);
/** @brief Create a lanelet that represents the departure check area.
 * @param [in] pull_over_lanes Lanelets that the vehicle will pull over to.
 * @param [in] route_handler RouteHandler object.
 * @return Lanelet that goal footprints should be inside.
 */
lanelet::Lanelet createDepartureCheckLanelet(
  const lanelet::ConstLanelets & pull_over_lanes, const route_handler::RouteHandler & route_handler,
  const bool left_side_parking);

std::optional<Pose> calcRefinedGoal(
  const Pose & goal_pose, const std::shared_ptr<RouteHandler> route_handler,
  const bool left_side_parking, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const GoalPlannerParameters & parameters);

std::optional<Pose> calcClosestPose(
  const lanelet::ConstLineString3d line, const Point & query_point);

autoware_perception_msgs::msg::PredictedObjects extract_dynamic_objects(
  const autoware_perception_msgs::msg::PredictedObjects & original_objects,
  const route_handler::RouteHandler & route_handler, const GoalPlannerParameters & parameters,
  const double vehicle_width);

bool is_goal_reachable_on_path(
  const lanelet::ConstLanelets current_lanes, const route_handler::RouteHandler & route_handler,
  const bool left_side_parking);

}  // namespace autoware::behavior_path_planner::goal_planner_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
