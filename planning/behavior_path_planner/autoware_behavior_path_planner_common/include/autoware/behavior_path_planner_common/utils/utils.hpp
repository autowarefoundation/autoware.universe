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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__UTILS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;

using autoware::route_handler::RouteHandler;
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::Polygon2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

static constexpr double eps = 0.01;

struct PolygonPoint
{
  geometry_msgs::msg::Point point;
  size_t bound_seg_idx{0};
  double lon_dist_to_segment{0.0};
  double lat_dist_to_bound{0.0};

  bool is_after(const PolygonPoint & other_point) const
  {
    if (bound_seg_idx == other_point.bound_seg_idx) {
      return other_point.lon_dist_to_segment < lon_dist_to_segment;
    }
    return other_point.bound_seg_idx < bound_seg_idx;
  }

  bool is_outside_bounds(const bool is_on_right) const
  {
    if (is_on_right) {
      return lat_dist_to_bound < 0.0;
    }
    return 0.0 < lat_dist_to_bound;
  };
};

struct FrenetPoint
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

// data conversions
template <class T>
FrenetPoint convertToFrenetPoint(
  const T & points, const Point & search_point_geom, const size_t seg_idx)
{
  FrenetPoint frenet_point;

  const double longitudinal_length =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, search_point_geom);
  frenet_point.length =
    autoware::motion_utils::calcSignedArcLength(points, 0, seg_idx) + longitudinal_length;
  frenet_point.distance =
    autoware::motion_utils::calcLateralOffset(points, search_point_geom, seg_idx);

  return frenet_point;
}

/**
 * @brief Converts a Lanelet point to a ROS Pose message.
 *
 * This function converts a point from a Lanelet map to a ROS geometry_msgs::msg::Pose.
 * It sets the position from the point and calculates the orientation (yaw) based on the target
 * lane.
 *
 * @tparam LaneletPointType The type of the input point.
 *
 * @param[in] src_point The point to convert.
 * @param[in] target_lane The lanelet used to determine the orientation.
 *
 * @return A Pose message with the position and orientation of the point.
 */
template <class LaneletPointType>
Pose to_geom_msg_pose(const LaneletPointType & src_point, const lanelet::ConstLanelet & target_lane)
{
  const auto point = lanelet::utils::conversion::toGeomMsgPt(src_point);
  const auto yaw = lanelet::utils::getLaneletAngle(target_lane, point);
  geometry_msgs::msg::Pose pose;
  pose.position = point;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  pose.orientation = tf2::toMsg(quat);
  return pose;
}

// distance (arclength) calculation

double l2Norm(const Vector3 vector);

double getDistanceToEndOfLane(const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Calculates the distance to the next intersection.
 * @param current_pose Ego pose.
 * @param lanelets Lanelets to check.
 * @return Distance.
 */
double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Calculates the distance to the next crosswalk.
 * @param current_pose Ego pose.
 * @param lanelets Lanelets to check.
 * @return Distance.
 */
double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose, const lanelet::ConstLanelets & lanelets);

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const Pose & pose);

/**
 * @brief Check collision between ego path footprints with extra longitudinal stopping margin and
 * objects.
 * @return Has collision or not
 */
bool checkCollisionWithExtraStoppingMargin(
  const PathWithLaneId & ego_path, const PredictedObjects & dynamic_objects,
  const double base_to_front, const double base_to_rear, const double width,
  const double maximum_deceleration, const double margin, const double max_stopping_margin);

/**
 * @brief Check collision between ego path footprints and objects.
 * @return Has collision or not
 */
bool checkCollisionBetweenPathFootprintsAndObjects(
  const autoware::universe_utils::LinearRing2d & vehicle_footprint, const PathWithLaneId & ego_path,
  const PredictedObjects & dynamic_objects, const double margin);

/**
 * @brief Check collision between ego footprints and objects.
 * @return Has collision or not
 */
bool checkCollisionBetweenFootprintAndObjects(
  const autoware::universe_utils::LinearRing2d & vehicle_footprint, const Pose & ego_pose,
  const PredictedObjects & dynamic_objects, const double margin);

/**
 * @brief calculate lateral distance from ego pose to object
 * @return distance from ego pose to object
 */
double calcLateralDistanceFromEgoToObject(
  const Pose & ego_pose, const double vehicle_width, const PredictedObject & dynamic_object);

/**
 * @brief calculate longitudinal distance from ego pose to object
 * @return distance from ego pose to object
 */
double calc_longitudinal_distance_from_ego_to_object(
  const Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const PredictedObject & dynamic_object);

/**
 * @brief calculate minimum longitudinal distance from ego pose to objects
 * @return minimum distance from ego pose to objects
 */
double calcLongitudinalDistanceFromEgoToObjects(
  const Pose & ego_pose, double base_link2front, double base_link2rear,
  const PredictedObjects & dynamic_objects);

// drivable area generation
lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes);
lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes);
std::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes);
std::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes);
// goal management

/**
 * @brief Modify the path points near the goal to smoothly connect the input path and the goal
 * point
 * @details Remove the path points that are forward from the goal by the distance of
 * search_radius_range. Then insert the goal into the path. The previous goal point generated
 * from the goal posture information is also inserted for the smooth connection of the goal pose.
 * @param [in] search_radius_range distance on path to be modified for goal insertion
 * @param [in] search_rad_range [unused]
 * @param [in] input original path
 * @param [in] goal original goal pose
 * @param [in] goal_lane_id [unused]
 * @param [in] output_ptr output path with modified points for the goal
 */
bool set_goal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id, PathWithLaneId * output_ptr);

/**
 * @brief Recreate the goal pose to prevent the goal point being too far from the lanelet, which
 *  causes the path to twist near the goal.
 * @details Return the goal point projected on the straight line of the segment of lanelet
 *  closest to the original goal.
 * @param [in] goal original goal pose
 * @param [in] goal_lanelet lanelet containing the goal pose
 */
const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

/**
 * @brief Recreate the path with a given goal pose.
 * @param search_radius_range Searching radius.
 * @param search_rad_range Searching angle.
 * @param input Input path.
 * @param goal Goal pose.
 * @param goal_lane_id Lane ID of goal lanelet.
 * @return Recreated path
 */
PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id);

bool isAllowedGoalModification(const std::shared_ptr<RouteHandler> & route_handler);
bool checkOriginalGoalIsInShoulder(const std::shared_ptr<RouteHandler> & route_handler);

/**
 * @brief Checks if the given pose is inside the given lanes.
 * @param pose Ego pose.
 * @param lanes Lanelets to check.
 * @return True if the ego pose is inside the lanes.
 */
bool isInLanelets(const Pose & pose, const lanelet::ConstLanelets & lanes);

/**
 * @brief Checks if the given pose is inside the given lane within certain yaw difference.
 * @param current_pose Ego pose.
 * @param lanelet Lanelet to check.
 * @param yaw_threshold Yaw angle difference threshold.
 * @param radius Search radius
 * @return True if the ego pose is inside the lane.
 */
bool isInLaneletWithYawThreshold(
  const Pose & current_pose, const lanelet::ConstLanelet & lanelet, const double yaw_threshold,
  const double radius = 0.0);

bool isEgoOutOfRoute(
  const Pose & self_pose, const lanelet::ConstLanelet & closest_road_lane,
  const std::optional<PoseWithUuidStamped> & modified_goal,
  const std::shared_ptr<RouteHandler> & route_handler);

/**
 * @brief Checks if the given pose is inside the original lane.
 * @param current_lanes Original lanes.
 * @param current_pose Ego pose.
 * @param common_param Parameters used for behavior path planner.
 * @param outer_margin Allowed margin.
 * @return True if the ego pose is inside the original lane.
 */
bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin = 0.0);

bool isEgoWithinOriginalLane(
  const lanelet::BasicPolygon2d & lane_polygon, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin = 0.0);

// path management

// TODO(Horibe) There is a similar function in route_handler. Check.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief Inserts a stop point with given length
 * @param length Distance to insert stop point.
 * @param path Original path.
 * @return Inserted stop point.
 */
PathPointWithLaneId insertStopPoint(const double length, PathWithLaneId & path);

/**
 * @brief Calculates distance to lane boundary.
 * @param lanelet Target lanelet.
 * @param position Ego position.
 * @param left_side Whether to check left side boundary.
 * @return Distance to boundary.
 */
double getSignedDistanceFromLaneBoundary(
  const lanelet::ConstLanelet & lanelet, const Point & position, const bool left_side);

double getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, const bool left_side);

/**
 * @brief Calculates distance to lane boundary.
 * @param lanelet Target lanelet.
 * @param vehicle_width Ego vehicle width.
 * @param base_link2front Ego vehicle distance from base link to front.
 * @param base_link2rear Ego vehicle distance from base link to rear.
 * @param vehicle_pose Ego vehicle pose.
 * @param left_side Whether to check left side boundary.
 * @return Distance to boundary.
 */
std::optional<double> getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const Pose & vehicle_pose, const bool left_side);

// misc

/**
 * @brief Convert lanelet to 2D polygon.
 * @param lanelet Target lanelet.
 * @return Polygon
 */
Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet);

Polygon2d toPolygon2d(const lanelet::BasicPolygon2d & polygon);

PathWithLaneId getCenterLinePathFromLanelet(
  const lanelet::ConstLanelet & current_route_lanelet,
  const std::shared_ptr<const PlannerData> & planner_data);

// route handler
PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter);

// object label
std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification);

lanelet::ConstLanelets getCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_path_length,
  const double forward_path_length);
lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets extendNextLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route = false);

lanelet::ConstLanelets extendPrevLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route = false);

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes);

/**
 * @brief Retrieves sequences of preceding lanelets from the target lanes.
 * @param route_handler Reference to the route handler.
 * @param target_lanes The set of target lanelets.
 * @param current_pose The current pose of ego vehicle.
 * @param backward_length The backward search length [m].
 * @return A vector of lanelet sequences that precede the target lanes
 */
std::vector<lanelet::ConstLanelets> getPrecedingLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length);

/**
 * @brief Retrieves all preceding lanelets as a flat list.
 * @param route_handler Reference to the route handler.
 * @param target_lanes The set of target lanelets.
 * @param current_pose The current pose of ego vehicle.
 * @param backward_length The backward search length [m].
 * @return Preceding lanelets within the specified backward length.
 */

lanelet::ConstLanelets getBackwardLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length);

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length,
  const double forward_length, const bool forward_only_in_route);

lanelet::ConstLanelets getExtendedCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const double backward_length, const double forward_length, const bool forward_only_in_route);

/**
 * @brief Calculates the sequence of lanelets around a given pose within a specified range.
 * @param route_handler A shared pointer to the RouteHandler for accessing route and lanelet
 * information.
 * @param pose The reference pose to locate the lanelets around.
 * @param forward_length The length of the route to extend forward from the reference pose.
 * @param backward_length The length of the route to extend backward from the reference pose.
 * @param dist_threshold The maximum allowable distance to consider a lanelet as closest.
 * @param yaw_threshold The maximum allowable yaw difference (in radians) for lanelet matching.
 * @return A sequence of lanelets around the given pose.
 */
lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<RouteHandler> & route_handler, const geometry_msgs::msg::Pose & pose,
  const double forward_length, const double backward_length,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

/**
 * @brief Checks whether the relative angles between consecutive triplets of points in a path remain
 * within a specified threshold.
 * @param path Input path.
 * @param angle_threshold The maximum allowable angle in radians.
 * @return True if all relative angles are within the threshold or the path has fewer than three
 * points.
 */
bool checkPathRelativeAngle(const PathWithLaneId & path, const double angle_threshold);

lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler);

/**
 * @brief Converts camel case string to snake case string.
 * @param input_str Input string.
 * @return String
 */
std::string convertToSnakeCase(const std::string & input_str);

std::optional<lanelet::Polygon3d> getPolygonByPoint(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstPoint3d & point,
  const std::string & polygon_name);

template <class T>
size_t findNearestSegmentIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  if (nearest_idx) {
    return nearest_idx.value();
  }

  return autoware::motion_utils::findNearestSegmentIndex(points, pose.position);
}
}  // namespace autoware::behavior_path_planner::utils

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__UTILS_HPP_
