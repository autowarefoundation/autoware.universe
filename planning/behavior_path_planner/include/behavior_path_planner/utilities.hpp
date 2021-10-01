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

#ifndef BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometry.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"

#include "opencv2/opencv.hpp"

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/route_handler.hpp"

namespace autoware_utils
{
template<>
inline geometry_msgs::msg::Pose getPose(
  const autoware_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose;
}
}  // namespace autoware_utils

namespace tf2
{
inline
void fromMsg(const geometry_msgs::msg::PoseStamped & msg, tf2::Stamped<tf2::Transform> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline
geometry_msgs::msg::Point & toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline
void fromMsg(const geometry_msgs::msg::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

template<>
inline
void doTransform(
  const geometry_msgs::msg::Point & t_in, geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}
}  // namespace tf2

namespace behavior_path_planner
{
namespace util
{
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::DynamicObject;
using autoware_perception_msgs::msg::DynamicObjectArray;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPointWithLaneId;
using autoware_planning_msgs::msg::PathWithLaneId;
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;

struct FrenetCoordinate3d
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

// data conversions

Path convertToPathFromPathWithLaneId(const PathWithLaneId & path_with_lane_id);

std::vector<Point> convertToPointArray(const PathWithLaneId & path);

std::vector<Point> convertToGeometryPointArray(const PathWithLaneId & path);

PoseArray convertToGeometryPoseArray(const PathWithLaneId & path);

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const Twist & vehicle_twist,
  const Pose & vehicle_pose, const double duration, const double resolution,
  const double acceleration);

bool convertToFrenetCoordinate3d(
  const PathWithLaneId & path,
  const Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate);

bool convertToFrenetCoordinate3d(
  const std::vector<Point> & linestring,
  const Point search_point_geom, FrenetCoordinate3d * frenet_coordinate);

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets);

// distance (arclength) calculation

double l2Norm(const Vector3 vector);

double getDistanceToEndOfLane(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets);

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const Pose & pose);

// object collision check

Pose lerpByPose(const Pose & p1, const Pose & p2, const double t);

Point lerpByLength(const std::vector<Point> & array, const double length);

bool lerpByTimeStamp(
  const PredictedPath & path, const rclcpp::Time & t, Pose * lerped_pt);

bool lerpByDistance(
  const behavior_path_planner::PullOutPath & path, const double & s,
  Pose * lerped_pt, const lanelet::ConstLanelets & road_lanes);

bool calcObjectPolygon(const DynamicObject & object, Polygon2d * object_polygon);

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution,
  const double duration);

double getDistanceBetweenPredictedPaths(
  const PredictedPath & path1, const PredictedPath & path2, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObject(
  const DynamicObject & object, const PredictedPath & path, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObjectPolygon(
  const DynamicObject & object, const PullOutPath & ego_path,
  const autoware_utils::LinearRing2d & vehicle_footprint, double distance_resolution,
  const lanelet::ConstLanelets & road_lanes);

/**
 * @brief Get index of the obstacles inside the lanelets with start and end length
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const DynamicObjectArray & objects, const lanelet::ConstLanelets & lanelets,
  const double start_arc_length,
  const double end_arc_length);

/**
 * @brief Get index of the obstacles inside the lanelets
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const DynamicObjectArray & objects, const lanelet::ConstLanelets & target_lanelets);

std::vector<size_t> filterObjectsByPath(
  const DynamicObjectArray & objects, const std::vector<size_t> & object_indices,
  const PathWithLaneId & ego_path, const double vehicle_width);

DynamicObjectArray filterObjectsByVelocity(const DynamicObjectArray & objects, double lim_v);

DynamicObjectArray filterObjectsByVelocity(
  const DynamicObjectArray & objects, double min_v, double max_v);

// drivable area generation

void occupancyGridToImage(const OccupancyGrid & occupancy_grid, cv::Mat * cv_image);

void imageToOccupancyGrid(const cv::Mat & cv_image, OccupancyGrid * occupancy_grid);

cv::Point toCVPoint(
  const Point & geom_point, const double width_m, const double height_m,
  const double resolution);

OccupancyGrid generateDrivableArea(
  const lanelet::ConstLanelets & lanes, const PoseStamped & current_pose,
  const double width, const double height, const double resolution, const double vehicle_length,
  const RouteHandler & route_handler);

// goal management

bool setGoal(
  const double search_radius_range, const double search_rad_range,
  const PathWithLaneId & input, const Pose & goal,
  const int64_t goal_lane_id, PathWithLaneId * output_ptr);

const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range,
  const PathWithLaneId & input, const Pose & goal, const int64_t goal_lane_id);

PathWithLaneId removeOverlappingPoints(
  const PathWithLaneId & input_path);

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id);

// path management

// TODO(Horibe) There is a similar function in route_handler. Check.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data);

PathPointWithLaneId insertStopPoint(double length, PathWithLaneId * path);

double getDistanceToShoulderBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose);

// misc

lanelet::Polygon3d getVehiclePolygon(
  const Pose & vehicle_pose, const double vehicle_width, const double base_link2front);

std::vector<Polygon2d> getTargetLaneletPolygons(
  const lanelet::ConstLanelets & lanelets, const Pose & pose,
  const double check_length, const std::string & target_type);

void shiftPose(Pose * pose, double shift_length);

}  // namespace util
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_
