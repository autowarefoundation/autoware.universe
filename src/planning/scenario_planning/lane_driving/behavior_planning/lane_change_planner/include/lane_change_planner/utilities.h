/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LANE_CHANGE_PLANNER_UTILITIES_H
#define LANE_CHANGE_PLANNER_UTILITIES_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>

#include <lanelet2_core/geometry/Lanelet.h>

#include <limits>
#include <vector>

namespace lane_change_planner
{
namespace util
{
struct FrenetCoordinate3d
{
  double length;
  double distance;
  FrenetCoordinate3d() : length(0), distance(0) {}
};

double l2Norm(const geometry_msgs::Vector3 vector);

Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt);
std::vector<geometry_msgs::Point> convertToGeometryPointArray(
  const autoware_planning_msgs::PathWithLaneId & path);
geometry_msgs::PoseArray convertToGeometryPoseArray(
  const autoware_planning_msgs::PathWithLaneId & path);

autoware_perception_msgs::PredictedPath convertToPredictedPath(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Twist & vehicle_twist,
  const geometry_msgs::Pose & vehicle_pose);
autoware_perception_msgs::PredictedPath resamplePredictedPath(
  const autoware_perception_msgs::PredictedPath & input_path, const double resolution,
  const double duration);

bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point search_point_geom, FrenetCoordinate3d * frenet_coordinate);

geometry_msgs::Pose lerpByPose(
  const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2, const double t);
bool lerpByTimeStamp(
  const autoware_perception_msgs::PredictedPath & path, const ros::Time & t,
  geometry_msgs::Pose * lerped_pt);

double getDistance3d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2);
double getDistanceBetweenPredictedPaths(
  const autoware_perception_msgs::PredictedPath & path1,
  const autoware_perception_msgs::PredictedPath & path2, const double start_time,
  const double end_time, const double resolution, const bool use_vehicle_width,
  const double vehicle_width = 0.0);

std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & lanelets, const double start_arc_length = 0,
  const double end_arc_length = std::numeric_limits<double>::max());

const geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

autoware_planning_msgs::PathWithLaneId refinePath(
  const double search_radius_range, const double search_rad_range,
  const autoware_planning_msgs::PathWithLaneId & input, const geometry_msgs::Pose & goal,
  const int64_t goal_lane_id);
autoware_planning_msgs::PathWithLaneId removeOverlappingPoints(
  const autoware_planning_msgs::PathWithLaneId & input_path);

nav_msgs::OccupancyGrid convertLanesToDrivableArea(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::PoseStamped & current_pose,
  const double width, const double height, const double resolution);
double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets);

}  // namespace util
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_UTILITIES_H
