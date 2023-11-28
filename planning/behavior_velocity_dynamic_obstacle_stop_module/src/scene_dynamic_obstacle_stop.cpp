// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include "scene_dynamic_obstacle_stop.hpp"

#include "debug.hpp"
#include "footprint.hpp"
#include "types.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), params_(std::move(planner_param))
{
  velocity_factor_.init(VelocityFactor::UNKNOWN);
}

std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const PlannerParam & params)
{
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filtered_objects;
  for (const auto & object : objects.objects) {
    const auto vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const auto width = object.shape.dimensions.y + params.extra_object_width;
    const auto lateral_offset = std::abs(motion_utils::calcLateralOffset(
      path.points, object.kinematics.initial_pose_with_covariance.pose.position));
    if (
      vel >= params.minimum_object_velocity && lateral_offset <= width / 2 + params.lateral_offset)
      filtered_objects.push_back(object);
  }
  return filtered_objects;
}

bool DynamicObstacleStopModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_.reset_data();
  *stop_reason = planning_utils::initializeStopReason(StopReason::OBSTACLE_STOP);
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path.points = path->points;
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(ego_data.path.points, ego_data.pose.position);
  motion_utils::removeOverlapPoints(ego_data.path.points);
  ego_data.velocity = planner_data_->current_velocity->twist.linear.x;
  ego_data.max_decel = -planner_data_->max_stop_acceleration_threshold;

  const auto dynamic_obstacles =
    filter_predicted_objects(*planner_data_->predicted_objects, ego_data.path, params_);

  const auto obstacle_forward_footprints = make_forward_footprints(dynamic_obstacles, params_);

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(logger_, "Total time = %2.2fus\n", total_time_us);
  debug_data_.dynamic_obstacles = dynamic_obstacles;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  return true;
}

MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
  // dynamic obstacles
  const auto obstacle_markers = debug::make_dynamic_obstacle_markers(debug_data_.dynamic_obstacles);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), obstacle_markers.begin(), obstacle_markers.end());
  const auto delete_obstacle_markers = debug::make_delete_markers(
    obstacle_markers.size(), debug_data_.prev_dynamic_obstacles_nb, "dynamic_obstacles");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_obstacle_markers.begin(),
    delete_obstacle_markers.end());
  // dynamic obstacles footprints
  const auto obstacle_footprint_markers =
    debug::make_polygon_markers(debug_data_.obstacle_footprints, "dynamic_obstacles_footprints", z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), obstacle_footprint_markers.begin(),
    obstacle_footprint_markers.end());
  const auto delete_footprint_markers = debug::make_delete_markers(
    obstacle_footprint_markers.size(), debug_data_.prev_dynamic_obstacles_nb,
    "dynamic_obstacles_footprints");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_footprint_markers.begin(),
    delete_footprint_markers.end());

  debug_data_.prev_dynamic_obstacles_nb = obstacle_markers.size();
  return debug_marker_array;
}

motion_utils::VirtualWalls DynamicObstacleStopModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "dynamic_obstacle_stop";
  wall.longitudinal_offset = params_.longitudinal_offset;
  wall.style = motion_utils::VirtualWallType::stop;
  // for (const auto & stop : debug_data_.stops) {
  //   wall.pose = stop
  //   virtual_walls.push_back(wall);
  // }
  return virtual_walls;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
