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

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(logger_, "Total time = %2.2fus\n", total_time_us);
  return true;
}

MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  // constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
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
