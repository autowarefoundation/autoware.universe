// Copyright 2022 Tier IV, Inc.
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

#ifndef STATIC_PATH_SMOOTHER__FUNCTIONS_HPP_
#define STATIC_PATH_SMOOTHER__FUNCTIONS_HPP_

#include "route_handler/route_handler.hpp"

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "autoware_auto_planning_msgs/msg/had_map_route.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <memory>
#include <string>
#include <vector>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;

HADMapBin::ConstSharedPtr create_map(
  const std::string & lanelet2_file_name, const rclcpp::Time & current_time);
std::vector<geometry_msgs::msg::Pose> create_check_points(
  const route_handler::RouteHandler & route_handler, const size_t start_lanelet_id,
  const size_t end_lanelet_id);
HADMapRoute plan_route(
  const HADMapBin::ConstSharedPtr map_bin_msg_ptr,
  const std::vector<geometry_msgs::msg::Pose> & check_points);
PathWithLaneId get_path_with_lane_id(
  const route_handler::RouteHandler & route_handler, const HADMapRoute & route,
  const geometry_msgs::msg::Pose & start_pose);

#endif  // STATIC_PATH_SMOOTHER__FUNCTIONS_HPP_
