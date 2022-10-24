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

#ifndef STATIC_CENTERLINE_OPTIMIZER__UTILS_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__UTILS_HPP_

#include "route_handler/route_handler.hpp"
#include "static_centerline_optimizer/type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
namespace utils
{
HADMapBin::ConstSharedPtr create_map(
  rclcpp::Node & node, const std::string & lanelet2_file_name, const rclcpp::Time & current_time);

std::vector<geometry_msgs::msg::Pose> create_check_points(
  const RouteHandler & route_handler, const size_t start_lanelet_id, const size_t end_lanelet_id);

HADMapRoute plan_route(
  const HADMapBin::ConstSharedPtr map_bin_msg_ptr,
  const std::vector<geometry_msgs::msg::Pose> & check_points);

geometry_msgs::msg::Pose get_center_pose(
  const RouteHandler & route_handler, const size_t lanelet_id);

PathWithLaneId get_path_with_lane_id(
  const RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double nearset_ego_dist_threshold,
  const double nearest_ego_yaw_threshold);

void update_centerline(
  RouteHandler & route_handler, const lanelet::ConstLanelets & lanelets,
  const std::vector<TrajectoryPoint> & new_centerline);
}  // namespace utils
}  // namespace static_centerline_optimizer

#endif  // STATIC_CENTERLINE_OPTIMIZER__UTILS_HPP_
