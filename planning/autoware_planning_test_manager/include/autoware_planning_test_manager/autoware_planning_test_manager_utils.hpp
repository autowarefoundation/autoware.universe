// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#define AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

namespace autoware_planning_test_manager::utils
{
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

Pose createPoseFromLaneID(const lanelet::Id & lane_id);

// Function to create a route from given start and goal lanelet ids
// start pose and goal pose are set to the middle of the lanelet
LaneletRoute makeBehaviorRouteFromLaneId(const int & start_lane_id, const int & goal_lane_id);

Odometry makeInitialPoseFromLaneId(const lanelet::Id & lane_id);

}  // namespace autoware_planning_test_manager::utils
#endif  // AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
