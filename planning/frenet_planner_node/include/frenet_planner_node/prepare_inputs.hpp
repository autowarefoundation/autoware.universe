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

#ifndef FRENET_PLANNER_NODE__PREPARE_INPUTS_HPP_
#define FRENET_PLANNER_NODE__PREPARE_INPUTS_HPP_

#include "frenet_planner/structures.hpp"
#include "frenet_planner/transform/spline_transform.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <vector>

namespace frenet_planner_node
{

frenet_planner::Constraints prepareConstraints(
  const nav_msgs::msg::OccupancyGrid & drivable_area,
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects);
frenet_planner::SamplingParameters prepareSamplingParameters(
  const frenet_planner::FrenetState & initial_state,
  const autoware_auto_planning_msgs::msg::Path & path, const double base_duration,
  const frenet_planner::transform::Spline2D & path_spline);
frenet_planner::transform::Spline2D preparePathSpline(
  const autoware_auto_planning_msgs::msg::Path & path_msg);
frenet_planner::Trajectory preparePreviousTrajectory(
  const frenet_planner::Trajectory & prev_trajectory,
  const frenet_planner::transform::Spline2D & path_spline);
}  // namespace frenet_planner_node

#endif  // FRENET_PLANNER_NODE__PREPARE_INPUTS_HPP_
