/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
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

#pragma once

#include <bezier_sampler/bezier_sampling.hpp>
#include <eigen3/Eigen/Core>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <utility>
#include <vector>

namespace bezier_sampler
{
/// @brief split the given vector of PathPoint into fixed arc-length segments
std::vector<std::pair<sampler_common::State, sampler_common::State>> splitPath(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path, double split_length,
  double max_length, int ego_pose_index);
/// @brief split the given vector of PathPoint into segments with a maximum curvature integral
std::vector<std::pair<sampler_common::State, sampler_common::State>> splitPathByCurvature(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path, double split_curvature);
/// @brief calculates the Menger curvature between the 3 given PathPoint
inline double curvature(
  const autoware_auto_planning_msgs::msg::PathPoint & p0,
  const autoware_auto_planning_msgs::msg::PathPoint & p1,
  const autoware_auto_planning_msgs::msg::PathPoint & p2)
{
  const double x0 = p0.pose.position.x;
  const double x1 = p1.pose.position.x;
  const double x2 = p2.pose.position.x;
  const double y0 = p0.pose.position.y;
  const double y1 = p1.pose.position.y;
  const double y2 = p2.pose.position.y;
  const double angle = std::atan2(y2 - y0, x2 - x0) - std::atan2(y1 - y0, x1 - x0);
  return 2 * std::sin(angle) / std::sqrt((x2 - x0) * (x2 - x0) + (y2 - y0) * (y2 - y0));
}
}  // namespace bezier_sampler
