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

#ifndef AUTOWARE__PATH_SMOOTHER__TYPE_ALIAS_HPP_
#define AUTOWARE__PATH_SMOOTHER__TYPE_ALIAS_HPP_

#include "autoware_internal_debug_msgs/msg/float64_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/string_stamped.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

namespace autoware::path_smoother
{
// std_msgs
using std_msgs::msg::Header;
// planning
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
// navigation
using nav_msgs::msg::Odometry;
// debug
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_debug_msgs::msg::StringStamped;
}  // namespace autoware::path_smoother

#endif  // AUTOWARE__PATH_SMOOTHER__TYPE_ALIAS_HPP_
