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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{
/// @brief parameters for the "out of lane" module
struct PlannerParam
{
  bool print_runtimes;
  double longitudinal_offset;
  double lateral_offset;
};

struct EgoData
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path{};
  size_t first_path_idx{};
  double velocity{};   // [m/s]
  double max_decel{};  // [m/s²]
  geometry_msgs::msg::Pose pose;
};

/// @brief debug data
struct DebugData
{
  void reset_data() {}
};

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // TYPES_HPP_
