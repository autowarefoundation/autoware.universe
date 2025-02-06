// Copyright 2023 The Autoware Contributors
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_INTERFACE_HPP_

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{

class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(rclcpp::Node & node) = 0;
  virtual void plan(tier4_planning_msgs::msg::PathWithLaneId * path) = 0;
  virtual void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const tier4_planning_msgs::msg::PathWithLaneId & path) = 0;
  virtual const char * getModuleName() = 0;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__PLUGIN_INTERFACE_HPP_
