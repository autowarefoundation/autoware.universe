// Copyright 2024 The Autoware Contributors
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

#ifndef MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_
#define MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_

#include "planner_data.hpp"
#include "velocity_planning_result.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <vector>

namespace motion_velocity_planner
{

class PluginModuleInterface
{
public:
  virtual ~PluginModuleInterface() = default;
  virtual void init(rclcpp::Node & node) = 0;
  virtual void update_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;
  virtual VelocityPlanningResult plan(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const PlannerData & planner_data) = 0;
  virtual std::string get_module_name() const = 0;
};

}  // namespace motion_velocity_planner

#endif  // MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_
