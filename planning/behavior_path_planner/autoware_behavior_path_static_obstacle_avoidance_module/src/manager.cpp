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

#include "autoware/behavior_path_static_obstacle_avoidance_module/manager.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/parameter_helper.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
void StaticObstacleAvoidanceModuleManager::init(rclcpp::Node * node)
{
  using autoware::universe_utils::getOrDeclareParameter;
  using autoware_perception_msgs::msg::ObjectClassification;

  // init manager interface
  initInterface(node, {"left", "right"});

  param_listener_ = std::make_shared<::static_obstacle_avoidance::ParamListener>(
    node_->get_node_parameters_interface(), "avoidance");

  auto p = getParameter(param_listener_);

  parameters_ = std::make_shared<AvoidanceParameters>(p);
}

void StaticObstacleAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  parameters_ = std::make_shared<AvoidanceParameters>(getParameter(param_listener_));

  std::for_each(observers_.begin(), observers_.end(), [this](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(parameters_);
  });
}
}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
