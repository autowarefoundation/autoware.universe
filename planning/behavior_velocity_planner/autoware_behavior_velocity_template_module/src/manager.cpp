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

#include "manager.hpp"

#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;

TemplateModuleManager::TemplateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(TemplateModuleManager::getModuleName());
  dummy_parameter_ = getOrDeclareParameter<double>(node, ns + ".dummy");
}

void TemplateModuleManager::launchNewModules(
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(std::make_shared<TemplateModule>(
      module_id, logger_.get_child(getModuleName()), clock_, time_keeper_,
      planning_factor_interface_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TemplateModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  return []([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
    return false;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::TemplateModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
