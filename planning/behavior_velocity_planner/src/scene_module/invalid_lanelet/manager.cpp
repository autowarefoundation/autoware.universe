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

#include "scene_module/invalid_lanelet/manager.hpp"

namespace behavior_velocity_planner
{

InvalidLaneletModuleManager::InvalidLaneletModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = node.declare_parameter(ns + ".stop_margin", 1.5);
  planner_param_.print_debug_info = node.declare_parameter(ns + ".print_debug_info", false);
}

void InvalidLaneletModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & ll : planning_utils::getLaneletsOnPath(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    const std::string invalid_lanelet_attribute = ll.attributeOr("invalid_lanelet", "no");
    if (invalid_lanelet_attribute != "yes") {
      continue;
    }

    registerModule(std::make_shared<InvalidLaneletModule>(
      module_id, lane_id, planner_param_, logger_.get_child("invalid_lanelet"), clock_));
    generateUUID(module_id);
    updateRTCStatus(
      getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
InvalidLaneletModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_id_set = planning_utils::getLaneIdSetOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
