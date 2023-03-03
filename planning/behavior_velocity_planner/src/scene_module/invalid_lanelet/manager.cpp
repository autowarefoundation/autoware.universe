// Copyright 2021 Tier IV, Inc.
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

#include <lanelet2_extension/utility/query.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::InvalidLanelet;

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
  for (const auto & m : planning_utils::getRegElemMapOnPath<InvalidLanelet>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const int64_t module_id = m.first->id();
    const int64_t lane_id = m.second.id();

    if (!isModuleRegistered(module_id)) {
      // assign 1 invalid lanelet for each module
      registerModule(std::make_shared<InvalidLaneletModule>(
        module_id, lane_id, *m.first, planner_param_, logger_.get_child("invalid_lanelet_module"),
        clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
InvalidLaneletModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto invalid_lanelet_id_set = planning_utils::getRegElemIdSetOnPath<InvalidLanelet>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [invalid_lanelet_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return invalid_lanelet_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
