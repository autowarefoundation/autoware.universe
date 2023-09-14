// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::SpeedBump;
using tier4_autoware_utils::getOrDeclareParameter;

TemplateModuleManager::TemplateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(getModuleName());
  auto dummy_parameter = getOrDeclareParameter<double>(node, ns + ".slow_start_margin");
}

void TemplateModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & speed_bump_with_lane_id : planning_utils::getRegElemMapOnPath<SpeedBump>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = speed_bump_with_lane_id.second.id();
    const auto module_id = speed_bump_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<TemplateModule>(
        module_id, lane_id, *speed_bump_with_lane_id.first, planner_param_,
        logger_.get_child("speed_bump_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TemplateModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto speed_bump_id_set = planning_utils::getRegElemIdSetOnPath<SpeedBump>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [speed_bump_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return speed_bump_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::TemplateModulePlugin, behavior_velocity_planner::PluginInterface)
