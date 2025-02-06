// Copyright 2020 Tier IV, Inc.
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

#include "autoware/behavior_velocity_blind_spot_module/manager.hpp"

#include "autoware/behavior_velocity_blind_spot_module/util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <set>
#include <string>

namespace autoware::behavior_velocity_planner
{

BlindSpotModuleManager::BlindSpotModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(BlindSpotModuleManager::getModuleName());
  planner_param_ = PlannerParam::init(node, ns);
}

void BlindSpotModuleManager::launchNewModules(const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & ll : planning_utils::getLaneletsOnPath(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is turning lane?
    const std::string turn_direction_str = ll.attributeOr("turn_direction", "else");
    if (turn_direction_str != "left" && turn_direction_str != "right") {
      continue;
    }
    const auto turn_direction =
      turn_direction_str == "left" ? TurnDirection::LEFT : TurnDirection::RIGHT;

    registerModule(std::make_shared<BlindSpotModule>(
      module_id, lane_id, turn_direction, planner_data_, planner_param_,
      logger_.get_child("blind_spot_module"), clock_, time_keeper_, planning_factor_interface_));
    generateUUID(module_id);
    updateRTCStatus(
      getUUID(module_id), true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
      path.header.stamp);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
BlindSpotModuleManager::getModuleExpiredFunction(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_id_set = planning_utils::getLaneIdSetOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::BlindSpotModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
