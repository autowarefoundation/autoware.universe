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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using lanelet::autoware::NoStoppingArea;

NoStoppingAreaModuleManager::NoStoppingAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(NoStoppingAreaModuleManager::getModuleName());
  auto & pp = planner_param_;
  const auto & vi = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  pp.state_clear_time = getOrDeclareParameter<double>(node, ns + ".state_clear_time");
  pp.stuck_vehicle_vel_thr = getOrDeclareParameter<double>(node, ns + ".stuck_vehicle_vel_thr");
  pp.stop_margin = getOrDeclareParameter<double>(node, ns + ".stop_margin");
  pp.dead_line_margin = getOrDeclareParameter<double>(node, ns + ".dead_line_margin");
  pp.stop_line_margin = getOrDeclareParameter<double>(node, ns + ".stop_line_margin");
  pp.detection_area_length = getOrDeclareParameter<double>(node, ns + ".detection_area_length");
  pp.stuck_vehicle_front_margin =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle_front_margin");
  pp.path_expand_width = vi.vehicle_width_m * 0.5;
}

void NoStoppingAreaModuleManager::launchNewModules(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & m : planning_utils::getRegElemMapOnPath<NoStoppingArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const int64_t module_id = m.first->id();
    const int64_t lane_id = m.second.id();

    if (!isModuleRegistered(module_id)) {
      // assign 1 no stopping area for each module
      registerModule(std::make_shared<NoStoppingAreaModule>(
        module_id, lane_id, *m.first, planner_param_, logger_.get_child("no_stopping_area_module"),
        clock_, time_keeper_, planning_factor_interface_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, State::WAITING_FOR_EXECUTION,
        std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
NoStoppingAreaModuleManager::getModuleExpiredFunction(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto no_stopping_area_id_set = planning_utils::getRegElemIdSetOnPath<NoStoppingArea>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return
    [no_stopping_area_id_set](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
      return no_stopping_area_id_set.count(scene_module->getModuleId()) == 0;
    };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::NoStoppingAreaModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
