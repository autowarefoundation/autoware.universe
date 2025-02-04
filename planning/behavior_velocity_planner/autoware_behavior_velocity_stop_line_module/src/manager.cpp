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

#include "manager.hpp"

#include "autoware/universe_utils/ros/parameter.hpp"

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using lanelet::TrafficSign;

StopLineModuleManager::StopLineModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName()), planner_param_()
{
  const std::string ns(StopLineModuleManager::getModuleName());
  auto & p = planner_param_;
  p.stop_margin = getOrDeclareParameter<double>(node, ns + ".stop_margin");
  p.hold_stop_margin_distance =
    getOrDeclareParameter<double>(node, ns + ".hold_stop_margin_distance");
  p.stop_duration_sec = getOrDeclareParameter<double>(node, ns + ".stop_duration_sec");
}

std::vector<StopLineWithLaneId> StopLineModuleManager::getStopLinesWithLaneIdOnPath(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<StopLineWithLaneId> stop_lines_with_lane_id;

  for (const auto & m : planning_utils::getRegElemMapOnPath<TrafficSign>(
         path, lanelet_map, planner_data_->current_odometry->pose)) {
    const auto & traffic_sign_reg_elem = m.first;
    const int64_t lane_id = m.second.id();
    // Is stop sign?
    if (traffic_sign_reg_elem->type() != "stop_sign") {
      continue;
    }

    for (const auto & stop_line : traffic_sign_reg_elem->refLines()) {
      stop_lines_with_lane_id.emplace_back(stop_line, lane_id);
    }
  }

  return stop_lines_with_lane_id;
}

std::set<int64_t> StopLineModuleManager::getStopLineIdSetOnPath(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> stop_line_id_set;

  for (const auto & stop_line_with_lane_id : getStopLinesWithLaneIdOnPath(path, lanelet_map)) {
    stop_line_id_set.insert(stop_line_with_lane_id.first.id());
  }

  return stop_line_id_set;
}

void StopLineModuleManager::launchNewModules(const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & stop_line_with_lane_id :
       getStopLinesWithLaneIdOnPath(path, planner_data_->route_handler_->getLaneletMapPtr())) {
    const auto module_id = stop_line_with_lane_id.first.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<StopLineModule>(
        module_id, stop_line_with_lane_id.first, planner_param_,
        logger_.get_child("stop_line_module"), clock_, time_keeper_, planning_factor_interface_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
StopLineModuleManager::getModuleExpiredFunction(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto stop_line_id_set =
    getStopLineIdSetOnPath(path, planner_data_->route_handler_->getLaneletMapPtr());

  return [stop_line_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return stop_line_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::StopLineModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
