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

#include "behavior_path_external_request_lane_change_module/manager.hpp"

#include "behavior_path_external_request_lane_change_module/scene.hpp"
#include "behavior_path_lane_change_module/interface.hpp"

namespace autoware::behavior_path_planner
{

std::unique_ptr<SceneModuleInterface>
ExternalRequestLaneChangeRightModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_,
    std::make_unique<ExternalRequestLaneChange>(parameters_, direction_));
}

std::unique_ptr<SceneModuleInterface>
ExternalRequestLaneChangeLeftModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_,
    std::make_unique<ExternalRequestLaneChange>(parameters_, direction_));
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::ExternalRequestLaneChangeRightModuleManager,
  ::behavior_path_planner::SceneModuleManagerInterface)
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::ExternalRequestLaneChangeLeftModuleManager,
  ::behavior_path_planner::SceneModuleManagerInterface)
