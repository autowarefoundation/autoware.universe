// Copyright 2024 TIER IV, Inc.
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

#include "behavior_path_racing_overtake_module/manager.hpp"

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

RacingOvertakeModuleManager::RacingOvertakeModuleManager()
: SceneModuleManagerInterface{"racing_overtake"}
{
}

std::unique_ptr<SceneModuleInterface> RacingOvertakeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<RacingOvertakeModule>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_);
}

void RacingOvertakeModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});
  RacingOvertakeParameters p{};

  const std::string ns = "racing_overtake.";
  p.too_close_to_overtake_distance =
    node->declare_parameter<double>(ns + "too_close_to_overtake_distance");
  p.start_overtake_distance = node->declare_parameter<double>(ns + "start_overtake_distance");
  p.prepare_overtake_distance = node->declare_parameter<double>(ns + "prepare_overtake_distance");
  p.back_to_center_start_distance =
    node->declare_parameter<double>(ns + "back_to_center_start_distance");
  p.back_to_center_end_distance =
    node->declare_parameter<double>(ns + "back_to_center_end_distance");
  p.ego_course_width = node->declare_parameter<double>(ns + "ego_course_width");

  parameters_ = std::make_shared<RacingOvertakeParameters>(p);
}

void RacingOvertakeModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  const std::string ns = "racing_overtake.";

  updateParam(parameters, ns + "too_close_to_overtake_distance", p->too_close_to_overtake_distance);
  updateParam(parameters, ns + "start_overtake_distance", p->start_overtake_distance);
  updateParam(parameters, ns + "prepare_overtake_distance", p->prepare_overtake_distance);
  updateParam(parameters, ns + "back_to_center_start_distance", p->back_to_center_start_distance);
  updateParam(parameters, ns + "back_to_center_end_distance", p->back_to_center_end_distance);
  updateParam(parameters, ns + "ego_course_width", p->ego_course_width);
}

}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::RacingOvertakeModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
