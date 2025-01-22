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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene_crosswalk.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace autoware::behavior_velocity_planner
{

using tier4_planning_msgs::msg::PathWithLaneId;

class CrosswalkModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit CrosswalkModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "crosswalk"; }

private:
  CrosswalkModule::PlannerParam crosswalk_planner_param_{};

  void launchNewModules(const PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const PathWithLaneId & path) override;
};

class CrosswalkModulePlugin : public PluginWrapper<CrosswalkModuleManager>
{
};
}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
