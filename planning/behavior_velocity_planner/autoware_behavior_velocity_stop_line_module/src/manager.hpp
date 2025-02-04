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

#include "autoware/behavior_velocity_planner_common/plugin_wrapper.hpp"
#include "autoware/behavior_velocity_planner_common/scene_module_interface.hpp"
#include "scene.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <functional>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using StopLineWithLaneId = std::pair<lanelet::ConstLineString3d, int64_t>;

class StopLineModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit StopLineModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "stop_line"; }

private:
  StopLineModule::PlannerParam planner_param_;

  std::vector<StopLineWithLaneId> getStopLinesWithLaneIdOnPath(
    const tier4_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map);

  std::set<int64_t> getStopLineIdSetOnPath(
    const tier4_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map);

  void launchNewModules(const tier4_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const tier4_planning_msgs::msg::PathWithLaneId & path) override;
};

class StopLineModulePlugin : public PluginWrapper<StopLineModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
