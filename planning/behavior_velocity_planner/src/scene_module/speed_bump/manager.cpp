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

#include <scene_module/speed_bump/manager.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
std::vector<lanelet::ConstLanelet> getSpeedBumpsOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<lanelet::ConstLanelet> speed_bumps;

  std::set<int64_t> unique_lane_ids;
  for (const auto & p : path.points) {
    unique_lane_ids.insert(p.lane_ids.at(0));  // should we iterate ids? keep as it was.
  }

  for (const auto & lane_id : unique_lane_ids) {
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    constexpr int VEHICLE_GRAPH_ID = 0;
    const auto conflicting_speed_bumps = overall_graphs->conflictingInGraph(ll, VEHICLE_GRAPH_ID);
    for (const auto & speed_bump : conflicting_speed_bumps) {
      for (const auto & attr : speed_bump.attributes()) {
        if (attr.first == "subtype" && attr.second == "speed_bump") {
          speed_bumps.push_back(speed_bump);
        }
      }
    }
  }
  return speed_bumps;
}

std::set<int64_t> getSpeedBumpIdSetOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<int64_t> speed_bump_id_set;
  for (const auto & speed_bump : getSpeedBumpsOnPath(path, lanelet_map, overall_graphs)) {
    speed_bump_id_set.insert(speed_bump.id());
  }
  return speed_bump_id_set;
}

}  // namespace

SpeedBumpModuleManager::SpeedBumpModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());

  // parameters
  auto & pp = planner_param_;
  pp.slow_margin = node.declare_parameter(ns + ".slow_margin", 2.0);
  pp.acceleration_margin = node.declare_parameter(ns + ".acceleration_margin", 0.5);
}

void SpeedBumpModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  for (const auto & speed_bump :
       getSpeedBumpsOnPath(path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr())) {
    const auto module_id = speed_bump.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<SpeedBumpModule>(
        module_id, speed_bump, planner_param_, logger_.get_child("speed_bump_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  const auto speed_bump_id_set =
    getSpeedBumpIdSetOnPath(path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr());
  return [speed_bump_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return scene_module->is_speed_bump_module_expired_ &&
           speed_bump_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
