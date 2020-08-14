/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <scene_module/traffic_light/manager.h>

#include <tf2/utils.h>

namespace
{
std::unordered_map<lanelet::TrafficLightConstPtr, lanelet::ConstLanelet>
getTrafficLightRegElemsOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::unordered_map<lanelet::TrafficLightConstPtr, lanelet::ConstLanelet> traffic_light_reg_elems;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    const auto tls = ll.regulatoryElementsAs<const lanelet::TrafficLight>();
    for (const auto & tl : tls) {
      traffic_light_reg_elems.insert(std::make_pair(tl, ll));
    }
  }

  return traffic_light_reg_elems;
}

std::set<int64_t> getLaneletIdSetOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> lanelet_id_set;
  for (const auto & traffic_light_reg_elem : getTrafficLightRegElemsOnPath(path, lanelet_map)) {
    lanelet_id_set.insert(traffic_light_reg_elem.second.id());
  }
  return lanelet_id_set;
}

}  // namespace

TrafficLightModuleManager::TrafficLightModuleManager()
: SceneModuleManagerInterface(getModuleName())
{
  ros::NodeHandle pnh("~");
  const std::string ns(getModuleName());
  auto & p = planner_param_;
  pnh.param(ns + "/stop_margin", p.stop_margin, 0.0);
  pnh.param(ns + "/tl_state_timeout", p.tl_state_timeout, 1.0);
}

void TrafficLightModuleManager::launchNewModules(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem :
       getTrafficLightRegElemsOnPath(path, planner_data_->lanelet_map)) {
    const auto stop_line = traffic_light_reg_elem.first->stopLine();

    if (!stop_line) {
      ROS_FATAL(
        "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto module_id = traffic_light_reg_elem.second.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<TrafficLightModule>(
        module_id, *(traffic_light_reg_elem.first), traffic_light_reg_elem.second, planner_param_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto lanelet_id_set = getLaneletIdSetOnPath(path, planner_data_->lanelet_map);

  return [lanelet_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lanelet_id_set.count(scene_module->getModuleId()) == 0;
  };
}
