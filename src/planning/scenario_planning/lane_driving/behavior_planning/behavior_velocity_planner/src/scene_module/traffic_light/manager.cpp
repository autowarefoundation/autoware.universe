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
std::vector<lanelet::TrafficLightConstPtr> getTrafficLightRegElemsOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::TrafficLightConstPtr> traffic_light_reg_elems;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    const auto tls = ll.regulatoryElementsAs<const lanelet::TrafficLight>();
    for (const auto & tl : tls) {
      traffic_light_reg_elems.push_back(tl);
    }
  }

  return traffic_light_reg_elems;
}

std::set<int64_t> getStopLineIdSetOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> stop_line_id_set;

  for (const auto & traffic_light_reg_elem : getTrafficLightRegElemsOnPath(path, lanelet_map)) {
    const auto stop_line = traffic_light_reg_elem->stopLine();
    if (stop_line) {
      stop_line_id_set.insert(stop_line->id());
    }
  }

  return stop_line_id_set;
}

}  // namespace

void TrafficLightModuleManager::launchNewModules(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem :
       getTrafficLightRegElemsOnPath(path, planner_data_->lanelet_map)) {
    const auto stop_line = traffic_light_reg_elem->stopLine();

    if (!stop_line) {
      ROS_FATAL(
        "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem->id());
      continue;
    }

    // Use stop_line_id as module_id because some traffic lights may indicate the same stop line
    const auto module_id = stop_line->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<TrafficLightModule>(module_id, *traffic_light_reg_elem));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto stop_line_id_set = getStopLineIdSetOnPath(path, planner_data_->lanelet_map);

  return [stop_line_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return stop_line_id_set.count(scene_module->getModuleId()) == 0;
  };
}
