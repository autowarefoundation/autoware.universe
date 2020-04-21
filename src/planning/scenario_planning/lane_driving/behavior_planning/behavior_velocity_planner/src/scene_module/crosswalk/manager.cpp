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
#include <scene_module/crosswalk/manager.h>

namespace
{
std::vector<lanelet::ConstLanelet> getCrosswalksOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<lanelet::ConstLanelet> crosswalks;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    const auto conflicting_crosswalks = overall_graphs->conflictingInGraph(ll, 1);
    for (const auto & crosswalk : conflicting_crosswalks) {
      crosswalks.push_back(crosswalk);
    }
  }

  return crosswalks;
}

std::set<int64_t> getCrosswalkIdSetOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<int64_t> crosswalk_id_set;

  for (const auto & crosswalk : getCrosswalksOnPath(path, lanelet_map, overall_graphs)) {
    crosswalk_id_set.insert(crosswalk.id());
  }

  return crosswalk_id_set;
}

}  // namespace

void CrosswalkModuleManager::launchNewModules(const autoware_planning_msgs::PathWithLaneId & path)
{
  for (const auto & crosswalk :
       getCrosswalksOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs)) {
    const auto module_id = crosswalk.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<CrosswalkModule>(module_id, crosswalk));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CrosswalkModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto crosswalk_id_set =
    getCrosswalkIdSetOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs);

  return [crosswalk_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crosswalk_id_set.count(scene_module->getModuleId()) == 0;
  };
}
