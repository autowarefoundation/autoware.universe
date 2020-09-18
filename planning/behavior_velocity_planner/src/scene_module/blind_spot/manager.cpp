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
#include <scene_module/blind_spot/manager.h>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include "utilization/boost_geometry_helper.h"
#include "utilization/util.h"

namespace
{
std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::ConstLanelet> lanelets;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(const autoware_planning_msgs::PathWithLaneId & path)
{
  std::set<int64_t> lane_id_set;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    lane_id_set.insert(lane_id);
  }

  return lane_id_set;
}

}  // namespace

BlindSpotModuleManager::BlindSpotModuleManager() : SceneModuleManagerInterface(getModuleName())
{
  ros::NodeHandle pnh("~");
  const std::string ns(getModuleName());
  auto & p = planner_param_;
  pnh.param(ns + "/stop_line_margin", p.stop_line_margin, 1.0);
  pnh.param(ns + "/backward_length", p.backward_length, 15.0);
  pnh.param(ns + "/max_future_movement_time", p.max_future_movement_time, 10.0);
}

void BlindSpotModuleManager::launchNewModules(const autoware_planning_msgs::PathWithLaneId & path)
{
  for (const auto & ll : getLaneletsOnPath(path, planner_data_->lanelet_map)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is turning lane?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (turn_direction != "left" && turn_direction != "right") {
      continue;
    }

    registerModule(
      std::make_shared<BlindSpotModule>(module_id, lane_id, planner_data_, planner_param_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
BlindSpotModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto lane_id_set = getLaneIdSetOnPath(path);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}
