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
#include "scene_module/detection_area/manager.hpp"

#include "tf2/utils.h"

namespace
{
std::unordered_map<int64_t, lanelet::DetectionAreaConstPtr> getDetectionAreaRegElemsOnPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::unordered_map<int64_t, lanelet::DetectionAreaConstPtr> detection_area_reg_elems;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);
    const auto detection_areas = ll.regulatoryElementsAs<const lanelet::autoware::DetectionArea>();
    for (const auto & regelem : detection_areas) {
      detection_area_reg_elems.insert(std::make_pair(ll.id(), regelem));
    }
  }

  return detection_area_reg_elems;
}

std::set<int64_t> getLaneletIdSetOnPath(
  const autoware_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> lanelet_id_set;
  for (const auto & regelem : getDetectionAreaRegElemsOnPath(path, lanelet_map)) {
    lanelet_id_set.insert(regelem.first);
  }
  return lanelet_id_set;
}
}  // namespace

DetectionAreaModuleManager::DetectionAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = node.declare_parameter(ns + "/stop_margin", 0.0);
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & detection_area_reg_elem :
    getDetectionAreaRegElemsOnPath(path, planner_data_->lanelet_map))
  {
    // Use lanelet_id to unregister module when the route is changed
    const auto module_id = detection_area_reg_elem.first;
    if (!isModuleRegistered(module_id)) {
      registerModule(
        std::make_shared<DetectionAreaModule>(
          module_id, *(detection_area_reg_elem.second), planner_param_,
          logger_.get_child("detection_area_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DetectionAreaModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelet_id_set = getLaneletIdSetOnPath(path, planner_data_->lanelet_map);

  return [lanelet_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
           return lanelet_id_set.count(scene_module->getModuleId()) == 0;
         };
}
