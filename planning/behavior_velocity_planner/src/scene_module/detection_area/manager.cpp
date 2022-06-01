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

#include <lanelet2_extension/utility/query.hpp>
#include <scene_module/detection_area/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
DetectionAreaModuleManager::DetectionAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = node.declare_parameter(ns + ".stop_margin", 0.0);
  planner_param_.use_dead_line = node.declare_parameter(ns + ".use_dead_line", false);
  planner_param_.dead_line_margin = node.declare_parameter(ns + ".dead_line_margin", 5.0);
  planner_param_.use_pass_judge_line = node.declare_parameter(ns + ".use_pass_judge_line", false);
  planner_param_.state_clear_time = node.declare_parameter(ns + ".state_clear_time", 2.0);
}

std::vector<lanelet::DetectionAreaConstPtr>
DetectionAreaModuleManager::getDetectionAreaRegElemsOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::DetectionAreaConstPtr> detection_area_reg_elems;
  std::set<int64_t> unique_lane_ids;
  auto nearest_segment_idx = tier4_autoware_utils::findNearestSegmentIndex(
    path.points, planner_data_->current_pose.pose, std::numeric_limits<double>::max(), M_PI_2);

  // Add current lane id
  lanelet::ConstLanelets current_lanes;
  if (
    lanelet::utils::query::getCurrentLanelets(
      lanelet::utils::query::laneletLayer(lanelet_map), planner_data_->current_pose.pose,
      &current_lanes) &&
    nearest_segment_idx) {
    for (const auto & ll : current_lanes) {
      if (
        ll.id() == path.points.at(*nearest_segment_idx).lane_ids.at(0) ||
        ll.id() == path.points.at(*nearest_segment_idx + 1).lane_ids.at(0)) {
        unique_lane_ids.insert(ll.id());
      }
    }
  }

  // Add forward path lane_id
  const size_t start_idx = *nearest_segment_idx ? *nearest_segment_idx + 1 : 0;
  for (size_t i = start_idx; i < path.points.size(); i++) {
    unique_lane_ids.insert(
      path.points.at(i).lane_ids.at(0));  // should we iterate ids? keep as it was.
  }

  for (const auto lane_id : unique_lane_ids) {
    const auto ll = lanelet_map->laneletLayer.get(lane_id);
    const auto detection_areas = ll.regulatoryElementsAs<const lanelet::autoware::DetectionArea>();
    for (const auto & detection_area : detection_areas) {
      detection_area_reg_elems.push_back(detection_area);
    }
  }

  return detection_area_reg_elems;
}

std::set<int64_t> DetectionAreaModuleManager::getDetectionAreaIdSetOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> detection_area_id_set;
  for (const auto & detection_area : getDetectionAreaRegElemsOnPath(path, lanelet_map)) {
    detection_area_id_set.insert(detection_area->id());
  }
  return detection_area_id_set;
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & detection_area :
       getDetectionAreaRegElemsOnPath(path, planner_data_->route_handler_->getLaneletMapPtr())) {
    // Use lanelet_id to unregister module when the route is changed
    const auto module_id = detection_area->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<DetectionAreaModule>(
        module_id, *detection_area, planner_param_, logger_.get_child("detection_area_module"),
        clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DetectionAreaModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto detection_area_id_set =
    getDetectionAreaIdSetOnPath(path, planner_data_->route_handler_->getLaneletMapPtr());

  return [detection_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return detection_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
