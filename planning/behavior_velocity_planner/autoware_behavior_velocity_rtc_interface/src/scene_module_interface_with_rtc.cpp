// Copyright 2023 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

SceneModuleInterfaceWithRTC::SceneModuleInterfaceWithRTC(
  const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  activated_(false),
  safe_(false),
  distance_(std::numeric_limits<double>::lowest())
{
}

SceneModuleManagerInterfaceWithRTC::SceneModuleManagerInterfaceWithRTC(
  rclcpp::Node & node, const char * module_name, const bool enable_rtc)
: SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>(node, module_name),
  rtc_interface_(&node, module_name, enable_rtc),
  objects_of_interest_marker_interface_(&node, module_name)
{
}

void SceneModuleManagerInterfaceWithRTC::plan(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  setActivation();
  modify_path_velocity(path);
  sendRTC(path->header.stamp);
  publishObjectsOfInterestMarker();
}

void SceneModuleManagerInterfaceWithRTC::sendRTC(const Time & stamp)
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->get_module_id());
    const auto state = !scene_module->isActivated() && scene_module->isSafe()
                         ? State::WAITING_FOR_EXECUTION
                         : State::RUNNING;
    updateRTCStatus(uuid, scene_module->isSafe(), state, scene_module->getDistance(), stamp);
  }
  publishRTCStatus(stamp);
}

void SceneModuleManagerInterfaceWithRTC::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->get_module_id());
    scene_module->setActivation(rtc_interface_.isActivated(uuid));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(uuid));
  }
}

UUID SceneModuleManagerInterfaceWithRTC::getUUID(const int64_t & module_id) const
{
  if (map_uuid_.count(module_id) == 0) {
    const UUID uuid;
    return uuid;
  }
  return map_uuid_.at(module_id);
}

void SceneModuleManagerInterfaceWithRTC::generateUUID(const int64_t & module_id)
{
  map_uuid_.insert({module_id, autoware::universe_utils::generateUUID()});
}

void SceneModuleManagerInterfaceWithRTC::removeUUID(const int64_t & module_id)
{
  const auto result = map_uuid_.erase(module_id);
  if (result == 0) {
    RCLCPP_WARN_STREAM(logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
  }
}

void SceneModuleManagerInterfaceWithRTC::publishObjectsOfInterestMarker()
{
  for (const auto & scene_module : scene_modules_) {
    const auto objects = scene_module->get_objects_of_interest_data();
    for (const auto & obj : objects) {
      objects_of_interest_marker_interface_.insertObjectData(obj.pose, obj.shape, obj.color);
    }
    scene_module->clear_objects_of_interest_data();
  }

  objects_of_interest_marker_interface_.publishMarkerArray();
}

void SceneModuleManagerInterfaceWithRTC::delete_expired_modules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = get_module_expired_function(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      const UUID uuid = getUUID((*itr)->get_module_id());
      updateRTCStatus(
        uuid, (*itr)->isSafe(), State::SUCCEEDED, std::numeric_limits<double>::lowest(),
        clock_->now());
      removeUUID((*itr)->get_module_id());
      registered_module_id_set_.erase((*itr)->get_module_id());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

template size_t SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::find_ego_segment_index(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points) const;
template void
SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::update_scene_module_instances(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path);
template void SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::modify_path_velocity(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path);
template void SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::register_module(
  const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module);
}  // namespace autoware::behavior_velocity_planner
