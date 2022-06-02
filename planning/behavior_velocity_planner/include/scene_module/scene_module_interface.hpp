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

#ifndef SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
#define SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_

#include "behavior_velocity_planner/planner_data.hpp"

#include <utilization/util.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>

#include <memory>
#include <set>
#include <string>

// Debug
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace behavior_velocity_planner
{
class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : module_id_(module_id), logger_(logger), clock_(clock)
  {
  }
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason) = 0;
  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;
  virtual visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() = 0;

  int64_t getModuleId() const { return module_id_; }
  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> getInfrastructureCommand()
  {
    return infrastructure_command_;
  }

  void setInfrastructureCommand(
    const boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
  {
    infrastructure_command_ = command;
  }

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

protected:
  const int64_t module_id_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const PlannerData> planner_data_;
  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command_;
  boost::optional<int> first_stop_path_point_index_;
};

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(rclcpp::Node & node, const char * module_name)
  : clock_(node.get_clock()), logger_(node.get_logger())
  {
    const auto ns = std::string("~/debug/") + module_name;
    pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 20);
    pub_virtual_wall_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
      std::string("~/virtual_wall/") + module_name, 20);
    pub_stop_reason_ =
      node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 20);
    pub_infrastructure_commands_ =
      node.create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
        "~/output/infrastructure_commands", 20);
  }

  virtual ~SceneModuleManagerInterface() = default;

  virtual const char * getModuleName() = 0;

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    planner_data_ = planner_data;

    launchNewModules(path);
    deleteExpiredModules(path);
  }

  virtual void modifyPathVelocity(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
  {
    visualization_msgs::msg::MarkerArray debug_marker_array;
    visualization_msgs::msg::MarkerArray virtual_wall_marker_array;
    tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
    stop_reason_array.header.frame_id = "map";
    stop_reason_array.header.stamp = clock_->now();

    tier4_v2x_msgs::msg::InfrastructureCommandArray infrastructure_command_array;
    infrastructure_command_array.stamp = clock_->now();

    first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
    for (const auto & scene_module : scene_modules_) {
      tier4_planning_msgs::msg::StopReason stop_reason;
      scene_module->setPlannerData(planner_data_);
      scene_module->modifyPathVelocity(path, &stop_reason);

      if (stop_reason.reason != "") {
        stop_reason_array.stop_reasons.emplace_back(stop_reason);
      }

      if (const auto command = scene_module->getInfrastructureCommand()) {
        infrastructure_command_array.commands.push_back(*command);
      }

      if (scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
        first_stop_path_point_index_ = scene_module->getFirstStopPathPointIndex();
      }

      for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
        debug_marker_array.markers.push_back(marker);
      }

      for (const auto & marker : scene_module->createVirtualWallMarkerArray().markers) {
        virtual_wall_marker_array.markers.push_back(marker);
      }
    }

    if (!stop_reason_array.stop_reasons.empty()) {
      pub_stop_reason_->publish(stop_reason_array);
    }
    pub_infrastructure_commands_->publish(infrastructure_command_array);
    pub_debug_->publish(debug_marker_array);
    pub_virtual_wall_->publish(virtual_wall_marker_array);
  }

protected:
  template <class T>
  std::unordered_map<typename std::shared_ptr<const T>, lanelet::ConstLanelet> getRegElemMapOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map)
  {
    std::unordered_map<typename std::shared_ptr<const T>, lanelet::ConstLanelet>
      reg_elem_map_on_path;
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

      for (const auto & reg_elem : ll.regulatoryElementsAs<const T>()) {
        reg_elem_map_on_path.insert(std::make_pair(reg_elem, ll));
      }
    }

    return reg_elem_map_on_path;
  }

  template <class T>
  std::set<int64_t> getRegElemIdSetOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map)
  {
    std::set<int64_t> reg_elem_id_set;
    for (const auto & m : getRegElemMapOnPath<const T>(path, lanelet_map)) {
      reg_elem_id_set.insert(m.first->id());
    }
    return reg_elem_id_set;
  }

  template <class T>
  std::set<int64_t> getLaneletIdSetOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map)
  {
    std::set<int64_t> id_set;
    for (const auto & m : getRegElemMapOnPath<const T>(path, lanelet_map)) {
      id_set.insert(m.second.id());
    }
    return id_set;
  }

  virtual std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map)
  {
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

    std::vector<lanelet::ConstLanelet> lanelets;
    for (const auto lane_id : unique_lane_ids) {
      lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
    }

    return lanelets;
  }

  virtual std::set<int64_t> getLaneIdSetOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::LaneletMapPtr lanelet_map)
  {
    std::set<int64_t> lane_id_set;
    for (const auto & lane : getLaneletsOnPath(path, lanelet_map)) {
      lane_id_set.insert(lane.id());
    }

    return lane_id_set;
  }

  virtual void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
  getModuleExpiredFunction(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  void deleteExpiredModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    const auto isModuleExpired = getModuleExpiredFunction(path);

    // Copy container to avoid iterator corruption
    // due to scene_modules_.erase() in unregisterModule()
    const auto copied_scene_modules = scene_modules_;

    for (const auto & scene_module : copied_scene_modules) {
      if (isModuleExpired(scene_module)) {
        unregisterModule(scene_module);
      }
    }
  }

  bool isModuleRegistered(const int64_t module_id)
  {
    return registered_module_id_set_.count(module_id) != 0;
  }

  void registerModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "register task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.emplace(scene_module->getModuleId());
    scene_modules_.insert(scene_module);
  }

  void unregisterModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "unregister task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.erase(scene_module->getModuleId());
    scene_modules_.erase(scene_module);
  }

  std::set<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  std::shared_ptr<const PlannerData> planner_data_;

  boost::optional<int> first_stop_path_point_index_;
  rclcpp::Clock::SharedPtr clock_;
  // Debug
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr pub_stop_reason_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
