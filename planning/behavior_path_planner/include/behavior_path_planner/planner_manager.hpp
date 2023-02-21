// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_

#include "behavior_path_planner/scene_module_v2/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module_v2/scene_module_manager_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using tier4_autoware_utils::generateUUID;
using tier4_autoware_utils::StopWatch;
using unique_identifier_msgs::msg::UUID;
using ModuleID = std::pair<std::shared_ptr<SceneModuleManagerInterface>, UUID>;

struct SceneModuleStatus
{
  explicit SceneModuleStatus(const std::string & n) : module_name(n) {}

  std::string module_name;

  bool is_execution_ready{false};
  bool is_waiting_approval{false};

  ModuleStatus status{ModuleStatus::SUCCESS};
};

class PlannerManager
{
public:
  PlannerManager(rclcpp::Node & node, const bool verbose);

  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);

  void registerSceneModuleManager(
    const std::shared_ptr<SceneModuleManagerInterface> & scene_module_manager_ptr)
  {
    RCLCPP_INFO(logger_, "register %s module", scene_module_manager_ptr->getModuleName().c_str());
    scene_manager_ptrs_.push_back(scene_module_manager_ptr);
    processing_time_.emplace(scene_module_manager_ptr->getModuleName(), 0.0);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    std::for_each(
      scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(),
      [&parameters](const auto & m) { m->updateModuleParams(parameters); });
  }

  void reset()
  {
    approved_modules_.clear();
    candidate_module_id_ = boost::none;
    root_lanelet_ = boost::none;
    std::for_each(
      scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(), [](const auto & m) { m->reset(); });
    resetProcessingTime();
  }

  void publishDebugMarker()
  {
    std::for_each(scene_manager_ptrs_.begin(), scene_manager_ptrs_.end(), [](const auto & m) {
      m->publishDebugMarker();
    });
  }

  std::vector<std::shared_ptr<SceneModuleManagerInterface>> getSceneModuleManagers() const
  {
    return scene_manager_ptrs_;
  }

  std::vector<std::shared_ptr<SceneModuleStatus>> getSceneModuleStatus() const
  {
    std::vector<std::shared_ptr<SceneModuleStatus>> ret;

    for (const auto & m : approved_modules_) {
      const auto & manager = m.first;
      const auto & uuid = m.second;
      auto s = std::make_shared<SceneModuleStatus>(manager->getModuleName());
      s->is_waiting_approval = manager->isWaitingApproval(uuid);
      s->status = manager->getCurrentStatus(uuid);
      ret.push_back(s);
    }

    if (!!candidate_module_id_) {
      const auto & manager = candidate_module_id_.get().first;
      const auto & uuid = candidate_module_id_.get().second;
      auto s = std::make_shared<SceneModuleStatus>(manager->getModuleName());
      s->is_waiting_approval = manager->isWaitingApproval(uuid);
      s->status = manager->getCurrentStatus(uuid);
      ret.push_back(s);
    }

    return ret;
  }

  void resetRootLanelet(const std::shared_ptr<PlannerData> & data);

  void print() const;

private:
  BehaviorModuleOutput run(
    const ModuleID & id, const BehaviorModuleOutput & previous_module_output) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return {};
    }

    if (!manager->exist(uuid)) {
      return {};
    }

    return manager->run(uuid, previous_module_output);
  }

  bool isExecutionReady(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->isExecutionReady(uuid);
  }

  bool isWaitingApproval(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->isWaitingApproval(uuid);
  }

  bool isRunning(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return false;
    }

    if (!manager->exist(uuid)) {
      return false;
    }

    return manager->getCurrentStatus(uuid) == ModuleStatus::RUNNING;
  }

  void deleteExpiredModules(const ModuleID & id) const
  {
    const auto & manager = id.first;
    const auto & uuid = id.second;

    if (manager == nullptr) {
      return;
    }

    if (!manager->exist(uuid)) {
      return;
    }

    manager->deleteModules(uuid);
  }

  void addApprovedModule(const ModuleID & id) { approved_modules_.push_back(id); }

  void resetProcessingTime()
  {
    for (auto & t : processing_time_) {
      t.second = 0.0;
    }
  }

  lanelet::ConstLanelet updateRootLanelet(const std::shared_ptr<PlannerData> & data)
  {
    lanelet::ConstLanelet ret{};
    data->route_handler->getClosestLaneletWithinRoute(data->self_odometry->pose.pose, &ret);
    RCLCPP_DEBUG(logger_, "update start lanelet. id:%ld", ret.id());
    return ret;
  }

  BehaviorModuleOutput update(const std::shared_ptr<PlannerData> & data);

  BehaviorModuleOutput getReferencePath(const std::shared_ptr<PlannerData> & data) const;

  boost::optional<ModuleID> getCandidateModuleID(
    const BehaviorModuleOutput & previous_module_output) const;

  boost::optional<ModuleID> selectHighestPriorityModule(
    std::vector<ModuleID> & request_modules) const;

  boost::optional<ModuleID> candidate_module_id_{boost::none};

  boost::optional<lanelet::ConstLanelet> root_lanelet_{boost::none};

  std::vector<std::shared_ptr<SceneModuleManagerInterface>> scene_manager_ptrs_;

  std::vector<ModuleID> approved_modules_;

  rclcpp::Logger logger_;

  rclcpp::Clock clock_;

  mutable StopWatch<std::chrono::milliseconds> stop_watch_;

  mutable std::unordered_map<std::string, double> processing_time_;

  bool verbose_{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
