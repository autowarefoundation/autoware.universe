
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE_V2__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE_V2__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include "behavior_path_planner/scene_module_v2/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using tier4_autoware_utils::toHexString;
using unique_identifier_msgs::msg::UUID;

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(
    rclcpp::Node * node, const std::string & name, const size_t max_module_num,
    const size_t priority, const bool enable_simultaneous_execution)
  : node_(node),
    clock_(*node->get_clock()),
    logger_(node->get_logger().get_child(name)),
    name_(name),
    max_module_num_(max_module_num),
    priority_(priority),
    enable_simultaneous_execution_(enable_simultaneous_execution)
  {
    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name, 20);
  }

  virtual ~SceneModuleManagerInterface() = default;

  bool isExecutionRequested(const BehaviorModuleOutput & previous_module_output)
  {
    const auto m = getIdlingModule();

    m->setData(planner_data_);
    m->setPreviousModuleOutput(previous_module_output);
    m->updateData();

    return m->isExecutionRequested();
  }

  bool isExecutionReady(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return false;
    }

    return m->isExecutionReady();
  }

  bool isLockedNewModuleLaunch(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return false;
    }

    return m->isLockedNewModuleLaunch();
  }

  bool isWaitingApproval(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return false;
    }

    if (!m->isActivated()) {
      return m->isWaitingApproval();
    }

    return false;
  }

  ModuleStatus getCurrentStatus(const UUID & uuid) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return ModuleStatus::IDLE;
    }

    return m->getCurrentStatus();
  }

  BehaviorModuleOutput run(
    const UUID & uuid, const BehaviorModuleOutput & previous_module_output) const
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return {};
    }

    m->setData(planner_data_);
    m->setPreviousModuleOutput(previous_module_output);
    m->updateData();

    m->lockRTCCommand();
    const auto result = m->run();
    m->unlockRTCCommand();

    m->updateState();

    m->publishRTCStatus();

    return result;
  }

  void registerNewModule(const BehaviorModuleOutput & previous_module_output, const UUID & uuid)
  {
    const auto m = createNewSceneModuleInstance();

    m->setData(planner_data_);
    m->setPreviousModuleOutput(previous_module_output);
    m->onEntry();

    registered_modules_.insert(std::make_pair(toHexString(uuid), m));
  }

  void deleteModules(const UUID & uuid)
  {
    const auto m = findSceneModule(uuid);

    if (m == nullptr) {
      return;
    }

    m->onExit();
    m->publishRTCStatus();

    registered_modules_.erase(toHexString(uuid));

    pub_debug_marker_->publish(MarkerArray{});
  }

  void publishDebugMarker()
  {
    using tier4_autoware_utils::appendMarkerArray;

    MarkerArray markers{};

    for (const auto & m : registered_modules_) {
      const uint32_t marker_id = std::stoul(m.first.substr(0, sizeof(uint32_t) / 4), nullptr, 16);
      for (auto & marker : m.second->getDebugMarkers().markers) {
        marker.id += marker_id;
        markers.markers.push_back(marker);
      }
    }

    if (registered_modules_.empty() && idling_module_ != nullptr) {
      appendMarkerArray(idling_module_->getDebugMarkers(), &markers);
    }

    pub_debug_marker_->publish(markers);
  }

  bool exist(const UUID & uuid) const
  {
    if (registered_modules_.count(toHexString(uuid)) == 0) {
      RCLCPP_DEBUG(
        logger_, "uuid %s is not found in %s module.", toHexString(uuid).c_str(), name_.c_str());
      return false;
    }

    return true;
  }

  bool canLaunchNewModule() const { return registered_modules_.size() < max_module_num_; }

  bool isSimultaneousExecutable() const { return enable_simultaneous_execution_; }

  void setData(const std::shared_ptr<PlannerData> & planner_data) { planner_data_ = planner_data; }

  void reset()
  {
    std::for_each(registered_modules_.begin(), registered_modules_.end(), [](const auto & m) {
      m.second->onExit();
      m.second->publishRTCStatus();
    });
    registered_modules_.clear();

    idling_module_->onExit();
    idling_module_->publishRTCStatus();
    idling_module_.reset();

    pub_debug_marker_->publish(MarkerArray{});
  }

  size_t getPriority() const { return priority_; }

  std::string getModuleName() const { return name_; }

  std::vector<std::shared_ptr<SceneModuleInterface>> getSceneModules()
  {
    std::vector<std::shared_ptr<SceneModuleInterface>> modules;
    for (const auto & m : registered_modules_) {
      modules.push_back(m.second);
    }

    return modules;
  }

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

protected:
  virtual void getModuleParams(rclcpp::Node * node) = 0;

  virtual std::shared_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_;

  rclcpp::Clock clock_;

  rclcpp::Logger logger_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::shared_ptr<SceneModuleInterface> idling_module_;

  std::unordered_map<std::string, std::shared_ptr<SceneModuleInterface>> registered_modules_;

private:
  std::shared_ptr<SceneModuleInterface> findSceneModule(const UUID & uuid) const
  {
    const auto itr = registered_modules_.find(toHexString(uuid));
    if (itr == registered_modules_.end()) {
      return nullptr;
    }

    return registered_modules_.at(toHexString(uuid));
  }

  std::shared_ptr<SceneModuleInterface> getIdlingModule()
  {
    if (idling_module_ != nullptr) {
      return idling_module_;
    }

    idling_module_ = createNewSceneModuleInstance();
    return idling_module_;
  }

  size_t max_module_num_;

  size_t priority_;

  bool enable_simultaneous_execution_{false};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE_V2__SCENE_MODULE_MANAGER_INTERFACE_HPP_
