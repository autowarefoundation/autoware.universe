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

#ifndef SCENE_MODULE__TRAFFIC_LIGHT__MANAGER_HPP_
#define SCENE_MODULE__TRAFFIC_LIGHT__MANAGER_HPP_

#include <functional>
#include <memory>

#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "rclcpp/rclcpp.hpp"

#include "scene_module/scene_module_interface.hpp"
#include "scene_module/traffic_light/scene.hpp"

class TrafficLightModuleManager : public SceneModuleManagerInterface
{
public:
  explicit TrafficLightModuleManager(rclcpp::Node & node);

  const char * getModuleName() override {return "traffic_light";}

  void modifyPathVelocity(autoware_planning_msgs::msg::PathWithLaneId * path) override;

private:
  TrafficLightModule::PlannerParam planner_param_;
  void launchNewModules(const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  // Debug
  rclcpp::Publisher<autoware_perception_msgs::msg::LookingTrafficLightState>::SharedPtr
    pub_tl_state_;
};
#endif  // SCENE_MODULE__TRAFFIC_LIGHT__MANAGER_HPP_
