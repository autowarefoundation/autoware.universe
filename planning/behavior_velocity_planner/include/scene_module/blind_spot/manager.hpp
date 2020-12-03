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
#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/path_with_lane_id.hpp>

#include <scene_module/blind_spot/scene.hpp>
#include <scene_module/scene_module_interface.hpp>

class BlindSpotModuleManager : public SceneModuleManagerInterface
{
public:
  BlindSpotModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "blind_spot"; }

private:
  BlindSpotModule::PlannerParam planner_param_;

  void launchNewModules(const autoware_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_planning_msgs::msg::PathWithLaneId & path) override;
};
