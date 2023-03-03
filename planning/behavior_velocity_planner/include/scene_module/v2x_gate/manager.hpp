// Copyright 2023 The Autoware Contributors
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

#ifndef SCENE_MODULE__V2X_GATE__MANAGER_HPP_
#define SCENE_MODULE__V2X_GATE__MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <scene_module/v2x_gate/scene.hpp>

#include <memory>

namespace behavior_velocity_planner
{

class V2xGateManager : public SceneModuleManagerInterface
{
public:
  explicit V2xGateManager(rclcpp::Node & node);
  const char * getModuleName() override { return "v2x_gate"; }

private:
  using ExpiredFunction = std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>;
  V2xGateModule::PlannerParam planner_param_;

  void launchNewModules(const PathWithLaneId & path) override;
  ExpiredFunction getModuleExpiredFunction(const PathWithLaneId & path) override;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__V2X_GATE__MANAGER_HPP_
