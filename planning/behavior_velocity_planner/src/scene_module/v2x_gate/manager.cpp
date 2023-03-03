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

#include <scene_module/v2x_gate/manager.hpp>

namespace behavior_velocity_planner
{

V2xGateManager::V2xGateManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
}

void V2xGateManager::launchNewModules(const PathWithLaneId & path)
{
  (void)path;
  RCLCPP_INFO_STREAM(logger_, "[v2x gate] launch_new_modules");
}

V2xGateManager::ExpiredFunction V2xGateManager::getModuleExpiredFunction(
  const PathWithLaneId & path)
{
  (void)path;
  return [](const auto &) { return true; };
}

}  // namespace behavior_velocity_planner
