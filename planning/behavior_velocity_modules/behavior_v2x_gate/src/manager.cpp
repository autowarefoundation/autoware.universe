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

#include "manager.hpp"

#include <memory>

namespace behavior_v2x_gate
{

void SceneManager::init(rclcpp::Node * node) { node_ = node; }

void SceneManager::updateSceneModuleInstances(
  const std::shared_ptr<const PlannerData> & data, const PathWithLaneId & path)
{
  (void)data;
  (void)path;
  RCLCPP_INFO_STREAM(node_->get_logger(), "v2x gate update");
}

void SceneManager::plan(PathWithLaneId * path) { (void)path; }

boost::optional<int> SceneManager::getFirstStopPathPointIndex() { return boost::none; }

const char * SceneManager::getModuleName() { return ""; }

}  // namespace behavior_v2x_gate

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_v2x_gate::SceneManager, behavior_velocity_planner::SceneManagerPlugin)
