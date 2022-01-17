// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/empty/empty_module.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <memory>
#include <string>

namespace behavior_path_planner
{
EmptyModule::EmptyModule(
  const std::string & name, rclcpp::Node & node, const EmptyParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  approval_handler_.clearWaitApproval();
  current_state_ = BT::NodeStatus::IDLE;
}
bool EmptyModule::isExecutionRequested() const { return false; }
bool EmptyModule::isExecutionReady() const { return false; }
BT::NodeStatus EmptyModule::updateState() { return BT::NodeStatus::SUCCESS; }
BehaviorModuleOutput EmptyModule::plan() { return BehaviorModuleOutput(); }
PathWithLaneId EmptyModule::planCandidate() const
{
  PathWithLaneId path;
  return path;
}
BehaviorModuleOutput EmptyModule::planWaitingApproval() { return BehaviorModuleOutput(); }
void EmptyModule::onEntry() { RCLCPP_ERROR(getLogger(), "Dont Entry This"); }
void EmptyModule::onExit() {}

}  // namespace behavior_path_planner
