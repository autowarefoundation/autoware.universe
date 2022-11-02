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

#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

namespace behavior_path_planner
{
std::shared_ptr<AvoidanceDebugMsgArray> SceneModuleVisitor::get_avoidance_debug_msg_array() const
{
  return avoidance_visitor_;
}

std::shared_ptr<LaneChangeDebugMsgArray> SceneModuleVisitor::get_lane_change_debug_msg_array() const
{
  return lane_change_visitor_;
}

void SceneModuleVisitor::set_avoidance_debug_ptr(
  const std::shared_ptr<AvoidanceDebugMsgArray> & debug_msg_ptr)
{
  avoidance_visitor_ = debug_msg_ptr;
}

void SceneModuleVisitor::set_lane_change_debug_ptr(
  const std::shared_ptr<LaneChangeDebugMsgArray> & debug_msg_ptr)
{
  lane_change_visitor_ = debug_msg_ptr;
}

}  // namespace behavior_path_planner
