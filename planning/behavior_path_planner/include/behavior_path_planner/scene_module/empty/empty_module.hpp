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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__EMPTY_MODULE__EMPTY_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__EMPTY_MODULE__EMPTY_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
struct EmptyParameters
{
};
class EmptyModule : public SceneModuleInterface
{
public:
  EmptyModule(const std::string & name, rclcpp::Node & node, const EmptyParameters & parameters);
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

private:
  EmptyParameters parameters_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__EMPTY_MODULE__EMPTY_MODULE_HPP_
