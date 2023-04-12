// Copyright 2023 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__INTERFACE_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/lane_change/base_class.hpp"
#include "behavior_path_planner/scene_module/lane_change/normal.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/util/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class LaneChangeInterface : public SceneModuleInterface
{
public:
  LaneChangeInterface(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<LaneChangeParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map,
    std::unique_ptr<LaneChangeBase> && module_type);

  void processOnEntry() override;
  void processOnExit() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;

  ModuleStatus updateState() override;

  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;

  std::shared_ptr<LaneChangeDebugMsgArray> get_debug_msg_array() const;

  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;
  void updateModuleParams(const std::shared_ptr<LaneChangeParameters> & parameters);

  void setData(const std::shared_ptr<const PlannerData> & data) override;

private:
  std::shared_ptr<LaneChangeParameters> parameters_;
  std::unique_ptr<LaneChangeBase> module_type_;

  void resetPathIfAbort();

protected:
  void setObjectDebugVisualization() const;
  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  mutable LaneChangeDebugMsgArray lane_change_debug_msg_array_;

  std::unique_ptr<PathWithLaneId> prev_approved_path_;
  void clearAbortApproval() { is_abort_path_approved_ = false; }
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__INTERFACE_HPP_
