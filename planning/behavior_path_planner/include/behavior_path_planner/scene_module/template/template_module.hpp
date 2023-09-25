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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__TEMPLATE__TEMPLATE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__TEMPLATE__TEMPLATE_MODULE_HPP_

#include "behavior_path_planner/marker_utils/utils.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utils/template/template_parameters.hpp"
#include "behavior_path_planner/utils/template/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
class TemplateModule : public SceneModuleInterface
{
public:
  TemplateModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<TemplateParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<TemplateParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  bool canTransitSuccessState() override { return false; }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return false; }

  // member
  std::shared_ptr<TemplateParameters> parameters_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__TEMPLATE__TEMPLATE_MODULE_HPP_
