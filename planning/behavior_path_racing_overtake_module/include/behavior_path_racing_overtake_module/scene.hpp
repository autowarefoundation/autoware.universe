// Copyright 2024 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_
#define BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_

#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_racing_overtake_module/data_structs.hpp"
#include "behavior_path_racing_overtake_module/state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace behavior_path_planner
{
class RacingOvertakeModule : public SceneModuleInterface
{
public:
  RacingOvertakeModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<RacingOvertakeParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;

  void updateModuleParams(const std::any & parameters) override
  {
    context_.updateParameters(
      *std::any_cast<std::shared_ptr<RacingOvertakeParameters>>(parameters));
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  racing_overtake::state::Context context_;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_
