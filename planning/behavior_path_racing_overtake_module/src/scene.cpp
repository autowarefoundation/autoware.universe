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

#include "behavior_path_racing_overtake_module/scene.hpp"

#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <memory>
#include <string>

namespace behavior_path_planner
{

RacingOvertakeModule::RacingOvertakeModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<RacingOvertakeParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},
  context_(*parameters)
{
}

bool RacingOvertakeModule::isExecutionRequested() const
{
  return context_.getState().getName() != "ModuleNotLaunched";
}

bool RacingOvertakeModule::isExecutionReady() const
{
  return true;
}

bool RacingOvertakeModule::canTransitSuccessState()
{
  return context_.getState().getName() == "ModuleNotLaunched";
}

void RacingOvertakeModule::updateData()
{
  context_.updateState(planner_data_);
  // std::cerr << "Curren State: " << context_.getState().getName() << std::endl;
  return;
}

BehaviorModuleOutput RacingOvertakeModule::plan()
{
  BehaviorModuleOutput output = getPreviousModuleOutput();
  context_.getState().getPath(planner_data_, &output);
  return output;
}

CandidateOutput RacingOvertakeModule::planCandidate() const
{
  return CandidateOutput{};
}

}  // namespace behavior_path_planner
