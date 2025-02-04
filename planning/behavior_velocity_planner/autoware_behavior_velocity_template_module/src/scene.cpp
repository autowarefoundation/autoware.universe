// Copyright 2023 TIER IV, Inc.
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

#include "scene.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
// #include "autoware/universe_utils/autoware_universe_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner
{

TemplateModule::TemplateModule(
  const int64_t module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface)
{
}

visualization_msgs::msg::MarkerArray TemplateModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray ma;
  return ma;
};

autoware::motion_utils::VirtualWalls TemplateModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls vw;
  return vw;
}

bool TemplateModule::modifyPathVelocity([[maybe_unused]] PathWithLaneId * path)
{
  RCLCPP_INFO_ONCE(logger_, "Template Module is executing!");
  return false;
}

}  // namespace autoware::behavior_velocity_planner
