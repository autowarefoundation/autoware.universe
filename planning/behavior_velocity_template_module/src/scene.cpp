// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "motion_utils/motion_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

namespace behavior_velocity_planner
{
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;

using geometry_msgs::msg::Point32;

TemplateModule::TemplateModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  lane_id_(lane_id),
  speed_bump_reg_elem_(std::move(speed_bump_reg_elem)),
  planner_param_(planner_param)
{
}

bool TemplateModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  if (path->points.empty()) {
    return false;
  }

  std::cout << "----------------------------\n";
  int idx = 0;
  for (const auto & point : path->points) {
    auto longitudinal_velocity_ = point.point.longitudinal_velocity_mps;
    std::cout << "Point ID:" << idx++ << "\n";
    std::cout << "Longitudinal Velocity " << longitudinal_velocity_ << "\n";
  }
  std::cout << "----------------------------\n";

  return false;
}

}  // namespace behavior_velocity_planner
