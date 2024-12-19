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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_VELOCITY_MODULE_INTERFACE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_VELOCITY_MODULE_INTERFACE_HPP_

#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_stop_module.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"

#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

#include <memory>

namespace autoware::motion_planning
{
class ObstacleVelocityModuleInterface
{
public:
  ObstacleVelocityModuleInterface(rclcpp::Node * node, const VehicleInfo & vehicle_info)
  : clock_(node->get_clock()), vehicle_info_(vehicle_info)
  {
    objects_of_interest_marker_interface_ = std::make_unique<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
      node, "obstacle_cruise_planner");
  }

  void plan_velocity() { objects_of_interest_marker_interface_->publishMarkerArray(); }

protected:
  rclcpp::Clock::SharedPtr clock_;
  VehicleInfo vehicle_info_;
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  BehaviorDeterminationParam behavior_determination_param_;
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_VELOCITY_MODULE_INTERFACE_HPP_
