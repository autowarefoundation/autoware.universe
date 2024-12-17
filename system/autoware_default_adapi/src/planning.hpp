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

#ifndef PLANNING_HPP_
#define PLANNING_HPP_

#include <autoware/adapi_specs/planning.hpp>
#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>
#include <tier4_planning_msgs/msg/planning_factor_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

using autoware_adapi_v1_msgs::msg::PlanningBehavior;
using autoware_adapi_v1_msgs::msg::SteeringFactor;
using autoware_adapi_v1_msgs::msg::SteeringFactorArray;
using autoware_adapi_v1_msgs::msg::VelocityFactor;
using autoware_adapi_v1_msgs::msg::VelocityFactorArray;
using tier4_planning_msgs::msg::PlanningFactor;
using tier4_planning_msgs::msg::PlanningFactorArray;

const std::map<std::uint16_t, std::uint16_t> direction_map = {
  {PlanningFactor::SHIFT_RIGHT, SteeringFactor::RIGHT},
  {PlanningFactor::SHIFT_LEFT, SteeringFactor::LEFT},
  {PlanningFactor::TURN_RIGHT, SteeringFactor::RIGHT},
  {PlanningFactor::TURN_LEFT, SteeringFactor::LEFT}};

const std::map<std::string, std::string> conversion_map = {
  {"static_obstacle_avoidance", PlanningBehavior::AVOIDANCE},
  {"crosswalk", PlanningBehavior::CROSSWALK},
  {"goal_planner", PlanningBehavior::GOAL_PLANNER},
  {"intersection", PlanningBehavior::INTERSECTION},
  {"lane_change_left", PlanningBehavior::LANE_CHANGE},
  {"lane_change_right", PlanningBehavior::LANE_CHANGE},
  {"merge_from_private", PlanningBehavior::MERGE},
  {"no_stopping_area", PlanningBehavior::NO_STOPPING_AREA},
  {"blind_spot", PlanningBehavior::REAR_CHECK},
  {"obstacle_cruise_planner", PlanningBehavior::ROUTE_OBSTACLE},
  {"motion_velocity_planner", PlanningBehavior::ROUTE_OBSTACLE},
  {"walkway", PlanningBehavior::SIDEWALK},
  {"start_planner", PlanningBehavior::START_PLANNER},
  {"stop_line", PlanningBehavior::STOP_SIGN},
  {"surround_obstacle_checker", PlanningBehavior::SURROUNDING_OBSTACLE},
  {"traffic_light", PlanningBehavior::TRAFFIC_SIGNAL},
  {"detection_area", PlanningBehavior::USER_DEFINED_DETECTION_AREA},
  {"virtual_traffic_light", PlanningBehavior::VIRTUAL_TRAFFIC_LIGHT},
  {"run_out", PlanningBehavior::RUN_OUT}};

class PlanningNode : public rclcpp::Node
{
public:
  explicit PlanningNode(const rclcpp::NodeOptions & options);

private:
  Pub<autoware::adapi_specs::planning::VelocityFactors> pub_velocity_factors_;
  Pub<autoware::adapi_specs::planning::SteeringFactors> pub_steering_factors_;
  Sub<autoware::component_interface_specs::planning::Trajectory> sub_trajectory_;
  Sub<autoware::component_interface_specs::localization::KinematicState> sub_kinematic_state_;
  std::vector<rclcpp::Subscription<PlanningFactorArray>::SharedPtr> sub_factors_;
  std::vector<PlanningFactorArray::ConstSharedPtr> factors_;
  rclcpp::TimerBase::SharedPtr timer_;

  using VehicleStopChecker = autoware::motion_utils::VehicleStopCheckerBase;
  using Trajectory = autoware::component_interface_specs::planning::Trajectory::Message;
  using KinematicState = autoware::component_interface_specs::localization::KinematicState::Message;
  void on_trajectory(const Trajectory::ConstSharedPtr msg);
  void on_kinematic_state(const KinematicState::ConstSharedPtr msg);
  void on_timer();

  double stop_distance_;
  double stop_duration_;
  std::unique_ptr<VehicleStopChecker> stop_checker_;
  Trajectory::ConstSharedPtr trajectory_;
  KinematicState::ConstSharedPtr kinematic_state_;
};

}  // namespace autoware::default_adapi

#endif  // PLANNING_HPP_
