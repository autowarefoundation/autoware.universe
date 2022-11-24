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

#include <autoware_ad_api_specs/planning.hpp>
#include <component_interface_specs/localization.hpp>
#include <component_interface_specs/planning.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class PlanningNode : public rclcpp::Node
{
public:
  explicit PlanningNode(const rclcpp::NodeOptions & options);

private:
  using VelocityFactorArray = autoware_adapi_v1_msgs::msg::VelocityFactorArray;
  using SteeringFactorArray = autoware_adapi_v1_msgs::msg::SteeringFactorArray;
  Pub<autoware_ad_api::planning::VelocityFactors> pub_velocity_factors_;
  Pub<autoware_ad_api::planning::SteeringFactors> pub_steering_factors_;
  Sub<localization_interface::KinematicState> sub_kinematic_state_;
  Sub<planning_interface::Trajectory> sub_trajectory_;
  std::vector<rclcpp::Subscription<VelocityFactorArray>::SharedPtr> sub_velocity_factors_;
  std::vector<rclcpp::Subscription<SteeringFactorArray>::SharedPtr> sub_steering_factors_;
  std::vector<VelocityFactorArray::ConstSharedPtr> velocity_factors_;
  std::vector<SteeringFactorArray::ConstSharedPtr> steering_factors_;
  rclcpp::TimerBase::SharedPtr timer_;

  double stop_judge_distance_;
  localization_interface::KinematicState::Message::ConstSharedPtr kinematic_state_;
  planning_interface::Trajectory::Message::ConstSharedPtr trajectory_;
};

}  // namespace default_ad_api

#endif  // PLANNING_HPP_
