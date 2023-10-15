// Copyright 2022 Tier IV, Inc.
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

#ifndef EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_
#define EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

namespace emergency_goal_manager
{

struct Parameters
{
  int update_rate;             // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;          // [m/s^3]
};

class EmergencyGoalManagerNode : public rclcpp::Node
{
public:
  explicit EmergencyGoalManagerNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Subscriber

  // Server

  // Publisher

  // Timer

  // States

  // Algorithm
};

}  // namespace emergency_goal_manager

#endif  // EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_
