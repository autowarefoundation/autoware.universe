// Copyright 2020 Tier IV, Inc.
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

#ifndef EMERGENCY_HANDLER__STATE_TIMEOUT_CHECKER_CORE_HPP_
#define EMERGENCY_HANDLER__STATE_TIMEOUT_CHECKER_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "std_msgs/msg/bool.hpp"

struct TimeoutThreshold
{
  double InitializingVehicle;
  double WaitingForRoute;
  double Planning;
};

class StateTimeoutChecker : public rclcpp::Node
{
public:
  StateTimeoutChecker();

private:
  // Parameter
  int update_rate_;
  TimeoutThreshold th_timeout_;

  // Subscriber
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr
    sub_autoware_state_;

  rclcpp::Time autoware_state_received_time_;

  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;

  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    pub_is_state_timeout_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  void onTimer();

  // Algorithm
  rclcpp::Time state_change_time_;

  bool isTimeout(const rclcpp::Time & state_change_time, const double th_timeout);
  bool isStateTimeout();
};

#endif  // EMERGENCY_HANDLER__STATE_TIMEOUT_CHECKER_CORE_HPP_
