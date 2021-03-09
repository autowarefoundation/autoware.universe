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

#include <memory>
#include <utility>

#include "emergency_handler/state_timeout_checker_core.hpp"

StateTimeoutChecker::StateTimeoutChecker()
: Node("emergency_handler"),
  update_rate_(declare_parameter<int>("update_rate", 10))
{
  using std::placeholders::_1;

  th_timeout_.InitializingVehicle = declare_parameter<double>("timeout.InitializingVehicle", 60.0);
  th_timeout_.WaitingForRoute = declare_parameter<double>("timeout.WaitingForRoute", 60.0);
  th_timeout_.Planning = declare_parameter<double>("timeout.Planning", 60.0);

  // Subscriber
  sub_autoware_state_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
    "input/autoware_state", rclcpp::QoS{1},
    std::bind(&StateTimeoutChecker::onAutowareState, this, _1));

  // Publisher
  pub_is_state_timeout_ =
    create_publisher<autoware_system_msgs::msg::TimeoutNotification>(
    "output/is_state_timeout", rclcpp::QoS{1});

  // Timer
  auto timer_callback = std::bind(&StateTimeoutChecker::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void StateTimeoutChecker::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  if (!autoware_state_ || autoware_state_->state != msg->state) {
    state_change_time_ = this->now();
  }

  autoware_state_ = msg;
  autoware_state_received_time_ = this->now();
}

bool StateTimeoutChecker::isDataReady()
{
  if (!autoware_state_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for autoware_state msg...");
    return false;
  }

  return true;
}

void StateTimeoutChecker::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Check timeout
  autoware_system_msgs::msg::TimeoutNotification is_state_timeout;
  is_state_timeout.is_timeout = isStateTimeout();
  is_state_timeout.stamp = this->now();
  pub_is_state_timeout_->publish(is_state_timeout);
}

bool StateTimeoutChecker::isTimeout(const rclcpp::Time & state_change_time, const double th_timeout)
{
  if (th_timeout == 0.0) {
    return false;
  }

  const auto time_diff = this->now() - state_change_time;
  return time_diff.seconds() >= th_timeout;
}

bool StateTimeoutChecker::isStateTimeout()
{
  using autoware_system_msgs::msg::AutowareState;

  constexpr double th_autoware_state_timeout = 30.0;
  if ((this->now() - autoware_state_received_time_).seconds() > th_autoware_state_timeout) {
    return true;
  }

  if (autoware_state_->state == AutowareState::INITIALIZING_VEHICLE) {
    return isTimeout(state_change_time_, th_timeout_.InitializingVehicle);
  }

  if (autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) {
    return isTimeout(state_change_time_, th_timeout_.WaitingForRoute);
  }

  if (autoware_state_->state == AutowareState::PLANNING) {
    return isTimeout(state_change_time_, th_timeout_.Planning);
  }

  return false;
}
