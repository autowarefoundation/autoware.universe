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

#ifndef EMERGENCY_STOP_OPERATOR_HPP_
#define EMERGENCY_STOP_OPERATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace emergency_handler::emergency_stop_operator
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

struct Param
{
  int update_rate;
  double target_acceleration;
  double target_jerk;
  bool turning_hazard_on;
  bool use_parking_after_stopped;
};

class EmergencyStopOperator
{
public:
  explicit EmergencyStopOperator(rclcpp::Node * node);
  bool operate();
  bool cancel();

private:
  // Parameters
  Param params_;

  // Subscriber
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  void onControlCommand(AckermannControlCommand::ConstSharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;
  void publishControlCmd();

  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_light_cmd_;
  void publishHazardLightCmd();

  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  void publishGearCmd();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  // Alrogithm
  bool is_prev_control_cmd_subscribed_;
  AckermannControlCommand prev_control_cmd_;
  bool is_stopped_;

  rclcpp::Node * node_;
};

}  // namespace emergency_handler::emergency_stop_operator

#endif  // EMERGENCY_STOP_OPERATOR_HPP_
