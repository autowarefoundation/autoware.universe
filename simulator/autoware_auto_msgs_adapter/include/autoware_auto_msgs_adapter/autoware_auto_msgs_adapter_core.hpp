// Copyright 2023 The Autoware Foundation
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
#ifndef AUTOWARE_AUTO_MSGS_ADAPTER__AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_
#define AUTOWARE_AUTO_MSGS_ADAPTER__AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/type_adapter.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>

/// custom_type: autoware_control_msgs::msg::Control
/// ros_message_type: autoware_auto_control_msgs::msg::AckermannControlCommand
template <>
struct rclcpp::TypeAdapter<
  autoware_control_msgs::msg::Control, autoware_auto_control_msgs::msg::AckermannControlCommand>
{
  using is_specialized = std::true_type;
  using custom_type = autoware_control_msgs::msg::Control;
  using ros_message_type = autoware_auto_control_msgs::msg::AckermannControlCommand;

  /// \brief Convert from custom type to ROS message type. Will be used when subscribing to a
  /// autoware_auto_control_msgs::msg::AckermannControlCommand topic.
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.stamp = source.stamp;

    const auto & lateral_auto = source.lateral;
    auto & lateral = destination.lateral;
    lateral.stamp = lateral_auto.stamp;
    lateral.steering_tire_angle = lateral_auto.steering_tire_angle;
    lateral.steering_tire_rotation_rate = lateral_auto.steering_tire_rotation_rate;

    const auto & longitudinal_auto = source.longitudinal;
    auto & longitudinal = destination.longitudinal;
    longitudinal.stamp = longitudinal_auto.stamp;
    longitudinal.acceleration = longitudinal_auto.acceleration;
    longitudinal.jerk = longitudinal_auto.jerk;
    longitudinal.speed = longitudinal_auto.velocity;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.stamp = source.stamp;

    const auto & lateral = source.lateral;
    auto & lateral_auto = destination.lateral;
    lateral_auto.stamp = lateral.stamp;
    lateral_auto.steering_tire_angle = lateral.steering_tire_angle;
    lateral_auto.steering_tire_rotation_rate = lateral.steering_tire_rotation_rate;

    const auto & longitudinal = source.longitudinal;
    auto & longitudinal_auto = destination.longitudinal;
    longitudinal_auto.stamp = longitudinal.stamp;
    longitudinal_auto.acceleration = longitudinal.acceleration;
    longitudinal_auto.jerk = longitudinal.jerk;
    longitudinal_auto.velocity = longitudinal.speed;
  }
};

namespace autoware_auto_msgs_adapter
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_control_msgs::msg::Control;

using Adapter = rclcpp::TypeAdapter<Control, AckermannControlCommand>;
class AutowareAutoMsgsAdapterNode : public rclcpp::Node
{
public:
  explicit AutowareAutoMsgsAdapterNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<Adapter>::SharedPtr sub_control_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_ackermann_control_command_;
};
}  // namespace autoware_auto_msgs_adapter

#endif  // AUTOWARE_AUTO_MSGS_ADAPTER__AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_
