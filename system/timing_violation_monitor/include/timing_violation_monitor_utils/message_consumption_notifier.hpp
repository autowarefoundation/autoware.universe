// Copyright 2023 Tier IV, Inc.
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

#ifndef TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_
#define TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"

#include "tier4_system_msgs/msg/message_tracking_tag.hpp"

#include <memory>
#include <string>

namespace timing_violation_monitor_utils
{
class MessageConsumptionNotifier
{
public:
  using MessageTrackingTag = tier4_system_msgs::msg::MessageTrackingTag;
  using MTTPublisher = rclcpp::Publisher<MessageTrackingTag>;

  RCLCPP_PUBLIC
  MessageConsumptionNotifier(
    rclcpp::Node * node, const std::string & notification_topic,
    const rclcpp::QoS & notification_qos);

  void notify(const builtin_interfaces::msg::Time & consumed_msg_stamp);

private:
  std::shared_ptr<MTTPublisher> pub_;
};

}  // namespace timing_violation_monitor_utils

#endif  // TIMING_VIOLATION_MONITOR_UTILS__MESSAGE_CONSUMPTION_NOTIFIER_HPP_
