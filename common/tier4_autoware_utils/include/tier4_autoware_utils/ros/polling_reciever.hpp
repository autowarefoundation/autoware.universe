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

#ifndef TIER4_AUTOWARE_UTILS__ROS__POLLING_RECEIVER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__POLLING_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace tier4_autoware_utils
{

template <typename T>
class InterProcessPollingReceiver
{
private:
  typename rclcpp::Subscription<T>::SharedPtr subscriber;

public:
  std::optional<T> data;
  explicit InterProcessPollingReceiver(rclcpp::Node * node, const std::string & topic_name)
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber = node->create_subscription<T>(
      topic_name, rclcpp::QoS{1}, [node](const typename T::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
  };
  bool takeLatestData()
  {
    rclcpp::MessageInfo message_info;
    T tmp;
    // The queue size (QoS) must be 1 to get the last message data.
    if (subscriber->take(tmp, message_info)) {
      data = tmp;
    }
    return data.has_value();
  };
};

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__POLLING_RECEIVER_HPP_
