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

#ifndef AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
#define AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/publisher_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

namespace debug_publisher
{
template<class T_msg, class T>
T_msg toMsg(const T & data)
{
  T_msg msg;
  msg.data = data;
  return msg;
}
}  // namespace debug_publisher

namespace autoware_utils
{
class DebugPublisher
{
public:
  explicit DebugPublisher(rclcpp::Node * node, const char * ns)
  : node_(node), ns_(ns) {}

  template<
    class T, std::enable_if_t<
      !rclcpp::serialization_traits::is_serialized_message_class<T>::value,
      std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    if (pub_map_.count(name) == 0) {
      pub_map_[name] = node_->create_publisher<T>(std::string(ns_) + "/" + name, qos);
    }

    std::dynamic_pointer_cast<rclcpp::Publisher<T>>(pub_map_.at(name))->publish(data);
  }

  template<
    class T_msg, class T,
    std::enable_if_t<
      !rclcpp::serialization_traits::is_serialized_message_class<T>::value, std::nullptr_t> =
    nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    publish(name, debug_publisher::toMsg<T_msg>(data), qos);
  }

private:
  rclcpp::Node * node_;
  const char * ns_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> pub_map_;
};
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
