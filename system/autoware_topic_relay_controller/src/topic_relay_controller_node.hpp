// Copyright 2025 TIER IV, Inc.
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

#ifndef TOPIC_RELAY_CONTROLLER_HPP_
#define TOPIC_RELAY_CONTROLLER_HPP_

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

namespace autoware::topic_relay_controller
{
class TopicRelayController : public rclcpp::Node
{
public:
  explicit TopicRelayController(const rclcpp::NodeOptions & options);

private:
};
}  // namespace autoware::topic_relay_controller

#endif  // TOPIC_RELAY_CONTROLLER_HPP_