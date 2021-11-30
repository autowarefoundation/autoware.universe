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

#ifndef PACMOD_ADDITIONAL_DEBUG_PUBLISHER__PACMOD_ADDITIONAL_DEBUG_PUBLISHER_NODE_HPP_
#define PACMOD_ADDITIONAL_DEBUG_PUBLISHER__PACMOD_ADDITIONAL_DEBUG_PUBLISHER_NODE_HPP_

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autoware_debug_msgs/msg/float32_multi_array_stamped.hpp"

class PacmodAdditionalDebugPublisherNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
  autoware_debug_msgs::msg::Float32MultiArrayStamped debug_value_;
  void canTxCallback(const can_msgs::msg::Frame::ConstSharedPtr msg);

public:
  PacmodAdditionalDebugPublisherNode();
  ~PacmodAdditionalDebugPublisherNode() {}
};

#endif  // PACMOD_ADDITIONAL_DEBUG_PUBLISHER__PACMOD_ADDITIONAL_DEBUG_PUBLISHER_NODE_HPP_
