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

#include <autoware_auto_msgs_adapter/autoware_auto_msgs_adapter_core.hpp>

#include <gtest/gtest.h>

class AutowareAutoMsgsAdapterFixture : public testing::Test
{
protected:
  using AutowareAutoMsgsAdapterNode = autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_ = std::make_shared<AutowareAutoMsgsAdapterNode>(node_options);
  }

  void TearDown() override { rclcpp::shutdown(); }

  autoware_auto_control_msgs::msg::AckermannControlCommand ackermann_control_command_;

  AutowareAutoMsgsAdapterNode::SharedPtr node_;
};

TEST_F(AutowareAutoMsgsAdapterFixture, Test1_1)  // NOLINT for gtest
{
  auto dummy_node = std::make_shared<rclcpp::Node>("dummy_node", rclcpp::NodeOptions{});
  auto pub = dummy_node->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "ackermann_control_command", rclcpp::QoS{1});
  pub->publish(ackermann_control_command_);
}