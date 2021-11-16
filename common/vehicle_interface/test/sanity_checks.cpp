// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "test_vi_node.hpp"

// Only check that you can minimally translate a raw command
// No reporting, or other kinds of commands
TEST_F(SanityChecks, RawTranslation)
{
  const auto raw_topic = "raw_command";
  // Construct

  rclcpp::NodeOptions options{};
  options
  .append_parameter_override("control_command", "raw");

  const auto vi_node = std::make_shared<TestVINode>(
    "sanity_check_vi_node", options, false);  // no failure

  // Test publisher
  const auto pub_node = std::make_shared<rclcpp::Node>("sanity_check_vi_pub_node");
  const auto test_pub = pub_node->create_publisher<RawControlCommand>(raw_topic, rclcpp::QoS{10});
  RawControlCommand msg{};
  {
    msg.stamp.sec = 5;
    msg.stamp.nanosec = 12345U;
    msg.brake = 33U;
    msg.throttle = 67U;
    msg.front_steer = 12;
    msg.rear_steer = -17;
  }
  // Publish some stuff
  constexpr auto max_iters{100};
  auto count{0};
  while (!vi_node->interface().raw_called()) {
    test_pub->publish(msg);
    rclcpp::spin_some(vi_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
    ++count;
    if (count > max_iters) {
      EXPECT_TRUE(false);  // soft fail
      break;
    }
  }
  // Check
  EXPECT_TRUE(vi_node->interface().raw_called());
  EXPECT_FALSE(vi_node->interface().basic_called());
  EXPECT_FALSE(vi_node->interface().state_called());
  EXPECT_EQ(msg, vi_node->interface().msg());
}
