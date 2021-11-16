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

// Check that state machine is inline and does something
TEST_F(SanityChecks, StateMachine)
{
  const auto control_topic = "vehicle_command";

  // Construct
  rclcpp::NodeOptions options{};
  options
  .append_parameter_override("control_command", "basic");

  const auto vi_node = std::make_shared<TestVINode>(
    "state_machine_vi_node", options, false);  // no failure

  // Test publisher
  const auto pub_node = std::make_shared<rclcpp::Node>("state_machine_vi_pub_node");
  const auto test_pub =
    pub_node->create_publisher<VehicleControlCommand>(control_topic, rclcpp::QoS{10});
  VehicleControlCommand msg{};
  {
    msg.stamp.sec = 5;
    msg.stamp.nanosec = 12345U;
    msg.long_accel_mps2 = -12.0F;
    msg.front_wheel_angle_rad = 4.0F;
    msg.rear_wheel_angle_rad = 5.0F;
  }
  // Publish some stuff
  constexpr auto max_iters{100};
  auto count{0};
  while (!vi_node->interface().basic_called()) {
    test_pub->publish(msg);
    ++msg.stamp.nanosec;
    rclcpp::spin_some(vi_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
    ++count;
    if (count > max_iters) {
      EXPECT_TRUE(false);  // soft fail
      break;
    }
  }
  // Check state
  {
    EXPECT_TRUE(vi_node->interface().basic_called());
    EXPECT_FALSE(vi_node->interface().raw_called());
    EXPECT_TRUE(vi_node->interface().state_called());
  }
  // check sanitized control message
  {
    const auto & msg = vi_node->interface().control();
    EXPECT_FLOAT_EQ(msg.long_accel_mps2, -3.0F);
    EXPECT_FLOAT_EQ(msg.front_wheel_angle_rad, 0.331F);
    // EXPECT_FLOAT_EQ(msg.rear_wheel_angle_rad, 3.0F);  // not implemented
  }
  // Check injected state command
  {
    const auto & msg = vi_node->interface().state();
    EXPECT_EQ(msg.gear, VehicleStateCommand::GEAR_REVERSE);
    EXPECT_EQ(msg.blinker, VehicleStateCommand::BLINKER_NO_COMMAND);
    EXPECT_EQ(msg.wiper, WipersCommand::NO_COMMAND);
    EXPECT_EQ(msg.headlight, HeadlightsCommand::NO_COMMAND);
    EXPECT_EQ(msg.mode, VehicleStateCommand::MODE_NO_COMMAND);
    EXPECT_FALSE(msg.hand_brake);
    EXPECT_FALSE(msg.horn);
  }
}
