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

#include <time_utils/time_utils.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "test_vi_node.hpp"

// Send periodic basic control command, expect that *something* happened
TEST_F(SanityChecks, Filtering)
{
  const auto cmd_topic = "vehicle_command";

  // Construct
  rclcpp::NodeOptions options{};
  options
  .append_parameter_override("control_command", "basic")
  .append_parameter_override("filter.longitudinal.type", "low_pass_filter")
  .append_parameter_override("filter.longitudinal.cutoff_frequency_hz", 30.0F)
  .append_parameter_override("filter.curvature.type", "low_pass_filter")
  .append_parameter_override("filter.curvature.cutoff_frequency_hz", 30.0F)
  .append_parameter_override("filter.front_steer.type", "low_pass_filter")
  .append_parameter_override("filter.front_steer.cutoff_frequency_hz", 30.0F)
  .append_parameter_override("filter.rear_steer.type", "low_pass_filter")
  .append_parameter_override("filter.rear_steer.cutoff_frequency_hz", 30.0F);

  const auto vi_node = std::make_shared<TestVINode>(
    "filter_vi_node", options, false);  // no failure

  // Test publisher
  const auto pub_node = std::make_shared<rclcpp::Node>("filter_vi_pub_node");
  const auto test_pub =
    pub_node->create_publisher<VehicleControlCommand>(cmd_topic, rclcpp::QoS{10});
  // Create periodic input signal at 500 Hz or so
  constexpr auto magnitude = 0.1F;
  constexpr auto period_2 = std::chrono::milliseconds{1LL};
  VehicleControlCommand msg{};
  {
    msg.long_accel_mps2 = magnitude;
    msg.front_wheel_angle_rad = magnitude;
    msg.rear_wheel_angle_rad = magnitude;
  }
  // Publish some stuff
  constexpr auto min_num_msgs{10};
  while (vi_node->interface().controls().size() < min_num_msgs) {
    // Update message
    msg.long_accel_mps2 = -msg.long_accel_mps2;
    msg.front_wheel_angle_rad = -msg.front_wheel_angle_rad;
    msg.rear_wheel_angle_rad = -msg.rear_wheel_angle_rad;
    {
      const auto next_time = ::time_utils::from_message(msg.stamp) + period_2;
      msg.stamp = ::time_utils::to_message(next_time);
    }
    // Publish, check that you're done
    test_pub->publish(msg);
    rclcpp::spin_some(vi_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{1LL});
  }
  // Check magnitude of output signal -> should be less than input signals magnitude
  for (auto idx = 1U; idx < vi_node->interface().controls().size(); ++idx) {
    // Semi hack: Ignore the first one since the time step might be bigger than normal
    const auto & cmd = vi_node->interface().controls()[idx];
    EXPECT_LT(std::fabs(cmd.long_accel_mps2), magnitude) << idx;
    EXPECT_LT(std::fabs(cmd.front_wheel_angle_rad), magnitude) << idx;
    EXPECT_LT(std::fabs(cmd.rear_wheel_angle_rad), magnitude) << idx;
  }
}
