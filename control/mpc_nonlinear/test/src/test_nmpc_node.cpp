/*
* Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>
#include "gtest/gtest.h"
#include "nonlinear_mpc_test_node.hpp"
#include "nmpc_test_utils.hpp"

TEST_F(FakeNodeFixture, no_input_stop_control_cmd_is_published)
{
  // Data to test
  ControlCmdMsg::SharedPtr cmd_msg;
  bool is_control_command_received = false;

  // Node
  std::shared_ptr<NonlinearMPCNode> node = makeNonlinearMPCNode();

  // Publishers
  rclcpp::Publisher<TrajectoryMsg>::SharedPtr traj_pub =
    this->create_publisher<TrajectoryMsg>("mpc_nonlinear/input/reference_trajectory");

  rclcpp::Publisher<VelocityMsg>::SharedPtr vel_pub =
    this->create_publisher<VelocityMsg>("mpc_nonlinear/input/current_velocity");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("mpc_nonlinear/input/current_steering");

  // Subscribers
  rclcpp::Subscription<ControlCmdMsg>::SharedPtr cmd_sub = this->create_subscription<ControlCmdMsg>(
    "mpc_nonlinear/output/control_cmd", *this->get_fake_node(),
    [&cmd_msg, &is_control_command_received](const ControlCmdMsg::SharedPtr msg)
    {
      cmd_msg = msg;
      is_control_command_received = true;
    });

  auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // test_utils::spinWhile(node);
  ns_utils::print("Is command received ? ", is_control_command_received ? "True" : "False");

  // No published data: expect a stopped command
  test_utils::waitForMessage(node, this, is_control_command_received, std::chrono::seconds{1LL}, false);
  ASSERT_TRUE(is_control_command_received);

}