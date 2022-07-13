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
#include "test_nonlinear_mpc_node.hpp"

/**
 * @brief Vehicle initial speed is set to zero and NMPC sends stop commands.
 *
 */
constexpr double EPS = std::numeric_limits<double>::epsilon();

TEST_F(NonlinearMPCNodeTestSuit, getStopMessage)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  fake_node_->setInitialPoseSpeed(0.0);
  fake_node_->setPoseInd(1);
  executor.add_node(fake_node_);
  executor.add_node(nmpc_follower_);

  rclcpp::WallRate rate(100ms);

  std::vector<double> acc_vect;
  //  if (rclcpp::ok()) {
  for (int i = 0; i < 20; ++i) {
    executor.spin_some();

    if (fake_node_->ctrl_cmd_msgs_.has_value()) {
      auto ctrl_cmd_msgs = fake_node_->ctrl_cmd_msgs_.get().control;

      double steering_angle = ctrl_cmd_msgs.steering_angle;
      double steering_vel = ctrl_cmd_msgs.steering_angle_velocity;
      double velocity = ctrl_cmd_msgs.velocity;
      double acceleration = ctrl_cmd_msgs.acceleration;

      // Stopping command values.
      if (
        std::fabs(steering_angle) <= EPS && std::fabs(steering_vel) <= EPS &&
        std::fabs(velocity) <= 0.0 && acceleration <= 1.5) {
        ASSERT_DOUBLE_EQ(acceleration, -1.5);
        break;
      }

      // DEBUG
      acc_vect.emplace_back(acceleration);
      ns_utils::print(" ctrl_cmd_msgs_  steering: ", ctrl_cmd_msgs.steering_angle);
      ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", ctrl_cmd_msgs.steering_angle_velocity);

      ns_utils::print(" ctrl_cmd_msgs_  steering rate : ", ctrl_cmd_msgs.velocity);
      ns_utils::print(" ctrl_cmd_msgs_  steering: ", ctrl_cmd_msgs.acceleration);
    }

    rate.sleep();
  }
}

/**
 * #brief NMPC reads the steering value from the vehicle messages which is set to zero.
 * */
TEST_F(NonlinearMPCNodeTestSuit, nmpcReadsMeasuredSteering)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  fake_node_->setInitialPoseSpeed(1.0);
  fake_node_->setPoseInd(1);
  executor.add_node(fake_node_);
  executor.add_node(nmpc_follower_);

  rclcpp::WallRate rate(100ms);

  for (int i = 0; i < 20; ++i) {
    executor.spin_some();

    if (fake_node_->nmpc_msgs_.has_value()) {
      ns_utils::print("Received NMPC Performance Message");

      auto nmpc_msg = fake_node_->nmpc_msgs_->nmpc_performance_msg;
      ns_utils::print("NMPC performance steering_measured ", nmpc_msg.steering_measured);

      ASSERT_DOUBLE_EQ(nmpc_msg.steering_measured, 0.0);
    }

    rate.sleep();
  }
}

/**
 * #brief NMPC reads the measured vehicle speed.
 * */
TEST_F(NonlinearMPCNodeTestSuit, nmpcReadsMeasuredSpeed)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  double set_velocity = 1.0;

  fake_node_->setInitialPoseSpeed(set_velocity);
  fake_node_->setPoseInd(1);
  executor.add_node(fake_node_);
  executor.add_node(nmpc_follower_);

  rclcpp::WallRate rate(100ms);

  for (int i = 0; i < 20; ++i) {
    executor.spin_some();

    if (fake_node_->nmpc_msgs_.has_value()) {
      ns_utils::print("Received NMPC Performance Message");

      auto nmpc_msg = fake_node_->nmpc_msgs_->nmpc_performance_msg;
      ns_utils::print("NMPC performance measured velocity ", nmpc_msg.velocity_measured);

      ASSERT_DOUBLE_EQ(nmpc_msg.velocity_measured, 1.0);
    }

    rate.sleep();
  }
}

/**
 * #brief NMPC reads the current vehicle position (we only test the yaw).
 * */
TEST_F(NonlinearMPCNodeTestSuit, nmpcGetPosition)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  double set_velocity = 1.0;

  fake_node_->setInitialPoseSpeed(set_velocity);
  fake_node_->setPoseInd(1);
  executor.add_node(fake_node_);
  executor.add_node(nmpc_follower_);

  rclcpp::WallRate rate(100ms);

  for (int i = 0; i < 20; ++i) {
    executor.spin_some();

    if (fake_node_->nmpc_msgs_.has_value()) {
      ns_utils::print("Received NMPC Performance Message");

      auto nmpc_msg = fake_node_->nmpc_msgs_->nmpc_performance_msg;
      ns_utils::print("NMPC performance yaw_vehicle ", nmpc_msg.yaw_vehicle);

      ASSERT_LE(std::fabs(nmpc_msg.yaw_vehicle), EPS);
    }

    rate.sleep();
  }
}

/**
 * #brief NMPC computes yaw and lateral errors. Both of them must be close to zero as the
 * vehicle starts on the trajectory. The errors depends on the smoothed trajectory.
 * */
TEST_F(NonlinearMPCNodeTestSuit, nmpcComputeErrorsAreFinite)
{
  rclcpp::executors::SingleThreadedExecutor executor;

  double set_velocity = 1.0;

  fake_node_->setInitialPoseSpeed(set_velocity);
  fake_node_->setPoseInd(1);
  executor.add_node(fake_node_);
  executor.add_node(nmpc_follower_);

  rclcpp::WallRate rate(100ms);

  for (int i = 0; i < 20; ++i) {
    executor.spin_some();

    if (fake_node_->nmpc_msgs_.has_value()) {
      ns_utils::print("Received NMPC Performance Message");

      auto nmpc_msg = fake_node_->nmpc_msgs_->nmpc_performance_msg;
      ns_utils::print("NMPC performance lateral error ", nmpc_msg.lateral_error);
      ns_utils::print("NMPC performance yaw error ", nmpc_msg.yaw_error);

      ASSERT_LE(std::fabs(nmpc_msg.lateral_error), 1.);
      ASSERT_LE(std::fabs(nmpc_msg.yaw_error), 1.);
    }

    rate.sleep();
  }
}