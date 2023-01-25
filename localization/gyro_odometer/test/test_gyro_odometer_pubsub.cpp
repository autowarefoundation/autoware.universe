// Copyright 2021 Tier IV, Inc.
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

#include "gyro_odometer/gyro_odometer_core.hpp"

#include "test_gyro_odometer_helper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

/*
 * This test checks if twist is published from gyro_odometer
 */
using sensor_msgs::msg::Imu;
using geometry_msgs::msg::TwistWithCovarianceStamped;

class PubSubManager : public rclcpp::Node
{
public:
  PubSubManager() : Node("test_pub_sub")
  {
    imu_pub_ = create_publisher<Imu>("/imu", 1);
    vehicle_velocity_pub_ = create_publisher<TwistWithCovarianceStamped>("/vehicle/twist_with_covariance", 1);
    twist_sub_ = create_subscription<TwistWithCovarianceStamped>(
      "/twist_with_covariance", 1,
      [this](const TwistWithCovarianceStamped::ConstSharedPtr msg) { received_twists_.push_back(msg); });
  }

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr vehicle_velocity_pub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub_;

  std::vector<TwistWithCovarianceStamped::ConstSharedPtr> received_twists_;
};

void spinSome(rclcpp::Node::SharedPtr node_ptr)
{
  for (int i = 0; i < 50; ++i) {
    rclcpp::spin_some(node_ptr);
    rclcpp::WallRate(100).sleep();
  }
}

bool isAllOK(
  const std::vector<TwistWithCovarianceStamped::ConstSharedPtr> & twists,
  const TwistWithCovarianceStamped & twist_ground_truth)
{
  for (const auto & twist : twists) {
    if (twist->twist.twist.linear.x != twist_ground_truth.twist.twist.linear.x) {
      return false;
    }
    if (twist->twist.twist.linear.y != twist_ground_truth.twist.twist.linear.y) {
      return false;
    }
    if (twist->twist.twist.linear.z != twist_ground_truth.twist.twist.linear.z) {
      return false;
    }
  }
  return true;
}

void runWithBothMessages(const Imu & imu, const TwistWithCovarianceStamped & velocity, const TwistWithCovarianceStamped & twist_ground_truth)
{
  auto gyro_odometer_node = std::make_shared<GyroOdometer>(getNodeOptionsWithDefaultParams());
  auto manager_node = std::make_shared<PubSubManager>();
  EXPECT_GE(manager_node->imu_pub_->get_subscription_count(), 1U) << "imu is not connected.";

  manager_node->vehicle_velocity_pub_->publish(velocity); // need this for now, which should eventually be removed
  manager_node->vehicle_velocity_pub_->publish(velocity);
  manager_node->imu_pub_->publish(imu);

  spinSome(gyro_odometer_node);
  spinSome(manager_node);
  spinSome(gyro_odometer_node);

  EXPECT_GE(manager_node->received_twists_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager_node->received_twists_, twist_ground_truth));
}

// OK cases
TEST(GyroOdometer, TestGyroOdometerOK)
{
  TwistWithCovarianceStamped twist_ground_truth;
  twist_ground_truth.header.frame_id = "base_link";
  twist_ground_truth.twist.twist.linear.x = 1.0;
  twist_ground_truth.twist.twist.angular.x = 0.1;
  twist_ground_truth.twist.twist.angular.y = 0.2;
  twist_ground_truth.twist.twist.angular.z = 0.3;

  runWithBothMessages(
    generateDefaultImu(twist_ground_truth),
    generateDefaultVelocity(twist_ground_truth),
    twist_ground_truth
  );
}

// // Bad cases
// TEST(GyroOdometer, TestGyroOdometerNG)
// {
//   runWithImuOnly(generateDefaultImu());
// }

