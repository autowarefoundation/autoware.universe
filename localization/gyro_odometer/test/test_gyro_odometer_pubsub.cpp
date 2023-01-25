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
    imu_pub_ = create_publisher<Imu>("/gyro_odometer/input/imu", 1);
    vehicle_velocity_pub_ = create_publisher<TwistWithCovarianceStamped>("/gyro_odometer/input/velocity", 1);
    twist_sub_ = create_subscription<TwistWithCovarianceStamped>(
      "/gyro_odometer/output/twist", 1,
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

bool isAllOK(const std::vector<TwistWithCovarianceStamped::ConstSharedPtr> & twists)
{
  (void)twists;
  // for (const auto & diag : diags) {
  //   for (const auto & status : diag->status) {
  //     if (status.level != DiagnosticStatus::OK) {
  //       return false;
  //     }
  //   }
  // }
  return true;
}

void runWithBothMessages(const Imu & imu, const TwistWithCovarianceStamped & velocity)
{
  auto gyro_odometer_node = std::make_shared<GyroOdometer>(getNodeOptionsWithDefaultParams());
  auto manager_node = std::make_shared<PubSubManager>();
  EXPECT_GE(manager_node->imu_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager_node->imu_pub_->publish(imu);
  manager_node->vehicle_velocity_pub_->publish(velocity);
  spinSome(gyro_odometer_node);
  spinSome(manager_node);

  EXPECT_GE(manager_node->received_twists_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager_node->received_twists_));
}

void runWithImuOnly(const Imu & imu)
{
  auto gyro_odometer_node = std::make_shared<GyroOdometer>(getNodeOptionsWithDefaultParams());
  auto manager_node = std::make_shared<PubSubManager>();
  EXPECT_GE(manager_node->imu_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager_node->imu_pub_->publish(imu);
  spinSome(gyro_odometer_node);
  spinSome(manager_node);

  EXPECT_EQ(manager_node->received_twists_.size(), 0U);
}

// OK cases
TEST(GyroOdometer, TestGyroOdometerOK)
{
  runWithBothMessages(generateDefaultImu(), generateDefaultVelocity());
}

// Bad cases
TEST(GyroOdometer, TestGyroOdometerNG)
{
  runWithImuOnly(generateDefaultImu());
}

