// Copyright 2021 TierIV
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

#include "acceleration_estimator/acceleration_estimator.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

AccelerationEstimator::AccelerationEstimator(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&AccelerationEstimator::callbackOdometry, this, _1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist", 1, std::bind(&AccelerationEstimator::callbackTwistWithCovariance, this, _1));

  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("output/accel", 1);

  prev_twist_ptr_ = nullptr;
  prev_accel_ptr_ = nullptr;
  accel_lowpass_gain_ = declare_parameter("accel_lowpass_gain", 0.5);
  use_odom_ = declare_parameter("use_odom", true);

}

void AccelerationEstimator::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!use_odom_) return;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  estimateAccel(
    std::make_shared<geometry_msgs::msg::TwistStamped>(twist)
  );
}

void AccelerationEstimator::callbackTwistWithCovariance(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (use_odom_) return;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  estimateAccel(
    std::make_shared<geometry_msgs::msg::TwistStamped>(twist)
  );
}

void AccelerationEstimator::estimateAccel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
  accel_msg.header = msg->header;

  if (prev_accel_ptr_ != nullptr && prev_twist_ptr_ != nullptr) {
    const double dt = std::max(
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_twist_ptr_->header.stamp)).seconds(),
      1.0e-3
    );
    accel_msg.accel.accel.linear.x = (msg->twist.linear.x - prev_twist_ptr_->twist.linear.x) / dt;
    accel_msg.accel.accel.linear.y = (msg->twist.linear.y - prev_twist_ptr_->twist.linear.y) / dt;
    accel_msg.accel.accel.linear.z = (msg->twist.linear.z - prev_twist_ptr_->twist.linear.z) / dt;
    accel_msg.accel.accel.angular.x = (msg->twist.angular.x - prev_twist_ptr_->twist.angular.x) / dt;
    accel_msg.accel.accel.angular.y = (msg->twist.angular.y - prev_twist_ptr_->twist.angular.y) / dt;
    accel_msg.accel.accel.angular.z = (msg->twist.angular.z - prev_twist_ptr_->twist.angular.z) / dt;

    accel_msg.accel.accel.linear.x = accel_lowpass_gain_ * accel_msg.accel.accel.linear.x + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.linear.x;
    accel_msg.accel.accel.linear.y = accel_lowpass_gain_ * accel_msg.accel.accel.linear.y + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.linear.y;
    accel_msg.accel.accel.linear.z = accel_lowpass_gain_ * accel_msg.accel.accel.linear.z + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.linear.z;
    accel_msg.accel.accel.angular.x = accel_lowpass_gain_ * accel_msg.accel.accel.angular.x + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.angular.x;
    accel_msg.accel.accel.angular.y = accel_lowpass_gain_ * accel_msg.accel.accel.angular.y + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.angular.y;
    accel_msg.accel.accel.angular.z = accel_lowpass_gain_ * accel_msg.accel.accel.angular.z + (1 - accel_lowpass_gain_) * prev_accel_ptr_->accel.accel.angular.z;
  }

  pub_accel_->publish(accel_msg);
  prev_twist_ptr_ = msg;
  prev_accel_ptr_ = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>(accel_msg);
}
