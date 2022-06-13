// Copyright 2015-2019 Autoware Foundation
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

#include "twist_estimator/twist_estimator_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <cmath>
#include <memory>
#include <string>

TwistEstimator::TwistEstimator()
: Node("twist_estimator"), message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2))
{
  sensing_twist_with_covariance_sub_ =
    create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "in_twist_with_covariance", rclcpp::QoS{100},
      std::bind(&TwistEstimator::callbackTwistWithCovariance, this, std::placeholders::_1));

  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "out_twist_with_covariance", rclcpp::QoS{10});
}

TwistEstimator::~TwistEstimator() {}

void TwistEstimator::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr)
{
  geometry_msgs::msg::TwistWithCovarianceStamped msg = *twist_with_cov_msg_ptr;

  // clear velocity and angular velocity if vehicle is stopped
  if (std::fabs(msg.twist.twist.angular.z) < 0.01 && std::fabs(msg.twist.twist.linear.x) < 0.01) {
    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;
  }

  twist_with_covariance_pub_->publish(msg);
}
