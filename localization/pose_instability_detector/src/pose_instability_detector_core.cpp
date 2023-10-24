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

#include "pose_instability_detector_core.hpp"

PoseInstabilityDetector::PoseInstabilityDetector() : Node("pose_instability_detector")
{
  odometry_sub_ = this->create_subscription<Odometry>(
    "~/input/odometry", 10,
    std::bind(&PoseInstabilityDetector::callback_odometry, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&PoseInstabilityDetector::callback_twist, this, std::placeholders::_1));

  const double interval_sec = this->declare_parameter<double>("interval_sec");
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseInstabilityDetector::callback_timer, this));

  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
}

void PoseInstabilityDetector::callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr)
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_odometry" << odometry_msg_ptr->header.stamp.sec);
}

void PoseInstabilityDetector::callback_twist(
  TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_twist" << twist_msg_ptr->header.stamp.sec);
}

void PoseInstabilityDetector::callback_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_timer");
}
