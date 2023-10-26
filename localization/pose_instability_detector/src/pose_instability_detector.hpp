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

#ifndef POSE_INSTABILITY_DETECTOR_HPP_
#define POSE_INSTABILITY_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PoseInstabilityDetector : public rclcpp::Node
{
  using Odometry = nav_msgs::msg::Odometry;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

public:
  PoseInstabilityDetector();

private:
  void callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr);
  void callback_twist(TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr);
  void callback_timer();

  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

#endif  // POSE_INSTABILITY_DETECTOR_HPP_
