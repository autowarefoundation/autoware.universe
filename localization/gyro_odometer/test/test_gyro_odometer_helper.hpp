// Copyright 2023 TIER IV, Inc.
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

#ifndef TEST_PLANNING_VALIDATOR_HELPER_HPP_
#define TEST_PLANNING_VALIDATOR_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

using sensor_msgs::msg::Imu;
using geometry_msgs::msg::TwistWithCovarianceStamped;

Imu generateDefaultImu(const TwistWithCovarianceStamped & twist_ground_truth);
TwistWithCovarianceStamped generateDefaultVelocity(const TwistWithCovarianceStamped & twist_ground_truth);
rclcpp::NodeOptions getNodeOptionsWithDefaultParams();

#endif  // TEST_PLANNING_VALIDATOR_HELPER_HPP_
