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

#include "test_gyro_odometer_helper.hpp"

using sensor_msgs::msg::Imu;
using geometry_msgs::msg::TwistWithCovarianceStamped;

Imu generateDefaultImu()
{
  Imu imu;
  return imu;
}

TwistWithCovarianceStamped generateDefaultVelocity()
{
  TwistWithCovarianceStamped twist;
  return twist;
}

rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
{
  rclcpp::NodeOptions node_options;

  // for gyro_odometer
  node_options.append_parameter_override("output_frame", "base_link");
  node_options.append_parameter_override("message_timeout_sec", 0.1);
  return node_options;
}
