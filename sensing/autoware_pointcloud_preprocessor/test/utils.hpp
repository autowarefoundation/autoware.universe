// Copyright 2024 TIER IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstdint>
#include <string>

geometry_msgs::msg::TransformStamped generateTransformMsg(
  const int32_t seconds, const uint32_t nanoseconds, const std::string & parent_frame,
  const std::string & child_frame, double x, double y, double z, double qx, double qy, double qz,
  double qw)
{
  rclcpp::Time timestamp(seconds, nanoseconds, RCL_ROS_TIME);
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = timestamp;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation.x = qx;
  tf_msg.transform.rotation.y = qy;
  tf_msg.transform.rotation.z = qz;
  tf_msg.transform.rotation.w = qw;
  return tf_msg;
}

#endif  // UTILS_HPP_
