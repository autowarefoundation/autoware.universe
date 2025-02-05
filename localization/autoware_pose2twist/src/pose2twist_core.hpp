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

#ifndef POSE2TWIST_CORE_HPP_
#define POSE2TWIST_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::pose2twist
{
// Compute the relative rotation of q2 from q1 as a rotation vector
geometry_msgs::msg::Vector3 compute_relative_rotation_vector(
  const tf2::Quaternion & q1, const tf2::Quaternion & q2);

class Pose2Twist : public rclcpp::Node
{
public:
  explicit Pose2Twist(const rclcpp::NodeOptions & options);

private:
  void callback_pose(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_ptr);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr linear_x_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr angular_z_pub_;
};
}  // namespace autoware::pose2twist

#endif  // POSE2TWIST_CORE_HPP_
