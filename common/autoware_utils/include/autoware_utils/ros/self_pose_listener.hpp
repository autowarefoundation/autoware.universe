// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
#define AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/ros/transform_listener.hpp"

namespace autoware_utils
{
class SelfPoseListener
{
public:
  explicit SelfPoseListener(rclcpp::Node * node)
  : transform_listener_(node) {}
  void waitForFirstPose()
  {
    while (rclcpp::ok()) {
      const auto pose = getPoseAt(rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
      if (pose) {
        return;
      }
      RCLCPP_INFO(transform_listener_.getLogger(), "waiting for self pose...");
    }
  }

  geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose()
  {
    return getPoseAt(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0));
  }

  geometry_msgs::msg::PoseStamped::ConstSharedPtr getPoseAt(
    const rclcpp::Time & time, const rclcpp::Duration & duration)
  {
    const auto tf = transform_listener_.getTransform("map", "base_link", time, duration);

    if (!tf) {
      return {};
    }

    geometry_msgs::msg::PoseStamped::SharedPtr pose(new geometry_msgs::msg::PoseStamped());
    *pose = transform2pose(*tf);

    return geometry_msgs::msg::PoseStamped::ConstSharedPtr(pose);
  }

private:
  TransformListener transform_listener_;
};
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
