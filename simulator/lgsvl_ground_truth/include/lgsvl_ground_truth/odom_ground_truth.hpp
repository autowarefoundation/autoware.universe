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

#ifndef ODOM_GROUND_TRUTH__HPP_
#define ODOM_GROUND_TRUTH__HPP_

#include <lgsvl_ground_truth/visibility_control.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace lgsvl_ground_truth {

class LGSVL_GROUND_TRUTH_PUBLIC OdomGroundTruth : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) const;

public:
  explicit OdomGroundTruth(const rclcpp::NodeOptions & /*options*/);
};

}  // namespace lgsvl_ground_truth

#endif  // ODOM_GROUND_TRUTH__HPP_
