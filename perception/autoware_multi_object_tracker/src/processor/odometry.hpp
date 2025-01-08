// Copyright 2025 TIER IV, Inc.
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

#ifndef PROCESSOR__ODOMETRY_HPP_
#define PROCESSOR__ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace autoware::multi_object_tracker
{

class Odometry
{
public:
  Odometry(const std::string & world_frame_id, const rclcpp::Clock::SharedPtr clock);

  void setOdometry(const geometry_msgs::msg::PoseStamped & pose) { pose_ = pose; }

private:
  // frame id
  std::string world_frame_id_;  // absolute/relative ground frame

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // time-history of odometry
  geometry_msgs::msg::PoseStamped pose_;
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__ODOMETRY_HPP_
