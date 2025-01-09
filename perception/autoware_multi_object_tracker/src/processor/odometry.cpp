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

#include "odometry.hpp"

#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>

namespace autoware::multi_object_tracker
{

Odometry::Odometry(rclcpp::Node & node, const std::string & world_frame_id)
: node_(node),
  world_frame_id_(world_frame_id),
  tf_buffer_(node_.get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create tf timer
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    node_.get_node_base_interface(), node_.get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);
}

}  // namespace autoware::multi_object_tracker
