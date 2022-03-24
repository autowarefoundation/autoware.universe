// Copyright 2022 Tier IV, Inc.
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
//
//
// Author: v1.0 Yutaka Shimizu
///

#ifndef SIMPLE_OBJECT_TRACKER__SIMPLE_OBJECT_TRACKER_CORE_HPP_
#define SIMPLE_OBJECT_TRACKER__SIMPLE_OBJECT_TRACKER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

class SimpleObjectTracker : public rclcpp::Node
{
public:
  explicit SimpleObjectTracker(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_object_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void onMeasurement(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg);

  std::string world_frame_id_;  // tracking frame
};

#endif  // SIMPLE_OBJECT_TRACKER__SIMPLE_OBJECT_TRACKER_CORE_HPP_
