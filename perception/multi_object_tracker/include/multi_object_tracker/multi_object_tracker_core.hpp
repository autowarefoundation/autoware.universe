/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * v1.0 Yukihiro Saito
 */

#ifndef MULTI_OBJECT_TRACKER_CORE_HPP_
#define MULTI_OBJECT_TRACKER_CORE_HPP_

#include "multi_object_tracker/data_association/data_association.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <vector>

class MultiObjectTracker : public rclcpp::Node
{
public:
  explicit MultiObjectTracker(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    dynamic_object_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    dynamic_object_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;  // publish timer

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void measurementCallback(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr
    input_objects_msg);
  void publishTimerCallback();

  std::string world_frame_id_;  // tracking frame
  std::list<std::shared_ptr<Tracker>> list_tracker_;
  std::unique_ptr<DataAssociation> data_association_;
  bool enable_delay_compensation_;
};

#endif
