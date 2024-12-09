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

#ifndef TRAFFIC_LIGHT_SELECTOR_NODE_HPP_
#define TRAFFIC_LIGHT_SELECTOR_NODE_HPP_

#include "autoware/universe_utils/ros/transform_listener.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"

#include "tier4_perception_msgs/msg/detected_object_with_feature.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include "tier4_perception_msgs/msg/traffic_light_roi.hpp"
#include "tier4_perception_msgs/msg/traffic_light_roi_array.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

class TrafficLightSelectorNode : public rclcpp::Node
{
public:
  explicit TrafficLightSelectorNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  message_filters::Subscriber<DetectedObjectsWithFeature> detected_rois_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> rough_rois_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> expected_rois_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    DetectedObjectsWithFeature, TrafficLightRoiArray, TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  void objectsCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr & detected_rois_msg,
    const TrafficLightRoiArray::ConstSharedPtr & rough_rois_msg,
    const TrafficLightRoiArray::ConstSharedPtr & expect_rois_msg);
  // Publisher
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr pub_traffic_light_rois_;
  bool debug_{false};
  // declare publisher for debug image
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_SELECTOR_NODE_HPP_
