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

#ifndef DETECTED_OBJECT_FEATURE_REMOVER_NODE_HPP_
#define DETECTED_OBJECT_FEATURE_REMOVER_NODE_HPP_

#include "autoware_utils/ros/published_time_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

#include <memory>

namespace autoware::detected_object_feature_remover
{
using autoware_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;

class DetectedObjectFeatureRemover : public rclcpp::Node
{
public:
  explicit DetectedObjectFeatureRemover(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr sub_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
  void objectCallback(const DetectedObjectsWithFeature::ConstSharedPtr input);
  void convert(const DetectedObjectsWithFeature & objs_with_feature, DetectedObjects & objs);
};

}  // namespace autoware::detected_object_feature_remover

#endif  // DETECTED_OBJECT_FEATURE_REMOVER_NODE_HPP_
