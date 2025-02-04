// Copyright 2022 TIER IV, Inc.
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

#include "position_filter.hpp"

#include <memory>

namespace autoware::detected_object_validation
{
namespace position_filter
{
ObjectPositionFilterNode::ObjectPositionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_position_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  upper_bound_x_ = declare_parameter<float>("upper_bound_x");
  upper_bound_y_ = declare_parameter<float>("upper_bound_y");
  lower_bound_x_ = declare_parameter<float>("lower_bound_x");
  lower_bound_y_ = declare_parameter<float>("lower_bound_y");
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN");
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR");
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK");
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS");
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER");
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE");
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE");
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN");

  // Set publisher/subscriber
  object_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectPositionFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

void ObjectPositionFilterNode::objectCallback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  for (const auto & object : input_msg->objects) {
    const auto & label = object.classification.front().label;
    if (filter_target_.isTarget(label)) {
      if (isObjectInBounds(object)) {
        output_object_msg.objects.emplace_back(object);
      }
    } else {
      output_object_msg.objects.emplace_back(object);
    }
  }

  object_pub_->publish(output_object_msg);
  published_time_publisher_->publish_if_subscribed(object_pub_, output_object_msg.header.stamp);
}

bool ObjectPositionFilterNode::isObjectInBounds(
  const autoware_perception_msgs::msg::DetectedObject & object) const
{
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  return position.x > lower_bound_x_ && position.x < upper_bound_x_ &&
         position.y > lower_bound_y_ && position.y < upper_bound_y_;
}

}  // namespace position_filter
}  // namespace autoware::detected_object_validation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_validation::position_filter::ObjectPositionFilterNode)
