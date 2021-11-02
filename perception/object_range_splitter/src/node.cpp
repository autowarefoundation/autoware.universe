// Copyright 2020 TierIV
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

#include "object_range_splitter/node.hpp"

namespace object_range_splitter
{
ObjectRangeSplitterNode::ObjectRangeSplitterNode(const rclcpp::NodeOptions & node_options)
: Node("object_range_splitter_node", node_options)
{
  using std::placeholders::_1;
  spilt_range_ = declare_parameter("split_range", 30.0);
  sub_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectRangeSplitterNode::objectCallback, this, _1));
  long_range_object_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "output/long_range_object", rclcpp::QoS{1});
  short_range_object_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
      "output/short_range_object", rclcpp::QoS{1});
}

void ObjectRangeSplitterNode::objectCallback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  // Guard
  if (
    long_range_object_pub_->get_subscription_count() < 1 &&
    short_range_object_pub_->get_subscription_count() < 1) {
    return;
  }
  // build output msg
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_long_range_object_msg,
    output_short_range_object_msg;
  output_long_range_object_msg.header = input_msg->header;
  output_short_range_object_msg.header = input_msg->header;

  // split
  for (const auto & feature_object : input_msg->feature_objects) {
    const auto & position = feature_object.object.state.pose_covariance.pose.position;
    const auto object_sq_dist = position.x * position.x + position.y * position.y;
    if (object_sq_dist < spilt_range_ * spilt_range_) {  // short range
      output_short_range_object_msg.feature_objects.push_back(feature_object);
    } else {  // long range
      output_long_range_object_msg.feature_objects.push_back(feature_object);
    }
  }

  // publish output msg
  long_range_object_pub_->publish(output_long_range_object_msg);
  short_range_object_pub_->publish(output_short_range_object_msg);
}
}  // namespace object_range_splitter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_range_splitter::ObjectRangeSplitterNode)
