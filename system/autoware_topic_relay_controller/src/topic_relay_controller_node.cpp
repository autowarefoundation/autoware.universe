// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "topic_relay_controller_node.hpp"

#include <memory>
#include <string>

namespace autoware::topic_relay_controller
{
TopicRelayController::TopicRelayController(const rclcpp::NodeOptions & options)
: Node("topic_relay_controller", options), is_relaying_(true)
{
  // Parameter
  node_param_.topic = declare_parameter<std::string>("topic");
  node_param_.remap_topic = declare_parameter<std::string>("remap_topic");
  node_param_.qos = declare_parameter("qos", 1);
  node_param_.transient_local = declare_parameter("transient_local", false);
  node_param_.best_effort = declare_parameter("best_effort", false);
  node_param_.is_transform = (node_param_.topic == "/tf" || node_param_.topic == "/tf_static");
  node_param_.enable_relay_control = declare_parameter<bool>("enable_relay_control");
  node_param_.srv_name = declare_parameter<std::string>("srv_name");

  if (node_param_.is_transform) {
    node_param_.frame_id = declare_parameter<std::string>("frame_id");
    node_param_.child_frame_id = declare_parameter<std::string>("child_frame_id");
  } else {
    node_param_.topic_type = declare_parameter<std::string>("topic_type");
  }

  // Service
  if (node_param_.enable_relay_control) {
    srv_change_relay_control_ = create_service<tier4_system_msgs::srv::ChangeTopicRelayControl>(
      node_param_.srv_name,
      [this](
        const tier4_system_msgs::srv::ChangeTopicRelayControl::Request::SharedPtr request,
        tier4_system_msgs::srv::ChangeTopicRelayControl::Response::SharedPtr response) {
        is_relaying_ = request->relay_on;
        response->status.success = true;
      });
  }

  // Subscriber
  rclcpp::QoS qos = rclcpp::QoS{node_param_.qos};
  if (node_param_.transient_local) {
    qos.transient_local();
  }
  if (node_param_.best_effort) {
    qos.best_effort();
  }

  if (node_param_.is_transform) {
    // Publisher
    pub_transform_ = this->create_publisher<tf2_msgs::msg::TFMessage>(node_param_.remap_topic, qos);

    sub_transform_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      node_param_.topic, qos, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (
            transform.header.frame_id == node_param_.frame_id &&
            transform.child_frame_id == node_param_.child_frame_id && is_relaying_) {
            pub_transform_->publish(*msg);
          }
        }
      });
  } else {
    // Publisher
    pub_topic_ =
      this->create_generic_publisher(node_param_.remap_topic, node_param_.topic_type, qos);

    sub_topic_ = this->create_generic_subscription(
      node_param_.topic, node_param_.topic_type, qos,
      [this]([[maybe_unused]] std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (is_relaying_) pub_topic_->publish(*msg);
      });
  }
}
}  // namespace autoware::topic_relay_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::topic_relay_controller::TopicRelayController)
