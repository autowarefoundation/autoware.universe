// Copyright 2023 TIER IV, Inc.
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

#include "bytetrack/bytetrack.hpp"

#include <bytetrack/bytetrack_node.hpp>
#include <rclcpp/qos.hpp>

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

#include <rmw/qos_profiles.h>

#include <utility>
#include <vector>

namespace bytetrack
{
ByteTrackNode::ByteTrackNode(const rclcpp::NodeOptions & node_options)
: Node("bytetrack", node_options)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  int track_buffer_length = declare_parameter("track_buffer_length", 30);

  this->bytetrack_ = std::make_unique<bytetrack::ByteTrack>(track_buffer_length);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ByteTrackNode::on_connect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  objects_uuid_pub_ = this->create_publisher<tier4_perception_msgs::msg::DynamicObjectArray>(
    "~/out/objects/debug/uuid", 1);
}

void ByteTrackNode::on_connect()
{
  using std::placeholders::_1;
  if (
    objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0) {
    detection_rect_sub_.reset();
  } else if (!detection_rect_sub_) {
    detection_rect_sub_ =
      this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
        "~/in/rect", 1, std::bind(&ByteTrackNode::on_rect, this, _1));
  }
}

void ByteTrackNode::on_rect(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;
  tier4_perception_msgs::msg::DynamicObjectArray out_objects_uuid;

  // Unpack detection results
  ObjectArray object_array;
  for (auto & feat_obj : msg->feature_objects) {
    Object obj;
    obj.x_offset = feat_obj.feature.roi.x_offset;
    obj.y_offset = feat_obj.feature.roi.y_offset;
    obj.height = feat_obj.feature.roi.height;
    obj.width = feat_obj.feature.roi.width;
    obj.score = feat_obj.object.classification.front().probability;
    obj.type = feat_obj.object.classification.front().label;
    object_array.emplace_back(obj);
  }

  bytetrack::ObjectArray objects = bytetrack_->update_tracker(object_array);
  for (const auto & tracked_object : objects) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = tracked_object.x_offset;
    object.feature.roi.y_offset = tracked_object.y_offset;
    object.feature.roi.width = tracked_object.width;
    object.feature.roi.height = tracked_object.height;
    object.object.existence_probability = tracked_object.score;
    object.object.classification.emplace_back(
      autoware_auto_perception_msgs::build<Label>().label(tracked_object.type).probability(1.0f));

    out_objects.feature_objects.push_back(object);

    auto tracked_uuid = tracked_object.unique_id;
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::memcpy(uuid_msg.uuid.data(), &tracked_uuid, tracked_uuid.size());
    tier4_perception_msgs::msg::DynamicObject dynamic_obj;
    dynamic_obj.id = uuid_msg;
    out_objects_uuid.objects.push_back(dynamic_obj);
  }

  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);

  out_objects_uuid.header = msg->header;
  objects_uuid_pub_->publish(out_objects_uuid);
}
}  // namespace bytetrack

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack::ByteTrackNode)
