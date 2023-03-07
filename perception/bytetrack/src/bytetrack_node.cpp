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

#include "bytetrack/bytetrack.hpp"
#include <bytetrack/bytetrack_node.hpp>

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <string>
#include <utility>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace bytetrack
{
ByteTrackNode::ByteTrackNode(const rclcpp::NodeOptions & node_options)
    : Node("bytetrack", node_options)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  int track_buffer_length = declare_parameter("track_buffer_length", 30);

  this->bytetrack_ = std::make_unique<bytetrack::ByteTrack>(track_buffer_length);

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&ByteTrackNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "~/out/objects", 1);
  objects_uuid_pub_ = this->create_publisher<tier4_perception_msgs::msg::DynamicObjectArray>(
      "~/out/objects/debug/uuid", 1);}


void ByteTrackNode::onConnect()
{
  using std::placeholders::_1;
  if (objects_pub_->get_subscription_count() == 0 &&
      objects_pub_->get_intra_process_subscription_count() == 0)
  {
    detection_rect_sub_.reset();
  } else if (!detection_rect_sub_) {
    detection_rect_sub_ =
        this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
            "~/in/rect", 1, std::bind(&ByteTrackNode::onRect, this, _1));
  }
}

void ByteTrackNode::onRect(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr
    msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;
  tier4_perception_msgs::msg::DynamicObjectArray out_objects_uuid;

  // cv_bridge::CvImagePtr in_image_ptr;
  // try {
  //   in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // } catch (cv_bridge::Exception & e) {
  //   RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  //   return;
  // }
  // const auto width = in_image_ptr->image.cols;
  // const auto height = in_image_ptr->image.rows;

    // Unpack detection results
  ObjectArray object_array;
  for (auto& feat_obj: msg->feature_objects) {
    Object obj;
    obj.x_offset = feat_obj.feature.roi.x_offset;
    obj.y_offset = feat_obj.feature.roi.y_offset;
    obj.height = feat_obj.feature.roi.height;
    obj.width = feat_obj.feature.roi.width;
    obj.score = feat_obj.object.classification.front().probability;
    obj.type = feat_obj.object.classification.front().label;
    object_array.emplace_back(obj);
  }

  bytetrack::ObjectArray objects = bytetrack_->updateTracker(object_array);
  for (const auto & tracked_object : objects) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = tracked_object.x_offset;
    object.feature.roi.y_offset = tracked_object.y_offset;
    object.feature.roi.width = tracked_object.width;
    object.feature.roi.height = tracked_object.height;
    object.object.existence_probability = tracked_object.score;
    object.object.classification.emplace_back(
        autoware_auto_perception_msgs::build<Label>()
        .label(tracked_object.type)
        .probability(1.0f));

    out_objects.feature_objects.push_back(object);

    // auto track_id = tracked_object.track_id;
    // track_id_and_color.track_id.push_back(track_id);

    auto tracked_uuid = tracked_object.unique_id;
    unique_identifier_msgs::msg::UUID uuid_msg;
    std::memcpy(uuid_msg.uuid.data(), &tracked_uuid, tracked_uuid.size());
    tier4_perception_msgs::msg::DynamicObject dynamic_obj;
    dynamic_obj.id = uuid_msg;
    out_objects_uuid.objects.push_back(dynamic_obj);
    // track_id_and_color.object_id.push_back(uuid_msg);

    // auto color = bytetrack_->getColor(track_id);
    // std_msgs::msg::ColorRGBA color_msg;
    // color_msg.b= static_cast<float>(color[0]) / 255.;  // range [0, 1]
    // color_msg.g= static_cast<float>(color[1]) / 255.;
    // color_msg.r= static_cast<float>(color[2]) / 255.;
    // color_msg.a= 1.0;
    // track_id_and_color.color.push_back(color_msg);

    // const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    // const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    // const auto right =
    //   std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    // const auto bottom =
    //   std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    // cv::rectangle(
    //   in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), color, 3,
    //   8, 0);
    // cv::putText(in_image_ptr->image,
    //             cv::format("ID: %d (%s), type: %d", track_id,
    //                        boost::lexical_cast<std::string>(unique_id).c_str(),
    //                        tracked_object.type),
    //             cv::Point(left, top - 5), 0, 0.6, color, 2, cv::LINE_AA);
  }
  // image_pub_.publish(in_image_ptr->toImageMsg());

  // out_objects.header = in_image_ptr->header;
  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);

  out_objects_uuid.header = msg->header;
  objects_uuid_pub_->publish(out_objects_uuid);

  // track_id_and_color.header = out_objects.header;
  // track_id_pub_->publish(track_id_and_color);
}

// void ByteTrackNode::onRect(
//     const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr
//     msg)
// {
//   // Unpack detection results
//   ObjectArray object_array;
//   for (auto& feat_obj: msg->feature_objects) {
//     Object obj;
//     obj.x_offset = feat_obj.feature.roi.x_offset;
//     obj.y_offset = feat_obj.feature.roi.y_offset;
//     obj.height = feat_obj.feature.roi.height;
//     obj.width = feat_obj.feature.roi.width;
//     obj.score = feat_obj.object.classification.front().probability;
//     obj.type = feat_obj.object.classification.front().label;
//     object_array.emplace_back(obj);
//   }

//   bytetrack_->updateTracker(object_array);
// }

}  // namespace bytetrack

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack::ByteTrackNode)
