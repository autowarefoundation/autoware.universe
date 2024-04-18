// Copyright 2024 Tier IV, Inc.
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

#include "multi_object_tracker/debugger/debug_object.hpp"

#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"

#include <boost/uuid/uuid.hpp>

#include <functional>
#include <string>

namespace
{
int uuid_to_int(const unique_identifier_msgs::msg::UUID & uuid)
{
  // Convert UUID to string
  std::string uuid_str;
  for (auto byte : uuid.uuid) {
    uuid_str += std::to_string(byte);
  }

  // Hash the string to get an int
  std::hash<std::string> hasher;
  int hashed_uuid = hasher(uuid_str);

  return hashed_uuid;
}

}  // namespace

TrackerObjectDebugger::TrackerObjectDebugger(std::string frame_id)
{
  // set frame id
  frame_id_ = frame_id;

  // initialize markers
  markers_.markers.clear();
  current_ids_.clear();
  previous_ids_.clear();
  message_time_ = rclcpp::Time(0, 0);
}

void TrackerObjectDebugger::reset()
{
  previous_ids_.clear();
  previous_ids_ = current_ids_;
  current_ids_.clear();
  markers_.markers.clear();
}

void TrackerObjectDebugger::collect(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const uint & channel_index,
  const autoware_auto_perception_msgs::msg::DetectedObjects & /*detected_objects*/,
  const std::unordered_map<int, int> & /*direct_assignment*/,
  const std::unordered_map<int, int> & /*reverse_assignment*/)
{
  constexpr int PALETTE_SIZE = 32;
  constexpr std::array<std::array<double, 3>, PALETTE_SIZE> color_array = {{
    {{1.0, 0.0, 0.0}},    {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}},  // Red, Green, Blue
    {{1.0, 1.0, 0.0}},    {{0.0, 1.0, 1.0}},
    {{1.0, 0.0, 1.0}},  // Yellow, Cyan, Magenta
    {{1.0, 0.64, 0.0}},   {{0.75, 1.0, 0.0}},
    {{0.0, 0.5, 0.5}},  // Orange, Lime, Teal
    {{0.5, 0.0, 0.5}},    {{1.0, 0.75, 0.8}},
    {{0.65, 0.17, 0.17}},  // Purple, Pink, Brown
    {{0.5, 0.0, 0.0}},    {{0.5, 0.5, 0.0}},
    {{0.0, 0.0, 0.5}},  // Maroon, Olive, Navy
    {{0.5, 0.5, 0.5}},    {{1.0, 0.4, 0.4}},
    {{0.4, 1.0, 0.4}},  // Grey, Light Red, Light Green
    {{0.4, 0.4, 1.0}},    {{1.0, 1.0, 0.4}},
    {{0.4, 1.0, 1.0}},  // Light Blue, Light Yellow, Light Cyan
    {{1.0, 0.4, 1.0}},    {{1.0, 0.7, 0.4}},
    {{0.7, 0.4, 1.0}},  // Light Magenta, Light Orange, Light Purple
    {{1.0, 0.6, 0.8}},    {{0.71, 0.4, 0.12}},
    {{0.55, 0.0, 0.0}},  // Light Pink, Light Brown, Dark Red
    {{0.0, 0.4, 0.0}},    {{0.0, 0.0, 0.55}},
    {{0.55, 0.55, 0.0}},                       // Dark Green, Dark Blue, Dark Yellow
    {{0.0, 0.55, 0.55}},  {{0.55, 0.0, 0.55}}  // Dark Cyan, Dark Magenta
  }};

  message_time_ = message_time;
  if (!is_initialized_) is_initialized_ = true;

  for (const auto & tracker : list_tracker) {
    // get object
    autoware_auto_perception_msgs::msg::TrackedObject tracked_object;
    tracker->getTrackedObject(message_time_, tracked_object);
    const unique_identifier_msgs::msg::UUID uuid = tracked_object.object_id;
    const int uuid_int = uuid_to_int(uuid);
    current_ids_.emplace(uuid_int);

    // get existence probability

    // get color - by channel index
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = color_array[channel_index % PALETTE_SIZE][0];
    color.g = color_array[channel_index % PALETTE_SIZE][1];
    color.b = color_array[channel_index % PALETTE_SIZE][2];

    // get marker - box

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = message_time_;
    marker.ns = "boxes";
    marker.id = uuid_int;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = tracked_object.kinematics.pose_with_covariance.pose.position.x;
    marker.pose.position.y = tracked_object.kinematics.pose_with_covariance.pose.position.y;
    marker.pose.position.z = tracked_object.kinematics.pose_with_covariance.pose.position.z;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    markers_.markers.push_back(marker);

    // get marker - existence probability text
    std::vector<float> existence_vector;
    tracker->getExistenceProbabilityVector(existence_vector);
    std::string existence_probability_text = "P: ";
    for (const auto & existence_probability : existence_vector) {
      // probability to text, two digits of percentage
      existence_probability_text +=
        std::to_string(static_cast<int>(existence_probability * 100)) + " ";
    }

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id_;
    text_marker.header.stamp = message_time_;
    text_marker.ns = "existence_probability";
    text_marker.id = uuid_int;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = tracked_object.kinematics.pose_with_covariance.pose.position.x;
    text_marker.pose.position.y = tracked_object.kinematics.pose_with_covariance.pose.position.y;
    text_marker.pose.position.z =
      tracked_object.kinematics.pose_with_covariance.pose.position.z + 0.8;
    text_marker.scale.z = 0.8;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = existence_probability_text;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    markers_.markers.push_back(text_marker);

    // get marker - association link lines
  }
}

void TrackerObjectDebugger::draw()
{
  // constexpr int PALETTE_SIZE = 32;
  // constexpr std::array<std::array<double, 3>, PALETTE_SIZE> color_array = {{
  //   {{1.0, 0.0, 0.0}},    {{0.0, 1.0, 0.0}},
  //   {{0.0, 0.0, 1.0}},  // Red, Green, Blue
  //   {{1.0, 1.0, 0.0}},    {{0.0, 1.0, 1.0}},
  //   {{1.0, 0.0, 1.0}},  // Yellow, Cyan, Magenta
  //   {{1.0, 0.64, 0.0}},   {{0.75, 1.0, 0.0}},
  //   {{0.0, 0.5, 0.5}},  // Orange, Lime, Teal
  //   {{0.5, 0.0, 0.5}},    {{1.0, 0.75, 0.8}},
  //   {{0.65, 0.17, 0.17}},  // Purple, Pink, Brown
  //   {{0.5, 0.0, 0.0}},    {{0.5, 0.5, 0.0}},
  //   {{0.0, 0.0, 0.5}},  // Maroon, Olive, Navy
  //   {{0.5, 0.5, 0.5}},    {{1.0, 0.4, 0.4}},
  //   {{0.4, 1.0, 0.4}},  // Grey, Light Red, Light Green
  //   {{0.4, 0.4, 1.0}},    {{1.0, 1.0, 0.4}},
  //   {{0.4, 1.0, 1.0}},  // Light Blue, Light Yellow, Light Cyan
  //   {{1.0, 0.4, 1.0}},    {{1.0, 0.7, 0.4}},
  //   {{0.7, 0.4, 1.0}},  // Light Magenta, Light Orange, Light Purple
  //   {{1.0, 0.6, 0.8}},    {{0.71, 0.4, 0.12}},
  //   {{0.55, 0.0, 0.0}},  // Light Pink, Light Brown, Dark Red
  //   {{0.0, 0.4, 0.0}},    {{0.0, 0.0, 0.55}},
  //   {{0.55, 0.55, 0.0}},                       // Dark Green, Dark Blue, Dark Yellow
  //   {{0.0, 0.55, 0.55}},  {{0.55, 0.0, 0.55}}  // Dark Cyan, Dark Magenta
  // }};

  // do nothing for now
  return;
}

void TrackerObjectDebugger::getMessage(visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (!is_initialized_) return;

  // draw markers

  // fill in marker array
  for (auto & marker : markers_.markers) {
    marker_array.markers.push_back(marker);
  }

  // remove old markers
  for (const auto & previous_id : previous_ids_) {
    if (current_ids_.find(previous_id) != current_ids_.end()) {
      continue;
    }

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = message_time_;
    delete_marker.ns = "boxes";
    delete_marker.id = previous_id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.push_back(delete_marker);

    delete_marker.ns = "existence_probability";
    marker_array.markers.push_back(delete_marker);
  }

  return;
}
