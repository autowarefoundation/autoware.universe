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
  // maintain previous ids
  previous_ids_.clear();
  previous_ids_ = current_ids_;
  current_ids_.clear();

  // clear markers, object data list
  object_data_list_.clear();
  markers_.markers.clear();
}

void TrackerObjectDebugger::collect(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const uint & channel_index,
  const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects,
  const std::unordered_map<int, int> & direct_assignment,
  const std::unordered_map<int, int> & /*reverse_assignment*/)
{
  if (!is_initialized_) is_initialized_ = true;

  message_time_ = message_time;

  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker.begin(); tracker_itr != list_tracker.end();
       ++tracker_itr, ++tracker_idx) {
    ObjectData object_data;
    object_data.time = message_time;
    object_data.channel_id = channel_index;

    autoware_auto_perception_msgs::msg::TrackedObject tracked_object;
    (*(tracker_itr))->getTrackedObject(message_time, tracked_object);
    object_data.uuid = to_boost_uuid(tracked_object.object_id);

    // tracker
    bool is_associated = false;
    geometry_msgs::msg::Point tracker_point, detection_point;
    tracker_point.x = tracked_object.kinematics.pose_with_covariance.pose.position.x;
    tracker_point.y = tracked_object.kinematics.pose_with_covariance.pose.position.y;
    tracker_point.z = tracked_object.kinematics.pose_with_covariance.pose.position.z;

    // detection
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      detection_point.x = associated_object.kinematics.pose_with_covariance.pose.position.x;
      detection_point.y = associated_object.kinematics.pose_with_covariance.pose.position.y;
      detection_point.z = associated_object.kinematics.pose_with_covariance.pose.position.z;
      is_associated = true;
    }

    object_data.tracker_point = tracker_point;
    object_data.detection_point = detection_point;
    object_data.is_associated = is_associated;

    // existence probabilities
    std::vector<float> existence_vector;
    (*(tracker_itr))->getExistenceProbabilityVector(existence_vector);
    object_data.existence_vector = existence_vector;

    object_data_list_.push_back(object_data);
  }

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
}

void TrackerObjectDebugger::draw(
  const std::vector<std::vector<ObjectData>> object_data_groups,
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  // initialize markers
  marker_array.markers.clear();

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

  for (const auto & object_data_group : object_data_groups) {
    if (object_data_group.empty()) continue;
    const int & uuid_int = object_data_group.front().uuid_int;
    const auto object_data_front = object_data_group.front();
    const auto object_data_back = object_data_group.back();
    // set a reference marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = object_data_front.time;
    marker.id = uuid_int;
    marker.pose.position.x = object_data_front.tracker_point.x;
    marker.pose.position.y = object_data_front.tracker_point.y;
    marker.pose.position.z = object_data_front.tracker_point.z;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;  // white
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    // get marker - existence_probability
    visualization_msgs::msg::Marker text_marker;
    text_marker = marker;
    text_marker.ns = "existence_probability";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.z += 1.5;
    text_marker.scale.z = 0.8;

    // show the last existence probability
    std::string existence_probability_text = "P: ";
    for (const auto & existence_probability : object_data_front.existence_vector) {
      // probability to text, two digits of percentage
      existence_probability_text +=
        std::to_string(static_cast<int>(existence_probability * 100)) + " ";
    }
    text_marker.text = existence_probability_text;
    marker_array.markers.push_back(text_marker);

    // loop for each object_data in the group
    // boxed to tracker positions
    // and link lines to the detected positions
    visualization_msgs::msg::Marker marker_boxes;
    marker_boxes = marker;
    marker_boxes.ns = "boxes";
    marker_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_boxes.action = visualization_msgs::msg::Marker::ADD;

    for (const auto & object_data : object_data_group) {
      // get color - by channel index
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0;
      color.r = color_array[object_data.channel_id % PALETTE_SIZE][0];
      color.g = color_array[object_data.channel_id % PALETTE_SIZE][1];
      color.b = color_array[object_data.channel_id % PALETTE_SIZE][2];
      marker_boxes.color = color;
    }
  }

  return;
}

void TrackerObjectDebugger::getMessage(visualization_msgs::msg::MarkerArray & marker_array)
{
  if (!is_initialized_) return;

  // check all object data and update marker index
  update_id_map(object_data_list_);

  // update uuid_int
  for (auto & object_data : object_data_list_) {
    object_data.uuid_int = uuid_to_marker_id(object_data.uuid);
  }

  // sort by uuid, collect the same uuid object_data as a group, and loop for the groups
  std::vector<std::vector<ObjectData>> object_data_groups;
  {
    // sort by uuid_int
    std::sort(
      object_data_list_.begin(), object_data_list_.end(),
      [](const ObjectData & a, const ObjectData & b) { return a.uuid_int < b.uuid_int; });

    // collect the same uuid object_data as a group
    std::vector<ObjectData> object_data_group;
    boost::uuids::uuid previous_uuid = object_data_list_.front().uuid;
    for (const auto & object_data : object_data_list_) {
      if (object_data.uuid != previous_uuid) {
        object_data_groups.push_back(object_data_group);
        object_data_group.clear();
        previous_uuid = object_data.uuid;
      }
      object_data_group.push_back(object_data);
    }

    // fill the vector of groups
    object_data_groups.push_back(object_data_group);
  }

  // draw markers
  draw(object_data_groups, marker_array);

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
