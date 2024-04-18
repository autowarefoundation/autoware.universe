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

#ifndef MULTI_OBJECT_TRACKER__DEBUGGER__DEBUG_OBJECT_HPP_
#define MULTI_OBJECT_TRACKER__DEBUGGER__DEBUG_OBJECT_HPP_

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include "unique_identifier_msgs/msg/uuid.hpp"
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct ObjectData
{
  rclcpp::Time time;

  // object uuid
  unique_identifier_msgs::msg::UUID uuid;

  // association link, pair of coordinates
  // tracker to detection
  geometry_msgs::msg::Point tracker_point;
  geometry_msgs::msg::Point detection_point;

  // existence probabilities
  std::vector<float> existence_vector;

  // detection channel id
  uint channel_id;
};

class TrackerObjectDebugger
{
public:
  explicit TrackerObjectDebugger(std::string frame_id);

private:
  bool is_initialized_{false};
  std::string frame_id_;
  visualization_msgs::msg::MarkerArray markers_;
  std::unordered_set<int> current_ids_;
  std::unordered_set<int> previous_ids_;
  rclcpp::Time message_time_;

  std::unordered_map<int, ObjectData> object_data_;
  std::unordered_map<boost::uuids::uuid, int32_t, boost::hash<boost::uuids::uuid>> id_map_;
  std::list<int32_t> unused_marker_ids_;
  int32_t marker_id_ = 0;

public:
  void collect(
    const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
    const uint & channel_index,
    const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects,
    const std::unordered_map<int, int> & direct_assignment,
    const std::unordered_map<int, int> & reverse_assignment);

  void reset();
  void draw();
  void getMessage(visualization_msgs::msg::MarkerArray & message) const;

private:
  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & u) const
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
    }
    return ss.str();
  }

  boost::uuids::uuid to_boost_uuid(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    const std::string uuid_str = uuid_to_string(uuid_msg);
    boost::uuids::string_generator gen;
    boost::uuids::uuid uuid = gen(uuid_str);
    return uuid;
  }

  void update_id_map(const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & msg)
  {
    std::vector<boost::uuids::uuid> new_uuids;
    std::vector<boost::uuids::uuid> tracked_uuids;
    new_uuids.reserve(msg->objects.size());
    tracked_uuids.reserve(msg->objects.size());
    for (const auto & object : msg->objects) {
      const auto uuid = to_boost_uuid(object.object_id);
      ((id_map_.find(uuid) != id_map_.end()) ? tracked_uuids : new_uuids).push_back(uuid);
    }

    auto itr = id_map_.begin();
    while (itr != id_map_.end()) {
      if (
        std::find(tracked_uuids.begin(), tracked_uuids.end(), itr->first) == tracked_uuids.end()) {
        unused_marker_ids_.push_back(itr->second);
        itr = id_map_.erase(itr);
      } else {
        ++itr;
      }
    }

    for (const auto & new_uuid : new_uuids) {
      if (unused_marker_ids_.empty()) {
        id_map_.emplace(new_uuid, marker_id_);
        marker_id_++;
      } else {
        id_map_.emplace(new_uuid, unused_marker_ids_.front());
        unused_marker_ids_.pop_front();
      }
    }
  }

  int32_t uuid_to_marker_id(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    auto uuid = to_boost_uuid(uuid_msg);
    return id_map_.at(uuid);
  }
};

#endif  // MULTI_OBJECT_TRACKER__DEBUGGER__DEBUG_OBJECT_HPP_
