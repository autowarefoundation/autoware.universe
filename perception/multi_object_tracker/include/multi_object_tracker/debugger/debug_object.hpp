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
#include <visualization_msgs/msg/marker_array.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

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

public:
  void collect(
    const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
    const uint & channel_index,
    const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects,
    const std::unordered_map<int, int> & direct_assignment,
    const std::unordered_map<int, int> & reverse_assignment);

  void reset();

  void getMessage(visualization_msgs::msg::MarkerArray & message);
};

#endif  // MULTI_OBJECT_TRACKER__DEBUGGER__DEBUG_OBJECT_HPP_
