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
//
//

#ifndef MULTI_OBJECT_TRACKER__PROCESSOR__PROCESSOR_HPP_
#define MULTI_OBJECT_TRACKER__PROCESSOR__PROCESSOR_HPP_

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class TrackerProcessor
{
public:
  explicit TrackerProcessor(const std::map<std::uint8_t, std::string> & tracker_map)
  : tracker_map_(tracker_map)
  {
  }

  const std::list<std::shared_ptr<Tracker>> & getListTracker() const { return list_tracker_; }
  // tracker processes
  void predict(const rclcpp::Time & time);
  void update(
    const autoware_auto_perception_msgs::msg::DetectedObjects & transformed_objects,
    const geometry_msgs::msg::Transform & self_transform,
    const std::unordered_map<int, int> & direct_assignment);
  void spawn(
    const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects,
    const geometry_msgs::msg::Transform & self_transform,
    const std::unordered_map<int, int> & reverse_assignment);
  void checkTrackerLifeCycle(const rclcpp::Time & time);
  void sanitizeTracker(const rclcpp::Time & time);

private:
  std::map<std::uint8_t, std::string> tracker_map_;
  std::list<std::shared_ptr<Tracker>> list_tracker_;

  std::shared_ptr<Tracker> createNewTracker(
    const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) const;
};

#endif  // MULTI_OBJECT_TRACKER__PROCESSOR__PROCESSOR_HPP_
