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

#ifndef TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_
#define TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

// measurement state
enum class MEASUREMENT_STATE : int {
  MAIN_AND_SUB = 0,
  MAIN_ONLY = 1,
  SUB_ONLY = 2,
  NONE = 3,
};

class TrackerState
{
private:
  /* data */
  TrackedObject tracked_object_;
  rclcpp::Time last_update_time_;
  // Eigen::MatrixXf state_matrix_;
  // Eigen::MatrixXf covariance_matrix_;

  // handle existence probability
  double can_publish_threshold_ = 0.5;
  double remove_threshold_ = 0.3;
  std::unordered_map<int, double> existence_probability_map_;

  // handle uuid
  std::unordered_map<int, std::optional<unique_identifier_msgs::msg::UUID>> input_uuid_map_;
  const unique_identifier_msgs::msg::UUID const_uuid_;
  MEASUREMENT_STATE measurement_state_;

  // timer handle
  std::unordered_map<int, rclcpp::Time> last_updated_time_map_;
  double max_dt_ = 2.0;

public:
  TrackerState(
    const int input, const rclcpp::Time & last_update_time, const TrackedObject & tracked_object);
  ~TrackerState();

public:
  bool predict(const rclcpp::Time & time);
  bool predict(const double dt, std::function<TrackedObject(const TrackedObject &, double)> func);
  bool update(
    const int input, const rclcpp::Time & current_time, const TrackedObject & tracked_object);
  bool update(const int input, const TrackedObject & tracked_object);
  bool update(
    const int input, const rclcpp::Time & current_time, const TrackedObject & tracked_object,
    std::function<void(TrackedObject &, const TrackedObject &)> update_func);
  bool hasUUID(const int input, const unique_identifier_msgs::msg::UUID & uuid);
  TrackedObject getObject() const;
};

TrackedObjects getTrackedObjectsFromTrackerStates(
  const std::vector<TrackerState> & tracker_states, const rclcpp::Time & time);

#endif  // TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_
