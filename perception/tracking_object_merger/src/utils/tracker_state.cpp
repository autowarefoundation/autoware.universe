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

#include "tracking_object_merger/utils/tracker_state.hpp"

#include "tracking_object_merger/utils/utils.hpp"

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

/**
 * @brief Construct a new Tracker State:: Tracker State object
 *
 * @param input_source : input source distinguisher
 * @param tracked_object : input source tracked object
 * @param last_update_time : last update time
 */
TrackerState::TrackerState(
  const int input_source, const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object,
  const rclcpp::Time & last_update_time)
: tracked_object_(tracked_object), last_update_time_(last_update_time)
{
  input_uuid_map_[input_source] = tracked_object_.object_id;
  last_updated_time_map_[input_source] = last_update_time;
}

/**
 * @brief Predict state to current time
 *
 * @param current_time
 * @return true
 * @return false
 */
bool TrackerState::predict(const rclcpp::Time & current_time)
{
  // get dt and give warning if dt is too large
  double dt = (current_time - last_update_time_).seconds();
  if (std::abs(dt) > max_dt_) {
    std::cerr << "[tracking_object_merger] dt is too large: " << dt << std::endl;
    return false;
  }

  // do prediction
  last_update_time_ = current_time;
  return this->predict(dt, utils::predictPastOrFutureTrackedObject);
}

/**
 * @brief Predict state to current time with given update function
 *
 * @param dt : time to predict
 * @param func : update function (e.g. PredictPastOrFutureTrackedObject)
 * @return true: prediction successed
 * @return false: prediction failed
 */
bool TrackerState::predict(
  const double dt, std::function<TrackedObject(const TrackedObject &, double)> func)
{
  const auto predicted_object = func(tracked_object_, dt);
  tracked_object_ = predicted_object;
  return true;
}

bool TrackerState::update(
  const int input, const rclcpp::Time & current_time, const TrackedObject & tracked_object)
{
  // calc dt
  double dt = (current_time - last_update_time_).seconds();
  if (dt < 0.0) {
    std::cerr << "[tracking_object_merger] dt is negative: " << dt << std::endl;
    return false;
  }

  // predict
  if (dt > 0.0) {
    this->predict(dt, utils::predictPastOrFutureTrackedObject);
  }

  // merge update
  // update with suitable function
  this->update(input, current_time, tracked_object, merger_utils::updateWholeTrackedObject);
  return true;
}

bool TrackerState::update(
  const int input, const rclcpp::Time & current_time, const TrackedObject & input_tracked_object,
  std::function<void(TrackedObject &, const TrackedObject &)> update_func)
{
  // put input uuid and last update time
  last_update_time_ = current_time;
  last_updated_time_map_[input] = current_time;
  input_uuid_map_[input] = input_tracked_object.object_id;

  // update tracked object
  update_func(tracked_object_, input_tracked_object);

  return true;
}

TrackedObject TrackerState::getObject() const
{
  return tracked_object_;
}

bool TrackerState::hasUUID(const int input, const unique_identifier_msgs::msg::UUID & uuid)
{
  if (input_uuid_map_.find(input) == input_uuid_map_.end()) {
    return false;
  }
  return input_uuid_map_.at(input) == uuid;
}

TrackedObjects getTrackedObjectsFromTrackerStates(
  const std::vector<TrackerState> & tracker_states, const rclcpp::Time & current_time)
{
  TrackedObjects tracked_objects;
  for (const auto & tracker_state : tracker_states) {
    tracked_objects.objects.push_back(tracker_state.getObject());
  }
  // TODO(yoshiri): do prediction if object last input state is older than current time

  // update header
  tracked_objects.header.stamp = current_time;
  tracked_objects.header.frame_id = "map";  // TODO(yoshiri): get frame_id from input
  return tracked_objects;
}
