// Copyright 2020 Tier IV, Inc.
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
// Author: v1.0 Yukihiro Saito
//

#include "autoware/multi_object_tracker/tracker/model/pedestrian_and_bicycle_tracker.hpp"

namespace autoware::multi_object_tracker
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

PedestrianAndBicycleTracker::PedestrianAndBicycleTracker(
  const rclcpp::Time & time, const types::DynamicObject & object, const size_t channel_size)
: Tracker(time, object.classification, channel_size),
  pedestrian_tracker_(time, object, channel_size),
  bicycle_tracker_(time, object, channel_size)
{
  // initialize existence probability
  initializeExistenceProbabilities(object.channel_index, object.existence_probability);
}

bool PedestrianAndBicycleTracker::predict(const rclcpp::Time & time)
{
  pedestrian_tracker_.predict(time);
  bicycle_tracker_.predict(time);
  return true;
}

bool PedestrianAndBicycleTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  pedestrian_tracker_.measure(object, time, self_transform);
  bicycle_tracker_.measure(object, time, self_transform);
  if (
    autoware::object_recognition_utils::getHighestProbLabel(object.classification) !=
    Label::UNKNOWN)
    updateClassification(object.classification);
  return true;
}

bool PedestrianAndBicycleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object) const
{
  using Label = autoware_perception_msgs::msg::ObjectClassification;
  const uint8_t label = getHighestProbLabel();

  if (label == Label::PEDESTRIAN) {
    pedestrian_tracker_.getTrackedObject(time, object);
  } else if (label == Label::BICYCLE || label == Label::MOTORCYCLE) {
    bicycle_tracker_.getTrackedObject(time, object);
  }
  object.object_id = getUUID();
  object.classification = getClassification();
  return true;
}

}  // namespace autoware::multi_object_tracker
