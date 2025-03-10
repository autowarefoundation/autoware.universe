// Copyright 2024 TIER IV, Inc.
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

#include "processor.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{

using Label = autoware_perception_msgs::msg::ObjectClassification;
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

TrackerProcessor::TrackerProcessor(
  const TrackerProcessorConfig & config, const AssociatorConfig & associator_config,
  const std::vector<types::InputChannel> & channels_config)
: config_(config), channels_config_(channels_config)
{
  association_ = std::make_unique<DataAssociation>(associator_config);
}

void TrackerProcessor::predict(const rclcpp::Time & time)
{
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(time);
  }
}

void TrackerProcessor::associate(
  const types::DynamicObjectList & detected_objects,
  std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment) const
{
  const auto & tracker_list = list_tracker_;
  // global nearest neighbor
  Eigen::MatrixXd score_matrix = association_->calcScoreMatrix(
    detected_objects, tracker_list);  // row : tracker, col : measurement
  association_->assign(score_matrix, direct_assignment, reverse_assignment);
}

void TrackerProcessor::update(
  const types::DynamicObjectList & detected_objects,
  const geometry_msgs::msg::Transform & self_transform,
  const std::unordered_map<int, int> & direct_assignment)
{
  int tracker_idx = 0;
  const auto & time = detected_objects.header.stamp;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {
      // found
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      const types::InputChannel channel_info = channels_config_[associated_object.channel_index];
      (*(tracker_itr))
        ->updateWithMeasurement(associated_object, time, self_transform, channel_info);

    } else {
      // not found
      (*(tracker_itr))->updateWithoutMeasurement(time);
    }
  }
}

void TrackerProcessor::spawn(
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & reverse_assignment)
{
  const auto channel_config = channels_config_[detected_objects.channel_index];
  // If spawn is disabled, return
  if (!channel_config.is_spawn_enabled) {
    return;
  }

  // Spawn new trackers for the objects that are not associated
  const auto & time = detected_objects.header.stamp;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    const auto & new_object = detected_objects.objects.at(i);
    std::shared_ptr<Tracker> tracker = createNewTracker(new_object, time);

    // Initialize existence probabilities
    tracker->initializeExistenceProbabilities(
      new_object.channel_index, new_object.existence_probability);

    // Update the tracker with the new object
    list_tracker_.push_back(tracker);
  }
}

std::shared_ptr<Tracker> TrackerProcessor::createNewTracker(
  const types::DynamicObject & object, const rclcpp::Time & time) const
{
  const LabelType label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  if (config_.tracker_map.count(label) != 0) {
    const auto tracker = config_.tracker_map.at(label);
    if (tracker == "bicycle_tracker") return std::make_shared<BicycleTracker>(time, object);
    if (tracker == "big_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::big_vehicle, time, object);
    if (tracker == "multi_vehicle_tracker")
      return std::make_shared<MultipleVehicleTracker>(time, object);
    if (tracker == "normal_vehicle_tracker")
      return std::make_shared<VehicleTracker>(object_model::normal_vehicle, time, object);
    if (tracker == "pass_through_tracker")
      return std::make_shared<PassThroughTracker>(time, object);
    if (tracker == "pedestrian_and_bicycle_tracker")
      return std::make_shared<PedestrianAndBicycleTracker>(time, object);
    if (tracker == "pedestrian_tracker") return std::make_shared<PedestrianTracker>(time, object);
  }
  return std::make_shared<UnknownTracker>(time, object);
}

void TrackerProcessor::prune(const rclcpp::Time & time)
{
  // Check tracker lifetime: if the tracker is old, delete it
  removeOldTracker(time);
  // Check tracker overlap: if the tracker is overlapped, delete the one with lower IOU
  removeOverlappedTracker(time);
}

void TrackerProcessor::removeOldTracker(const rclcpp::Time & time)
{
  // Check elapsed time from last update
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    const bool is_old = config_.tracker_lifetime < (*itr)->getElapsedTimeFromLastUpdate(time);
    // If the tracker is old, delete it
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker_.erase(erase_itr);
    }
  }
}

// This function removes overlapped trackers based on distance and IoU criteria
void TrackerProcessor::removeOverlappedTracker(const rclcpp::Time & time)
{
  // Iterate through the list of trackers
  for (auto itr1 = list_tracker_.begin(); itr1 != list_tracker_.end(); ++itr1) {
    types::DynamicObject object1;
    if (!(*itr1)->getTrackedObject(time, object1)) continue;

    // Compare the current tracker with the remaining trackers
    for (auto itr2 = std::next(itr1); itr2 != list_tracker_.end(); ++itr2) {
      types::DynamicObject object2;
      if (!(*itr2)->getTrackedObject(time, object2)) continue;

      // Calculate the distance between the two objects
      const double distance = std::hypot(
        object1.pose.position.x - object2.pose.position.x,
        object1.pose.position.y - object2.pose.position.y);

      // If the distance is too large, skip
      if (distance > config_.distance_threshold) {
        continue;
      }

      // Check the Intersection over Union (IoU) between the two objects
      constexpr double min_union_iou_area = 1e-2;
      const auto iou = shapes::get2dIoU(object1, object2, min_union_iou_area);
      const auto & label1 = (*itr1)->getHighestProbLabel();
      const auto & label2 = (*itr2)->getHighestProbLabel();
      bool should_delete_tracker1 = false;
      bool should_delete_tracker2 = false;

      // If both trackers are UNKNOWN, delete the younger tracker
      // If one side of the tracker is UNKNOWN, delete UNKNOWN objects
      if (label1 == Label::UNKNOWN || label2 == Label::UNKNOWN) {
        if (iou > config_.min_unknown_object_removal_iou) {
          if (label1 == Label::UNKNOWN && label2 == Label::UNKNOWN) {
            if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          } else if (label1 == Label::UNKNOWN) {
            should_delete_tracker1 = true;
          } else if (label2 == Label::UNKNOWN) {
            should_delete_tracker2 = true;
          }
        }
      } else {  // If neither object is UNKNOWN, delete the younger tracker
        if (iou > config_.min_known_object_removal_iou) {
          if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
            should_delete_tracker1 = true;
          } else {
            should_delete_tracker2 = true;
          }
        }
      }

      // Delete the tracker
      if (should_delete_tracker1) {
        itr1 = list_tracker_.erase(itr1);
        --itr1;
        break;
      }
      if (should_delete_tracker2) {
        itr2 = list_tracker_.erase(itr2);
        --itr2;
      }
    }
  }
}

bool TrackerProcessor::isConfidentTracker(const std::shared_ptr<Tracker> & tracker) const
{
  // Confidence is determined by counting the number of measurements.
  // If the number of measurements is equal to or greater than the threshold, the tracker is
  // considered confident.
  auto label = tracker->getHighestProbLabel();
  return tracker->getTotalMeasurementCount() >= config_.confident_count_threshold.at(label);
}

void TrackerProcessor::getTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const
{
  tracked_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    // Skip if the tracker is not confident
    if (!isConfidentTracker(tracker)) continue;
    // Get the tracked object, extrapolated to the given time
    if (tracker->getTrackedObject(time, tracked_object)) {
      tracked_objects.objects.push_back(toTrackedObjectMsg(tracked_object));
    }
  }
}

void TrackerProcessor::getTentativeObjects(
  const rclcpp::Time & time,
  autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  tentative_objects.header.stamp = time;
  types::DynamicObject tracked_object;
  for (const auto & tracker : list_tracker_) {
    if (!isConfidentTracker(tracker)) {
      if (tracker->getTrackedObject(time, tracked_object)) {
        tentative_objects.objects.push_back(toTrackedObjectMsg(tracked_object));
      }
    }
  }
}

}  // namespace autoware::multi_object_tracker
