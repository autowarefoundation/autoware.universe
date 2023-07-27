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

#include "tracking_object_merger/decorative_tracker_merger.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "tracking_object_merger/data_association/solver/successive_shortest_path.hpp"
#include "tracking_object_merger/utils/utils.hpp"

#include <boost/optional.hpp>

#include <chrono>
#include <unordered_map>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

namespace tracking_object_merger
{

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

// get unix time from header
double getUnixTime(const std_msgs::msg::Header & header)
{
  return header.stamp.sec + header.stamp.nanosec * 1e-9;
}

DecorativeTrackerMergerNode::DecorativeTrackerMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("decorative_object_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  // Subscriber
  sub_main_objects_ = create_subscription<TrackedObjects>(
    "input/main_object", rclcpp::QoS{1},
    std::bind(&DecorativeTrackerMergerNode::mainObjectsCallback, this, std::placeholders::_1));
  sub_sub_objects_ = create_subscription<TrackedObjects>(
    "input/sub_object", rclcpp::QoS{1},
    std::bind(&DecorativeTrackerMergerNode::subObjectsCallback, this, std::placeholders::_1));

  // merged object publisher
  merged_object_pub_ = create_publisher<TrackedObjects>("output/object", rclcpp::QoS{1});

  // logging
  logging_.enable = declare_parameter<bool>("enable_logging", false);
  logging_.path =
    declare_parameter<std::string>("logging_file_path", "~/.ros/association_log.json");

  // Parameters
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id", "base_link");
  time_sync_threshold_ = declare_parameter<double>("time_sync_threshold", 0.05);
  sub_object_timeout_sec_ = declare_parameter<double>("sub_object_timeout_sec", 0.15);

  // Merger policy parameters
  merger_policy_params_.kinematics_to_be_merged =
    declare_parameter<std::string>("kinematics_to_be_merged", "velocity");

  std::string kinematics_merge_policy =
    declare_parameter<std::string>("kinematics_merge_policy", "overwrite");
  std::string classification_merge_policy =
    declare_parameter<std::string>("classification_merge_policy", "skip");
  std::string existence_prob_merge_policy =
    declare_parameter<std::string>("existence_prob_merge_policy", "skip");
  std::string shape_merge_policy = declare_parameter<std::string>("shape_merge_policy", "skip");

  // str to map
  merger_policy_params_.kinematics_merge_policy = merger_policy_map_[kinematics_merge_policy];
  merger_policy_params_.classification_merge_policy =
    merger_policy_map_[classification_merge_policy];
  merger_policy_params_.existence_prob_merge_policy =
    merger_policy_map_[existence_prob_merge_policy];
  merger_policy_params_.shape_merge_policy = merger_policy_map_[shape_merge_policy];

  // init association
  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);
}

/**
 * @brief callback function for main objects
 *
 * @param main_objects
 * @note if there are no sub objects, publish main objects as it is
 *       else, merge main objects and sub objects
 */
void DecorativeTrackerMergerNode::mainObjectsCallback(
  const TrackedObjects::ConstSharedPtr & main_objects)
{
  // if there are no sub objects, publish main objects as it is
  if (sub_objects_buffer_.empty()) {
    merged_object_pub_->publish(*main_objects);
    return;
  }

  // else, merge main objects and sub objects
  const auto merged_objects = decorativeMerger(main_objects);

  // publish merged objects
  merged_object_pub_->publish(merged_objects);
}

/**
 * @brief callback function for sub objects
 *
 * @param msg
 * @note push back sub objects to buffer and remove old sub objects
 */
void DecorativeTrackerMergerNode::subObjectsCallback(const TrackedObjects::ConstSharedPtr & msg)
{
  sub_objects_buffer_.push_back(msg);
  // remove old sub objects
  // const auto now = get_clock()->now();
  const auto now = rclcpp::Time(msg->header.stamp);
  const auto remove_itr = std::remove_if(
    sub_objects_buffer_.begin(), sub_objects_buffer_.end(), [now, this](const auto & sub_object) {
      return (now - rclcpp::Time(sub_object->header.stamp)).seconds() > sub_object_timeout_sec_;
    });
  sub_objects_buffer_.erase(remove_itr, sub_objects_buffer_.end());
}

/**
 * @brief merge main objects and sub objects
 *
 * @param main_objects
 * @return TrackedObjects
 *
 * @note 1. interpolate sub objects to sync main objects
 *       2. do association
 *       3. merge objects
 */
TrackedObjects DecorativeTrackerMergerNode::decorativeMerger(
  const TrackedObjects::ConstSharedPtr & main_objects)
{
  // get interpolated sub objects
  // get newest sub objects which timestamp is earlier to main objects
  TrackedObjects::ConstSharedPtr closest_time_sub_objects;
  TrackedObjects::ConstSharedPtr closest_time_sub_objects_later;
  for (const auto & sub_object : sub_objects_buffer_) {
    if (getUnixTime(sub_object->header) < getUnixTime(main_objects->header)) {
      closest_time_sub_objects = sub_object;
    } else {
      closest_time_sub_objects_later = sub_object;
      break;
    }
  }
  const auto interpolated_sub_objects = interpolateObjectState(
    closest_time_sub_objects, closest_time_sub_objects_later, main_objects->header);

  // define output object
  TrackedObjects output_objects;
  output_objects.header = main_objects->header;

  // if there are no sub objects, return main objects as it is
  if (!interpolated_sub_objects) {
    return *main_objects;
  }

  // else, merge main objects and sub objects
  // first, do association
  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  const auto & objects0 = main_objects->objects;
  const auto & objects1 = interpolated_sub_objects->objects;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    *interpolated_sub_objects, *main_objects, logging_.enable, logging_.path);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  for (size_t object0_idx = 0; object0_idx < objects0.size(); ++object0_idx) {
    const auto & object0 = objects0.at(object0_idx);
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found and merge
      const auto & object1 = objects1.at(direct_assignment.at(object0_idx));
      // merge object0 and object1
      // with each merge policy defined in merger_policy_params_
      TrackedObject merged_object = object0;
      merged_object.kinematics = merger_utils::objectKinematicsVXMerger(
        object0, object1, merger_policy_params_.kinematics_merge_policy);
      // merged_object.shape =
      //   merger_utils::shapeMerger(object0, object1, merger_policy_params_.shape_merge_policy);
      // merged_object.existence_probability = merger_utils::probabilityMerger(
      //   object0.existence_probability, object1.existence_probability,
      //   merger_policy_params_.existence_prob_merge_policy);
      // auto merged_classification = merger_utils::objectClassificationMerger(
      //   object0, object1, merger_policy_params_.classification_merge_policy);
      // merged_object.classification = merged_classification.classification;
      output_objects.objects.push_back(merged_object);
    } else {  // not found
      output_objects.objects.push_back(object0);
    }
  }
  for (size_t object1_idx = 0; object1_idx < objects1.size(); ++object1_idx) {
    const auto & object1 = objects1.at(object1_idx);
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_objects.objects.push_back(object1);
    }
  }

  return output_objects;
}

/**
 * @brief interpolate sub objects to sync main objects
 *
 * @param former_msg
 * @param latter_msg
 * @param output_header
 * @return std::optional<TrackedObjects>
 *
 * @note 1. if both msg is nullptr, return null optional
 *       2. if former_msg is nullptr, return latter_msg
 *       3. if latter_msg is nullptr, return former_msg
 *       4. if both msg is not nullptr, do the interpolation
 */
std::optional<TrackedObjects> DecorativeTrackerMergerNode::interpolateObjectState(
  const TrackedObjects::ConstSharedPtr & former_msg,
  const TrackedObjects::ConstSharedPtr & latter_msg, const std_msgs::msg::Header & output_header)
{
  // Assumption: output_header must be newer than former_msg and older than latter_msg
  // There three possible patterns
  // 1. both msg is nullptr
  // 2. former_msg is nullptr
  // 3. latter_msg is nullptr
  // 4. both msg is not nullptr

  // 1. both msg is nullptr
  if (former_msg == nullptr && latter_msg == nullptr) {
    // return null optional
    return std::nullopt;
  }

  // 2. former_msg is nullptr
  if (former_msg == nullptr) {
    // depends on header stamp difference
    if (
      (rclcpp::Time(latter_msg->header.stamp) - rclcpp::Time(output_header.stamp)).seconds() >
      time_sync_threshold_) {
      // do nothing
      return std::nullopt;
    } else {  // else, return latter_msg
      return *latter_msg;
    }

    // 3. latter_msg is nullptr
  } else if (latter_msg == nullptr) {
    // depends on header stamp difference
    if (
      (rclcpp::Time(output_header.stamp) - rclcpp::Time(former_msg->header.stamp)).seconds() >
      time_sync_threshold_) {
      // do nothing
      return std::nullopt;
    } else {
      // else, return former_msg
      return *former_msg;
      // (TODO) do prediction in here
    }

    // 4. both msg is not nullptr
  } else {
    // do the interpolation
    TrackedObjects interpolated_msg =
      utils::interpolateTrackedObjects(*former_msg, *latter_msg, output_header);
    return interpolated_msg;
  }
}

}  // namespace tracking_object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tracking_object_merger::DecorativeTrackerMergerNode)
