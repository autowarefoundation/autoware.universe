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

#ifndef AUTOWARE__TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_NODE_HPP_
#define AUTOWARE__TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_NODE_HPP_

#include "autoware/tracking_object_merger/association/data_association.hpp"
#include "autoware/tracking_object_merger/utils/tracker_state.hpp"
#include "autoware/tracking_object_merger/utils/utils.hpp"
#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/ros/published_time_publisher.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <std_msgs/msg/header.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tracking_object_merger
{

class DecorativeTrackerMergerNode : public rclcpp::Node
{
public:
  explicit DecorativeTrackerMergerNode(const rclcpp::NodeOptions & node_options);
  enum class PriorityMode : int { Object0 = 0, Object1 = 1, Confidence = 2 };

private:
  void set3dDataAssociation(
    const std::string & prefix,
    std::unordered_map<std::string, std::unique_ptr<DataAssociation>> & data_association_map);

  void mainObjectsCallback(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & main_objects);
  void subObjectsCallback(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & msg);

  bool decorativeMerger(
    const MEASUREMENT_STATE input_sensor,
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_objects_msg);
  autoware_perception_msgs::msg::TrackedObjects predictFutureState(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg,
    const std_msgs::msg::Header & header);
  std::optional<autoware_perception_msgs::msg::TrackedObjects> interpolateObjectState(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & former_msg,
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & latter_msg,
    const std_msgs::msg::Header & output_header);
  TrackedObjects getTrackedObjects(const std_msgs::msg::Header & header);
  TrackerState createNewTracker(
    const MEASUREMENT_STATE input_index, rclcpp::Time current_time,
    const autoware_perception_msgs::msg::TrackedObject & input_object);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr merged_object_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr sub_main_objects_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr sub_sub_objects_;
  // debug object publisher
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr debug_object_pub_;
  bool publish_interpolated_sub_objects_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> processing_time_publisher_;

  /* handle objects */
  std::unordered_map<MEASUREMENT_STATE, std::function<void(TrackedObject &, const TrackedObject &)>>
    input_merger_map_;
  MEASUREMENT_STATE main_sensor_type_;
  MEASUREMENT_STATE sub_sensor_type_;
  std::vector<TrackerState> inner_tracker_objects_;
  std::unordered_map<std::string, std::unique_ptr<DataAssociation>> data_association_map_;
  std::string base_link_frame_id_;
  std::string merge_frame_id_;
  // buffer to save the sub objects
  std::vector<autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr> sub_objects_buffer_;
  double sub_object_timeout_sec_;
  double time_sync_threshold_;

  // tracker default settings
  TrackerStateParameter tracker_state_parameter_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  // merge policy (currently not used)
  struct
  {
    std::string kinematics_to_be_merged;
    merger_utils::MergePolicy kinematics_merge_policy;
    merger_utils::MergePolicy classification_merge_policy;
    merger_utils::MergePolicy existence_prob_merge_policy;
    merger_utils::MergePolicy shape_merge_policy;
  } merger_policy_params_;

  std::map<std::string, merger_utils::MergePolicy> merger_policy_map_ = {
    {"skip", merger_utils::MergePolicy::SKIP},
    {"overwrite", merger_utils::MergePolicy::OVERWRITE},
    {"fusion", merger_utils::MergePolicy::FUSION}};

  // debug parameters
  struct logging
  {
    bool enable = false;
    std::string path;
  } logging_;
};

}  // namespace autoware::tracking_object_merger

#endif  // AUTOWARE__TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_NODE_HPP_
