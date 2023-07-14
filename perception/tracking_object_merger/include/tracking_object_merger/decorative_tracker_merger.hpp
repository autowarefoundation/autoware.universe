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

#ifndef TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_HPP_
#define TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_HPP_

#include "tracking_object_merger/data_association/data_association.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tracking_object_merger
{

class DecorativeTrackerMergerNode : public rclcpp::Node
{
public:
  explicit DecorativeTrackerMergerNode(const rclcpp::NodeOptions & node_options);
  enum class PriorityMode : int { Object0 = 0, Object1 = 1, Confidence = 2 };

private:
  void mainObjectsCallback(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg);
  void subObjectsCallback(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg);

  autoware_auto_perception_msgs::msg::TrackedObjects predictFutureState(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg,
    const std_msgs::msg::Header & header);
  autoware_auto_perception_msgs::msg::TrackedObjects interpolateState(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg1,
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & input_object_msg2,
    const std_msgs::msg::Header & header);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    merged_object_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    sub_main_objects_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    sub_sub_objects_;

  std::unique_ptr<DataAssociation> data_association_;
  std::string target_frame_;
  std::string base_link_frame_id_;
  // buffer to save the sub objects
  std::vector<autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr>
    sub_objects_buffer_;
  double sub_object_timeout_sec_;

  struct
  {
    double precision_threshold;
    double recall_threshold;
    double generalized_iou_threshold;
    std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  } overlapped_judge_param_;
};

}  // namespace tracking_object_merger

#endif  // TRACKING_OBJECT_MERGER__DECORATIVE_TRACKER_MERGER_HPP_
