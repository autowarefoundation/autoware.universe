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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware_point_types/types.hpp"

#include <autoware/universe_utils/ros/managed_transform_buffer.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace autoware::pointcloud_preprocessor
{

class CombineCloudHandler
{
private:
  rclcpp::Node * node_;

  std::vector<std::string> input_topics_;
  std::string output_frame_;
  bool is_motion_compensated_;
  bool keep_input_frame_in_synchronized_pointcloud_;
  std::unique_ptr<autoware::universe_utils::ManagedTransformBuffer> managed_tf_buffer_{nullptr};

  struct RclcppTimeHash_
  {
    std::size_t operator()(const rclcpp::Time & t) const
    {
      return std::hash<int64_t>()(t.nanoseconds());
    }
  };

public:
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

  CombineCloudHandler(
    rclcpp::Node * node, std::vector<std::string> input_topics, std::string output_frame,
    bool is_motion_compensated, bool keep_input_frame_in_synchronized_pointcloud,
    bool has_static_tf_only);
  void processTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & input);
  void processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & input);

  std::tuple<
    sensor_msgs::msg::PointCloud2::SharedPtr,
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>,
    std::unordered_map<std::string, double>>
  combinePointClouds(std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> &
                       topic_to_cloud_map_);

  Eigen::Matrix4f computeTransformToAdjustForOldTimestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);
};

}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_  // NOLINT
// clang-format on
