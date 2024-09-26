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

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware_point_types/types.hpp"
#include "cloud_collector.hpp"
#include "combine_cloud_handler.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
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

using autoware_point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;

class PointCloudConcatenateDataSynchronizerComponent : public rclcpp::Node
{
public:
  explicit PointCloudConcatenateDataSynchronizerComponent(const rclcpp::NodeOptions & node_options);
  virtual ~PointCloudConcatenateDataSynchronizerComponent() {}
  void publishClouds(
    ConcatenatedCloudResult concatenated_cloud_result, double reference_timestamp_min,
    double reference_timestamp_max);

private:
  struct Parameters
  {
    bool has_static_tf_only;
    int maximum_queue_size;
    double timeout_sec;
    bool is_motion_compensated;
    bool publish_synchronized_pointcloud;
    bool keep_input_frame_in_synchronized_pointcloud;
    bool publish_previous_but_late_pointcloud;
    std::string synchronized_pointcloud_postfix;
    std::string input_twist_topic_type;
    std::vector<std::string> input_topics;
    std::string output_frame;
    std::vector<double> lidar_timestamp_offsets;
    std::vector<double> lidar_timestamp_noise_window;
  } params_;

  double current_concatenate_cloud_timestamp_{0.0};
  double latest_concatenate_cloud_timestamp_{0.0};
  bool drop_previous_but_late_pointcloud_{false};
  bool publish_pointcloud_{false};
  double diagnostic_reference_timestamp_min_{0.0};
  double diagnostic_reference_timestamp_max_{0.0};
  std::unordered_map<std::string, double> diagnostic_topic_to_original_stamp_map_;

  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  std::shared_ptr<CloudCollector> cloud_collector_;
  std::list<std::shared_ptr<CloudCollector>> cloud_collectors_;
  std::mutex cloud_collectors_mutex_;
  std::unordered_map<std::string, double> topic_to_offset_map_;
  std::unordered_map<std::string, double> topic_to_noise_window_map_;

  // default postfix name for synchronized pointcloud
  static constexpr const char * default_sync_topic_postfix_ = "_synchronized";

  // subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr concatenated_cloud_publisher_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
    topic_to_transformed_cloud_publisher_map_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;

  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  diagnostic_updater::Updater diagnostic_updater_{this};

  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, const std::string & topic_name);
  void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr input);

  std::string formatTimestamp(double timestamp);
  void checkConcatStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::string replaceSyncTopicNamePostfix(
    const std::string & original_topic_name, const std::string & postfix);
  void convertToXYZIRCCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
    sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr);
};

}  // namespace autoware::pointcloud_preprocessor
