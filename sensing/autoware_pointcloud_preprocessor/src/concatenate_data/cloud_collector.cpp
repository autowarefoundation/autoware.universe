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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"

#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tuple>

namespace autoware::pointcloud_preprocessor
{

CloudCollector::CloudCollector(
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> && ros2_parent_node,
  std::list<std::shared_ptr<CloudCollector>> & collectors,
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler, int num_of_clouds, double timeout_sec)
: ros2_parent_node_(std::move(ros2_parent_node)),
  collectors_(collectors),
  combine_cloud_handler_(combine_cloud_handler),
  num_of_clouds_(num_of_clouds),
  timeout_sec_(timeout_sec)
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec_));

  timer_ = rclcpp::create_timer(
    ros2_parent_node_, ros2_parent_node_->get_clock(), period_ns,
    std::bind(&CloudCollector::concatenate_callback, this));
}

void CloudCollector::set_reference_timestamp(double timestamp, double noise_window)
{
  reference_timestamp_max_ = timestamp + noise_window;
  reference_timestamp_min_ = timestamp - noise_window;
}

std::tuple<double, double> CloudCollector::get_reference_timestamp_boundary()
{
  return std::make_tuple(reference_timestamp_min_, reference_timestamp_max_);
}

void CloudCollector::process_pointcloud(
  const std::string & topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  // Check if the map already contains an entry for the same topic. This shouldn't happen if the
  // parameter 'lidar_timestamp_noise_window' is set correctly.
  if (topic_to_cloud_map_.find(topic_name) != topic_to_cloud_map_.end()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      ros2_parent_node_->get_logger(), *ros2_parent_node_->get_clock(),
      std::chrono::milliseconds(10000).count(),
      "Topic '" << topic_name
                << "' already exists in the collector. Check the timestamp of the pointcloud.");
  }
  topic_to_cloud_map_[topic_name] = cloud;
  if (topic_to_cloud_map_.size() == num_of_clouds_) {
    concatenate_callback();
  }
}

void CloudCollector::concatenate_callback()
{
  // All pointclouds are received or the timer has timed out, cancel the timer and concatenate the
  // pointclouds in the collector.
  timer_->cancel();

  // lock for protecting collector list and concatenated pointcloud
  std::lock_guard<std::mutex> lock(mutex_);
  auto concatenated_cloud_result = concatenate_pointclouds(topic_to_cloud_map_);
  ros2_parent_node_->publish_clouds(
    concatenated_cloud_result, reference_timestamp_min_, reference_timestamp_max_);
  delete_collector();
}

ConcatenatedCloudResult CloudCollector::concatenate_pointclouds(
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map)
{
  return combine_cloud_handler_->combine_pointclouds(topic_to_cloud_map);
}

void CloudCollector::delete_collector()
{
  auto it = std::find_if(
    collectors_.begin(), collectors_.end(),
    [this](const std::shared_ptr<CloudCollector> & collector) { return collector.get() == this; });
  if (it != collectors_.end()) {
    collectors_.erase(it);
  }
}

std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
CloudCollector::get_topic_to_cloud_map()
{
  return topic_to_cloud_map_;
}

}  // namespace autoware::pointcloud_preprocessor
