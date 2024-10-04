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

#pragma once

#include "combine_cloud_handler.hpp"

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>

namespace autoware::pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent;
class CombineCloudHandler;

class CloudCollector
{
public:
  CloudCollector(
    std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> && ros2_parent_node,
    std::shared_ptr<CombineCloudHandler> & combine_cloud_handler, int num_of_clouds, double time);

  void set_reference_timestamp(double timestamp, double noise_window);
  std::tuple<double, double> get_reference_timestamp_boundary();
  void process_pointcloud(
    const std::string & topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void concatenate_callback();

  ConcatenatedCloudResult concatenate_pointclouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map);
  // void delete_collector();

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
  get_topic_to_cloud_map();

private:
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> ros2_parent_node_;
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map_;
  uint64_t num_of_clouds_;
  double timeout_sec_;
  double reference_timestamp_min_{0.0};
  double reference_timestamp_max_{0.0};
};

}  // namespace autoware::pointcloud_preprocessor
