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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_

#include "combine_cloud_handler.hpp"

#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent;
class CombineCloudHandler;

class CloudCollector
{
public:
  CloudCollector(
    std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node,
    std::list<std::shared_ptr<CloudCollector>> & collectors,
    std::shared_ptr<CombineCloudHandler> combine_cloud_handler, int num_of_clouds, double time);

  void setReferenceTimeStamp(double timestamp, double noise_window);
  std::tuple<double, double> getReferenceTimeStampBoundary();
  void processCloud(std::string topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void concatenateCallback();
  std::tuple<
    sensor_msgs::msg::PointCloud2::SharedPtr,
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>,
    std::unordered_map<std::string, double>>
  concatenateClouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map);

  void publishClouds(
    sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr,
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
      topic_to_transformed_cloud_map,
    std::unordered_map<std::string, double> topic_to_original_stamp_map);

  void deleteCollector();

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> getTopicToCloudMap();

private:
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node_;
  std::list<std::shared_ptr<CloudCollector>> & collectors_;
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map_;
  uint64_t num_of_clouds_;
  double timeout_sec_;
  double reference_timestamp_min_;
  double reference_timestamp_max_;
  std::mutex mutex_;
};

}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_  // NOLINT
// clang-format on
