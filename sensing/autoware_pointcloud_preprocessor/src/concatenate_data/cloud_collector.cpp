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

#ifdef USE_CUDA
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#endif

#include <tuple>

namespace autoware::pointcloud_preprocessor
{

template <typename PointCloudMessage>
CloudCollector<PointCloudMessage>::CloudCollector(
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> && ros2_parent_node,
  std::shared_ptr<CombineCloudHandler<PointCloudMessage>> & combine_cloud_handler,
  int num_of_clouds, double timeout_sec, bool debug_mode)
: ros2_parent_node_(std::move(ros2_parent_node)),
  combine_cloud_handler_(combine_cloud_handler),
  num_of_clouds_(num_of_clouds),
  timeout_sec_(timeout_sec),
  debug_mode_(debug_mode)
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec_));

  timer_ = rclcpp::create_timer(
    ros2_parent_node_, ros2_parent_node_->get_clock(), period_ns,
    std::bind(&CloudCollector::concatenate_callback, this));
}

template <typename PointCloudMessage>
void CloudCollector<PointCloudMessage>::set_reference_timestamp(
  double timestamp, double noise_window)
{
  reference_timestamp_max_ = timestamp + noise_window;
  reference_timestamp_min_ = timestamp - noise_window;
}

template <typename PointCloudMessage>
std::tuple<double, double> CloudCollector<PointCloudMessage>::get_reference_timestamp_boundary()
{
  return std::make_tuple(reference_timestamp_min_, reference_timestamp_max_);
}

template <typename PointCloudMessage>
void CloudCollector<PointCloudMessage>::process_pointcloud(
  const std::string & topic_name, typename PointCloudMessage::ConstSharedPtr cloud)
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

template <typename PointCloudMessage>
void CloudCollector<PointCloudMessage>::concatenate_callback()
{
  static_assert(
    std::is_same<PointCloudMessage, sensor_msgs::msg::PointCloud2>::value ||
      std::is_same<PointCloudMessage, cuda_blackboard::CudaPointCloud2>::value,
    "This function is only available for sensor_msgs::msg::PointCloud2 and "
    "cuda_blackboard::CudaPointCloud2");
}

template <>
void CloudCollector<sensor_msgs::msg::PointCloud2>::concatenate_callback()
{
  if (debug_mode_) {
    auto time_until_trigger = timer_->time_until_trigger();
    std::stringstream log_stream;
    log_stream << std::fixed << std::setprecision(6);
    log_stream << "Collector's concatenate callback time: "
               << ros2_parent_node_->get_clock()->now().seconds() << " seconds\n";

    log_stream << "Collector's reference time min: " << reference_timestamp_min_
               << " to max: " << reference_timestamp_max_ << " seconds\n";

    log_stream << "Time until trigger: " << (time_until_trigger.count() / 1e9) << " seconds\n";

    log_stream << "Pointclouds: [";
    std::string separator = "";
    for (const auto & [topic, cloud] : topic_to_cloud_map_) {
      log_stream << separator;
      log_stream << "[" << topic << ", " << rclcpp::Time(cloud->header.stamp).seconds() << "]";
      separator = ", ";
    }

    log_stream << "]\n";

    RCLCPP_INFO(ros2_parent_node_->get_logger(), "%s", log_stream.str().c_str());
  }

  // All pointclouds are received or the timer has timed out, cancel the timer and concatenate the
  // pointclouds in the collector.
  timer_->cancel();

  auto concatenated_cloud_result = concatenate_pointclouds(topic_to_cloud_map_);

  ros2_parent_node_->publish_clouds(
    std::move(concatenated_cloud_result), reference_timestamp_min_, reference_timestamp_max_);

  ros2_parent_node_->delete_collector(*this);
}

#ifdef USE_CUDA

template <>
void CloudCollector<cuda_blackboard::CudaPointCloud2>::concatenate_callback()
{
  if (debug_mode_) {
    auto time_until_trigger = timer_->time_until_trigger();
    std::stringstream log_stream;
    log_stream << std::fixed << std::setprecision(6);
    log_stream << "Collector's concatenate callback time: "
               << ros2_parent_node_->get_clock()->now().seconds() << " seconds\n";

    log_stream << "Collector's reference time min: " << reference_timestamp_min_
               << " to max: " << reference_timestamp_max_ << " seconds\n";

    log_stream << "Time until trigger: " << (time_until_trigger.count() / 1e9) << " seconds\n";

    log_stream << "Pointclouds: [";
    std::string separator = "";
    for (const auto & [topic, cloud] : topic_to_cloud_map_) {
      log_stream << separator;
      log_stream << "[" << topic << ", " << rclcpp::Time(cloud->header.stamp).seconds() << "]";
      separator = ", ";
    }

    log_stream << "]\n";

    RCLCPP_INFO(ros2_parent_node_->get_logger(), "%s", log_stream.str().c_str());
  }

  // All pointclouds are received or the timer has timed out, cancel the timer and concatenate the
  // pointclouds in the collector.
  timer_->cancel();

  auto concatenated_cloud_result = concatenate_pointclouds(topic_to_cloud_map_);

  ros2_parent_node_->publish_clouds(
    std::move(concatenated_cloud_result), reference_timestamp_min_, reference_timestamp_max_);

  ros2_parent_node_->delete_collector(*this);
}
#endif

template <typename PointCloudMessage>
ConcatenatedCloudResult<PointCloudMessage>
CloudCollector<PointCloudMessage>::concatenate_pointclouds(
  std::unordered_map<std::string, typename PointCloudMessage::ConstSharedPtr> topic_to_cloud_map)
{
  return combine_cloud_handler_->combine_pointclouds(topic_to_cloud_map);
}

template <typename PointCloudMessage>
std::unordered_map<std::string, typename PointCloudMessage::ConstSharedPtr>
CloudCollector<PointCloudMessage>::get_topic_to_cloud_map()
{
  return topic_to_cloud_map_;
}

}  // namespace autoware::pointcloud_preprocessor

template class autoware::pointcloud_preprocessor::CloudCollector<sensor_msgs::msg::PointCloud2>;

#ifdef USE_CUDA
template class autoware::pointcloud_preprocessor::CloudCollector<cuda_blackboard::CudaPointCloud2>;
#endif
