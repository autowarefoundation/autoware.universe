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

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#define DEFAULT_SYNC_TOPIC_POSTFIX \
  "_synchronized"  // default postfix name for synchronized pointcloud

namespace autoware::pointcloud_preprocessor
{

PointCloudConcatenateDataSynchronizerComponent::PointCloudConcatenateDataSynchronizerComponent(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_concatenator_component", node_options)
{
  // initialize debug tool
  using autoware::universe_utils::DebugPublisher;
  using autoware::universe_utils::StopWatch;
  stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<DebugPublisher>(this, "concatenate_data_synchronizer");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  //  initialize parameters
  params_.has_static_tf_only = declare_parameter<bool>("has_static_tf_only");
  params_.maximum_queue_size = declare_parameter<int>("maximum_queue_size");
  params_.timeout_sec = declare_parameter<double>("timeout_sec");
  params_.is_motion_compensated = declare_parameter<bool>("is_motion_compensated");
  params_.publish_synchronized_pointcloud =
    declare_parameter<bool>("publish_synchronized_pointcloud");
  params_.keep_input_frame_in_synchronized_pointcloud =
    declare_parameter<bool>("keep_input_frame_in_synchronized_pointcloud");
  params_.publish_previous_but_late_pointcloud =
    declare_parameter<bool>("publish_previous_but_late_pointcloud");
  params_.synchronized_pointcloud_postfix =
    declare_parameter<std::string>("synchronized_pointcloud_postfix");
  params_.input_twist_topic_type = declare_parameter<std::string>("input_twist_topic_type");
  params_.input_topics = declare_parameter<std::vector<std::string>>("input_topics");
  params_.output_frame = declare_parameter<std::string>("output_frame");
  params_.lidar_timestamp_offsets =
    declare_parameter<std::vector<double>>("lidar_timestamp_offsets");
  params_.lidar_timestamp_noise_window =
    declare_parameter<std::vector<double>>("lidar_timestamp_noise_window");

  if (params_.input_topics.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
    return;
  } else if (params_.input_topics.size() == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  if (params_.output_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
    return;
  }
  if (params_.lidar_timestamp_offsets.size() != params_.input_topics.size()) {
    RCLCPP_ERROR(
      get_logger(), "The number of topics does not match the number of timestamp offsets");
    return;
  }
  if (params_.lidar_timestamp_noise_window.size() != params_.input_topics.size()) {
    RCLCPP_ERROR(
      get_logger(), "The number of topics does not match the number of timestamp noise window");
    return;
  }

  for (size_t i = 0; i < params_.input_topics.size(); i++) {
    topic_to_offset_map_[params_.input_topics[i]] = params_.lidar_timestamp_offsets[i];
    topic_to_noise_window_map_[params_.input_topics[i]] = params_.lidar_timestamp_noise_window[i];
  }

  // Publishers
  concatenated_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));

  // Transformed Raw PointCloud2 Publisher to publish the transformed pointcloud
  if (params_.publish_synchronized_pointcloud) {
    for (auto & topic : params_.input_topics) {
      std::string new_topic =
        replaceSyncTopicNamePostfix(topic, params_.synchronized_pointcloud_postfix);
      auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        new_topic, rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
      topic_to_transformed_cloud_publisher_map_.insert({topic, publisher});
    }
  }

  // Subscribers
  if (params_.is_motion_compensated) {
    if (params_.input_twist_topic_type == "twist") {
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/input/twist", rclcpp::QoS{100},
        std::bind(
          &PointCloudConcatenateDataSynchronizerComponent::twist_callback, this,
          std::placeholders::_1));
    } else if (params_.input_twist_topic_type == "odom") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/odom", rclcpp::QoS{100},
        std::bind(
          &PointCloudConcatenateDataSynchronizerComponent::odom_callback, this,
          std::placeholders::_1));
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(), "input_twist_topic_type is invalid: " << params_.input_twist_topic_type);
      throw std::runtime_error(
        "input_twist_topic_type is invalid: " + params_.input_twist_topic_type);
    }
  }

  pointcloud_subs.resize(params_.input_topics.size());
  for (size_t topic_id = 0; topic_id < params_.input_topics.size(); ++topic_id) {
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> callback = std::bind(
      &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this, std::placeholders::_1,
      params_.input_topics[topic_id]);

    pointcloud_subs[topic_id].reset();
    pointcloud_subs[topic_id] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topics[topic_id], rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size),
      callback);
  }
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Subscribing to " << params_.input_topics.size() << " user given topics as inputs:");
  for (const auto & input_topic : params_.input_topics) {
    RCLCPP_DEBUG_STREAM(get_logger(), " - " << input_topic);
  }

  // Combine cloud handler
  combine_cloud_handler_ = std::make_shared<CombineCloudHandler>(
    this, params_.input_topics, params_.output_frame, params_.is_motion_compensated,
    params_.keep_input_frame_in_synchronized_pointcloud, params_.has_static_tf_only);

  // Diagnostic Updater
  diagnostic_updater_.setHardwareID("concatenate_data_checker");
  diagnostic_updater_.add(
    "concat_status", this, &PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus);
}

std::string PointCloudConcatenateDataSynchronizerComponent::replaceSyncTopicNamePostfix(
  const std::string & original_topic_name, const std::string & postfix)
{
  std::string replaced_topic_name;
  // separate the topic name by '/' and replace the last element with the new postfix
  size_t pos = original_topic_name.find_last_of("/");
  if (pos == std::string::npos) {
    // not found '/': this is not a namespaced topic
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The topic name is not namespaced. The postfix will be added to the end of the topic name.");
    return original_topic_name + postfix;
  } else {
    // replace the last element with the new postfix
    replaced_topic_name = original_topic_name.substr(0, pos) + "/" + postfix;
  }

  // if topic name is the same with original topic name, add postfix to the end of the topic name
  if (replaced_topic_name == original_topic_name) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The topic name "
                      << original_topic_name
                      << " have the same postfix with synchronized pointcloud. We use "
                         "the postfix "
                         "to the end of the topic name.");
    replaced_topic_name = original_topic_name + DEFAULT_SYNC_TOPIC_POSTFIX;
  }
  return replaced_topic_name;
}

void PointCloudConcatenateDataSynchronizerComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, const std::string & topic_name)
{
  stop_watch_ptr_->toc("processing_time", true);
  if (!utils::is_data_layout_compatible_with_point_xyzirc(*input_ptr)) {
    RCLCPP_ERROR(
      get_logger(), "The pointcloud layout is not compatible with PointXYZIRC. Aborting");

    if (utils::is_data_layout_compatible_with_point_xyzi(*input_ptr)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy code/data");
    }

    return;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr xyzirc_input_ptr(new sensor_msgs::msg::PointCloud2());
  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);
  if (input->data.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
    return;
  } else {
    // convert to XYZIRC pointcloud if pointcloud is not empty
    convertToXYZIRCCloud(input, xyzirc_input_ptr);
  }

  // protect cloud collectors list
  std::unique_lock<std::mutex> cloud_collectors_lock(cloud_collectors_mutex_);

  // For each callback, check whether there is a exist collector that matches this cloud
  bool collector_found = false;

  if (!cloud_collectors_.empty()) {
    for (const auto & cloud_collector : cloud_collectors_) {
      auto [reference_timestamp_min, reference_timestamp_max] =
        cloud_collector->getReferenceTimeStampBoundary();

      if (
        rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name] <
          reference_timestamp_max + topic_to_noise_window_map_[topic_name] &&
        rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name] >
          reference_timestamp_min - topic_to_noise_window_map_[topic_name]) {
        cloud_collectors_lock.unlock();
        cloud_collector->processCloud(topic_name, input_ptr);
        collector_found = true;
        break;
      }
    }
  }

  // if point cloud didn't find matched collector, create a new collector.
  if (!collector_found) {
    auto new_cloud_collector = std::make_shared<CloudCollector>(
      std::dynamic_pointer_cast<PointCloudConcatenateDataSynchronizerComponent>(shared_from_this()),
      cloud_collectors_, combine_cloud_handler_, params_.input_topics.size(), params_.timeout_sec);

    cloud_collectors_.push_back(new_cloud_collector);
    cloud_collectors_lock.unlock();
    new_cloud_collector->setReferenceTimeStamp(
      rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name],
      topic_to_noise_window_map_[topic_name]);
    new_cloud_collector->processCloud(topic_name, input_ptr);
  }
}

void PointCloudConcatenateDataSynchronizerComponent::twist_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input)
{
  combine_cloud_handler_->processTwist(input);
}

void PointCloudConcatenateDataSynchronizerComponent::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input)
{
  combine_cloud_handler_->processOdometry(input);
}

void PointCloudConcatenateDataSynchronizerComponent::publishClouds(
  sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr,
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> &
    topic_to_transformed_cloud_map,
  std::unordered_map<std::string, double> & topic_to_original_stamp_map,
  double reference_timestamp_min, double reference_timestamp_max)
{
  // should never come to this state.
  if (concatenate_cloud_ptr == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Concatenate cloud is a nullptr.");
    return;
  }
  current_concatenate_cloud_timestamp_ =
    rclcpp::Time(concatenate_cloud_ptr->header.stamp).seconds();

  if (
    current_concatenate_cloud_timestamp_ < latest_concatenate_cloud_timestamp_ &&
    !params_.publish_previous_but_late_pointcloud) {
    drop_previous_but_late_pointcloud_ = true;
  } else {
    publish_pointcloud_ = true;
    latest_concatenate_cloud_timestamp_ = current_concatenate_cloud_timestamp_;
    auto concatenate_pointcloud_output =
      std::make_unique<sensor_msgs::msg::PointCloud2>(*concatenate_cloud_ptr);
    concatenated_cloud_publisher_->publish(std::move(concatenate_pointcloud_output));

    // publish transformed raw pointclouds
    if (params_.publish_synchronized_pointcloud) {
      for (auto topic : params_.input_topics) {
        if (topic_to_transformed_cloud_map.find(topic) != topic_to_transformed_cloud_map.end()) {
          auto transformed_cloud_output =
            std::make_unique<sensor_msgs::msg::PointCloud2>(*topic_to_transformed_cloud_map[topic]);
          topic_to_transformed_cloud_publisher_map_[topic]->publish(
            std::move(transformed_cloud_output));
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "transformed_raw_points[%s] is nullptr, skipping pointcloud publish.", topic.c_str());
        }
      }
    }
  }

  diagnostic_reference_timestamp_min_ = reference_timestamp_min;
  diagnostic_reference_timestamp_max_ = reference_timestamp_max;
  diagnostic_topic_to_original_stamp_map_ = topic_to_original_stamp_map;
  diagnostic_updater_.force_update();

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    for (const auto & [topic, transformed_cloud] : topic_to_transformed_cloud_map) {
      if (transformed_cloud != nullptr) {
        const auto pipeline_latency_ms =
          std::chrono::duration<double, std::milli>(
            std::chrono::nanoseconds(
              (this->get_clock()->now() - transformed_cloud->header.stamp).nanoseconds()))
            .count();
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug" + topic + "/pipeline_latency_ms", pipeline_latency_ms);
      }
    }
  }
}

void PointCloudConcatenateDataSynchronizerComponent::convertToXYZIRCCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr)
{
  output_ptr->header = input_ptr->header;

  PointCloud2Modifier<PointXYZIRC, autoware_point_types::PointXYZIRCGenerator> output_modifier{
    *output_ptr, input_ptr->header.frame_id};
  output_modifier.reserve(input_ptr->width);

  bool has_valid_intensity =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_return_type =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "return_type" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_channel =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "channel" && field.datatype == sensor_msgs::msg::PointField::UINT16;
    });

  sensor_msgs::PointCloud2Iterator<float> it_x(*input_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*input_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*input_ptr, "z");

  if (has_valid_intensity && has_valid_return_type && has_valid_channel) {
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_i(*input_ptr, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_r(*input_ptr, "return_type");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> it_c(*input_ptr, "channel");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      point.return_type = *it_r;
      point.channel = *it_c;
      output_modifier.push_back(std::move(point));
    }
  } else {
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      output_modifier.push_back(std::move(point));
    }
  }
}

std::string PointCloudConcatenateDataSynchronizerComponent::formatTimestamp(double timestamp)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << timestamp;
  return oss.str();
}

void PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (publish_pointcloud_ || drop_previous_but_late_pointcloud_) {
    stat.add("concatenated cloud timestamp", formatTimestamp(current_concatenate_cloud_timestamp_));
    stat.add("reference timestamp min", formatTimestamp(diagnostic_reference_timestamp_min_));
    stat.add("reference timestamp max", formatTimestamp(diagnostic_reference_timestamp_max_));

    bool topic_miss = false;

    int concatenate_status = 1;
    for (auto topic : params_.input_topics) {
      int cloud_status;  // 1 for success, 0 for failure
      if (
        diagnostic_topic_to_original_stamp_map_.find(topic) !=
        diagnostic_topic_to_original_stamp_map_.end()) {
        cloud_status = 1;
        stat.add(
          topic + " timestamp", formatTimestamp(diagnostic_topic_to_original_stamp_map_[topic]));
      } else {
        topic_miss = true;
        cloud_status = 0;
        concatenate_status = 0;
      }
      stat.add(topic, cloud_status);
    }

    stat.add("concatenate status", concatenate_status);

    int8_t level;
    std::string message;
    if (topic_miss) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Concatenated pointcloud is published but miss some topics";
    } else if (drop_previous_but_late_pointcloud_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Concatenated pointcloud is not published as it is too late";
    } else {
      level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      message = "Concatenated pointcloud is published and include all topics";
    }

    stat.summary(level, message);

    publish_pointcloud_ = false;
    drop_previous_but_late_pointcloud_ = false;
  } else {
    const int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    const std::string message =
      "Concatenate node launch successfully, but waiting for input pointcloud";
    stat.summary(level, message);
  }
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
