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

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_pointclouds.hpp"

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// postfix for input topics
#define POSTFIX_NAME "_synchronized"

//////////////////////////////////////////////////////////////////////////////////////////////

namespace autoware::pointcloud_preprocessor
{
PointCloudConcatenationComponent::PointCloudConcatenationComponent(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_concatenator_component", node_options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "concatenate_pointclouds_debug");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Set parameters
  {
    output_frame_ = declare_parameter<std::string>("output_frame");
    if (output_frame_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
      return;
    }
    has_static_tf_only_ = declare_parameter<bool>(
      "has_static_tf_only", false);  // TODO(amadeuszsz): remove default value
    declare_parameter<std::vector<std::string>>("input_topics");
    input_topics_ = get_parameter("input_topics").as_string_array();
    if (input_topics_.empty()) {
      RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
      return;
    }
    if (input_topics_.size() == 1) {
      RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
      return;
    }

    // Optional parameters
    maximum_queue_size_ = declare_parameter<int64_t>("max_queue_size");
    /** input pointclouds should be */
    timeout_sec_ = declare_parameter<double>("timeout_sec");

    input_offset_ = declare_parameter<std::vector<double>>("input_offset");
    if (!input_offset_.empty() && input_topics_.size() != input_offset_.size()) {
      RCLCPP_ERROR(get_logger(), "The number of topics does not match the number of offsets.");
      return;
    }
  }

  // Initialize not_subscribed_topic_names_
  {
    for (const std::string & e : input_topics_) {
      not_subscribed_topic_names_.insert(e);
    }
  }

  // Initialize offset map
  {
    for (size_t i = 0; i < input_offset_.size(); ++i) {
      offset_map_[input_topics_[i]] = input_offset_[i];
    }
  }

  // tf2 listener
  {
    managed_tf_buffer_ =
      std::make_unique<autoware::universe_utils::ManagedTransformBuffer>(this, has_static_tf_only_);
  }

  // Output Publishers
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), pub_options);
  }

  // Subscribers
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "Subscribing to " << input_topics_.size() << " user given topics as inputs:");
    for (const auto & input_topic : input_topics_) {
      RCLCPP_INFO_STREAM(get_logger(), " - " << input_topic);
    }

    // Subscribe to the filters
    filters_.resize(input_topics_.size());

    // First input_topics_.size () filters are valid
    for (size_t d = 0; d < input_topics_.size(); ++d) {
      cloud_stdmap_.insert(std::make_pair(input_topics_[d], nullptr));
      cloud_stdmap_tmp_ = cloud_stdmap_;

      // CAN'T use auto type here.
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> cb = std::bind(
        &PointCloudConcatenationComponent::cloud_callback, this, std::placeholders::_1,
        input_topics_[d]);

      filters_[d].reset();
      filters_[d] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[d], rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), cb);
    }
  }

  // Set timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec_));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns,
      std::bind(&PointCloudConcatenationComponent::timer_callback, this));
  }

  // Diagnostic Updater
  {
    updater_.setHardwareID("concatenate_pc_checker");
    updater_.add("concat_status", this, &PointCloudConcatenationComponent::checkConcatStatus);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudConcatenationComponent::checkSyncStatus()
{
  // gather the stamps
  std::vector<rclcpp::Time> pc_stamps;
  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      const auto stamp = rclcpp::Time(e.second->header.stamp);
      auto it = std::find(pc_stamps.begin(), pc_stamps.end(), stamp);
      if (it != pc_stamps.end()) {
        // found
        continue;
      } else {
        // not found
        pc_stamps.push_back(stamp);
      }
    }
  }

  // 1. Check if all stamps are same
  if (pc_stamps.size() == 1) {
    return;
  }
  // else, do the same for the tmp cloud
  for (const auto & e : cloud_stdmap_tmp_) {
    if (e.second != nullptr) {
      const auto stamp = rclcpp::Time(e.second->header.stamp);
      auto it = std::find(pc_stamps.begin(), pc_stamps.end(), stamp);
      if (it != pc_stamps.end()) {
        // found
        continue;
      } else {
        // not found
        pc_stamps.push_back(stamp);
      }
    }
  }
  // sort pc_stamps
  std::sort(pc_stamps.begin(), pc_stamps.end());
  // restrict the size of pc_stamps to newer 2 stamps
  if (pc_stamps.size() > 2) {
    pc_stamps.erase(pc_stamps.begin(), pc_stamps.end() - 2);
  }

  // 2. if the stamp variation is 2, return true and reshape the cloud_stdmap_tmp_
  for (auto & e : cloud_stdmap_) {
    // if the cloud is nullptr, check if the tmp cloud is not nullptr and has the same stamp
    if (e.second == nullptr) {
      // do nothing
    } else {
      // else if cloud is not nullptr
      const auto current_stamp = rclcpp::Time(e.second->header.stamp);
      if (current_stamp == pc_stamps.front()) {
        // if the stamp is the oldest one, do nothing
      } else if (current_stamp == pc_stamps.back()) {
        // if the stamp is the newest one, move the cloud to the tmp cloud
        cloud_stdmap_tmp_[e.first] = e.second;
        e.second = nullptr;
      } else {
        // this state should not be reached. discard data
        e.second = nullptr;
      }
    }
    // check for the tmp cloud
    if (cloud_stdmap_tmp_[e.first] == nullptr) {
      continue;
    }
    const auto next_stamp = rclcpp::Time(cloud_stdmap_tmp_[e.first]->header.stamp);
    if (next_stamp == pc_stamps.front()) {
      e.second = cloud_stdmap_tmp_[e.first];
      cloud_stdmap_tmp_[e.first] = nullptr;
    } else if (next_stamp == pc_stamps.back()) {
      // do nothing
    } else {
      // this state should not be reached. discard data
      cloud_stdmap_tmp_[e.first] = nullptr;
    }
  }
  return;
}

void PointCloudConcatenationComponent::combineClouds(
  sensor_msgs::msg::PointCloud2::SharedPtr & concat_cloud_ptr)
{
  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      // transform to output frame
      sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(
        new sensor_msgs::msg::PointCloud2());
      managed_tf_buffer_->transformPointcloud(output_frame_, *e.second, *transformed_cloud_ptr);

      // concatenate
      if (concat_cloud_ptr == nullptr) {
        concat_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(*transformed_cloud_ptr);
      } else {
        pcl::concatenatePointCloud(*concat_cloud_ptr, *transformed_cloud_ptr, *concat_cloud_ptr);
      }
    } else {
      not_subscribed_topic_names_.insert(e.first);
    }
  }
}

void PointCloudConcatenationComponent::publish()
{
  stop_watch_ptr_->toc("processing_time", true);
  sensor_msgs::msg::PointCloud2::SharedPtr concat_cloud_ptr = nullptr;
  not_subscribed_topic_names_.clear();

  checkSyncStatus();
  combineClouds(concat_cloud_ptr);

  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      if (debug_publisher_) {
        const auto pipeline_latency_ms =
          std::chrono::duration<double, std::milli>(
            std::chrono::nanoseconds(
              (this->get_clock()->now() - e.second->header.stamp).nanoseconds()))
            .count();
        debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
          "debug" + e.first + "/pipeline_latency_ms", pipeline_latency_ms);
      }
    }
  }

  // publish concatenated pointcloud
  if (concat_cloud_ptr) {
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr);
    pub_output_->publish(std::move(output));
  } else {
    RCLCPP_WARN(this->get_logger(), "concat_cloud_ptr is nullptr, skipping pointcloud publish.");
  }

  updater_.force_update();

  // update cloud_stdmap_
  cloud_stdmap_ = cloud_stdmap_tmp_;
  std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
    e.second = nullptr;
  });

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenationComponent::convertToXYZIRCCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr)
{
  output_ptr->header = input_ptr->header;

  PointCloud2Modifier<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator> output_modifier{
    *output_ptr, input_ptr->header.frame_id};
  output_modifier.reserve(input_ptr->width);

  bool has_valid_intensity =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
      return field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_return_type =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
      return field.name == "return_type" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_channel =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](const auto & field) {
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

void PointCloudConcatenationComponent::setPeriod(const int64_t new_period)
{
  if (!timer_) {
    return;
  }
  int64_t old_period = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
  }
  ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
  }
}

void PointCloudConcatenationComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic_name)
{
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

  std::lock_guard<std::mutex> lock(mutex_);
  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);
  sensor_msgs::msg::PointCloud2::SharedPtr xyzirc_input_ptr(new sensor_msgs::msg::PointCloud2());
  convertToXYZIRCCloud(input, xyzirc_input_ptr);

  const bool is_already_subscribed_this = (cloud_stdmap_[topic_name] != nullptr);
  const bool is_already_subscribed_tmp = std::any_of(
    std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),
    [](const auto & e) { return e.second != nullptr; });

  if (is_already_subscribed_this) {
    cloud_stdmap_tmp_[topic_name] = xyzirc_input_ptr;

    if (!is_already_subscribed_tmp) {
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec_));
      try {
        setPeriod(period.count());
      } catch (rclcpp::exceptions::RCLError & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      }
      timer_->reset();
    }
  } else {
    cloud_stdmap_[topic_name] = xyzirc_input_ptr;

    const bool is_subscribed_all = std::all_of(
      std::begin(cloud_stdmap_), std::end(cloud_stdmap_),
      [](const auto & e) { return e.second != nullptr; });

    if (is_subscribed_all) {
      for (const auto & e : cloud_stdmap_tmp_) {
        if (e.second != nullptr) {
          cloud_stdmap_[e.first] = e.second;
        }
      }
      std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
        e.second = nullptr;
      });

      timer_->cancel();
      publish();
    } else if (offset_map_.size() > 0) {
      timer_->cancel();
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec_ - offset_map_[topic_name]));
      try {
        setPeriod(period.count());
      } catch (rclcpp::exceptions::RCLError & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      }
      timer_->reset();
    }
  }
}

void PointCloudConcatenationComponent::timer_callback()
{
  using std::chrono_literals::operator""ms;
  timer_->cancel();
  if (mutex_.try_lock()) {
    publish();
    mutex_.unlock();
  } else {
    try {
      std::chrono::nanoseconds period = 10ms;
      setPeriod(period.count());
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    }
    timer_->reset();
  }
}

void PointCloudConcatenationComponent::checkConcatStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  for (const std::string & e : input_topics_) {
    const std::string subscribe_status = not_subscribed_topic_names_.count(e) ? "NG" : "OK";
    stat.add(e, subscribe_status);
  }

  const int8_t level = not_subscribed_topic_names_.empty()
                         ? diagnostic_msgs::msg::DiagnosticStatus::OK
                         : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  const std::string message = not_subscribed_topic_names_.empty()
                                ? "Concatenate all topics"
                                : "Some topics are not concatenated";
  stat.summary(level, message);
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PointCloudConcatenationComponent)
