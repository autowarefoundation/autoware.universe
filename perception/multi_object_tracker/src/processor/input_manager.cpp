// Copyright 2024 Tier IV, Inc.
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

#include "multi_object_tracker/processor/input_manager.hpp"
#include <cassert>

InputStream::InputStream(rclcpp::Node & node) : node_(node)
{
}

void InputStream::init(
  const std::string & input_topic, const std::string & long_name, const std::string & short_name)
{
  // Initialize parameters
  input_topic_ = input_topic;
  long_name_ = long_name;
  short_name_ = short_name;

  // debug message
  RCLCPP_INFO(
    node_.get_logger(), "Initializing %s input stream from %s", long_name_.c_str(),
    input_topic_.c_str());

  // Initialize subscription
  std::function<void(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)>
    func = std::bind(&InputStream::setObjects, this, std::placeholders::_1);
  sub_objects_ = node_.create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    input_topic_, rclcpp::QoS{1}, func);

  // Initialize queue
  objects_que_.clear();

  // Initialize latency statistics
  expected_rate_ = 10.0;
  latency_mean_ = 0.10;  // [s]
  latency_var_ = 0.0;
  interval_mean_ = 1 / expected_rate_;
  interval_var_ = 0.0;
}

void InputStream::setObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  // debug message
  RCLCPP_INFO(
    node_.get_logger(), "Received %s message from %s", long_name_.c_str(), input_topic_.c_str());

  const autoware_auto_perception_msgs::msg::DetectedObjects object = *msg;
  objects_que_.push_back(object);
  if (objects_que_.size() > que_size_) {
    objects_que_.pop_front();
  }

  // Filter parameters
  constexpr double gain = 0.05;
  const auto now = node_.now();

  // Calculate interval, Update interval statistics
  if (is_time_initialized_) {
    const double interval = (now - latest_message_time_).seconds();
    interval_mean_ = (1.0 - gain) * interval_mean_ + gain * interval;
    const double interval_delta = interval - interval_mean_;
    interval_var_ = (1.0 - gain) * interval_var_ + gain * interval_delta * interval_delta;
  }

  // Update time
  latest_message_time_ = now;
  latest_measurement_time_ = object.header.stamp;
  if (!is_time_initialized_) is_time_initialized_ = true;

  // Calculate latency
  const double latency = (latest_message_time_ - latest_measurement_time_).seconds();

  // Update latency statistics
  latency_mean_ = (1.0 - gain) * latency_mean_ + gain * latency;
  const double latency_delta = latency - latency_mean_;
  latency_var_ = (1.0 - gain) * latency_var_ + gain * latency_delta * latency_delta;
}

void InputStream::getObjectsOlderThan(
  const rclcpp::Time & time, const double duration,
  std::vector<autoware_auto_perception_msgs::msg::DetectedObjects> & objects)
{
  objects.clear();
  for (const auto & object : objects_que_) {
    // Skip if the object is older than the specified duration
    const rclcpp::Time object_time = rclcpp::Time(object.header.stamp);
    const double time_diff = (object_time - time).seconds();
    if (time_diff > duration) {
      continue;
    }
    // Add the object if the object is older than the specified time
    if (time_diff >= 0.0) {
      objects.push_back(object);
    }
  }
}

InputManager::InputManager(rclcpp::Node & node) : node_(node)
{
}

void InputManager::init(
  const std::vector<std::string> & input_topics, const std::vector<std::string> & long_names, const std::vector<std::string> & short_names)
{
  input_size_ = input_topics.size();
  RCLCPP_INFO(node_.get_logger(), "Initializing input manager with %zu input streams", input_size_);
  assert(input_size_ == long_names.size());
  assert(input_size_ == short_names.size());

  for (size_t i = 0; i < input_size_; i++) {
    InputStream input_stream(node_);
    input_stream.init(input_topics.at(i), long_names.at(i), short_names.at(i));
    input_streams_.push_back(std::make_shared<InputStream>(input_stream));
  }
}

bool InputManager::getObjects(
  const rclcpp::Time & now,
  std::vector<autoware_auto_perception_msgs::msg::DetectedObjects> & objects)
{
  objects.clear();

  // Get proper latency
  constexpr double target_latency = 0.15;   // [s], measurement to tracking latency, as much as the
                                            // detector latency, the less total latency
  constexpr double acceptable_band = 0.15;  // [s], acceptable band from the target latency
  rclcpp::Time time = now - rclcpp::Duration::from_seconds(target_latency);

  // Get objects from all input streams
  for (const auto & input_stream : input_streams_) {
    std::vector<autoware_auto_perception_msgs::msg::DetectedObjects> objects_tmp;
    input_stream->getObjectsOlderThan(time, acceptable_band, objects_tmp);
    objects.insert(objects.end(), objects_tmp.begin(), objects_tmp.end());
  }

  // Sort objects by timestamp
  std::sort(
    objects.begin(), objects.end(),
    [](
      const autoware_auto_perception_msgs::msg::DetectedObjects & a,
      const autoware_auto_perception_msgs::msg::DetectedObjects & b) {
      return (rclcpp::Time(a.header.stamp) - rclcpp::Time(b.header.stamp)).seconds() < 0;
    });

  return !objects.empty();
}
