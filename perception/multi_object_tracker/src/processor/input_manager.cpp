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

InputManager::InputManager(rclcpp::Node & node) : node_(node)
{
}

void InputManager::init(
  const std::string & input_topic, const std::string & long_name, const std::string & short_name)
{
  // Initialize parameters
  input_topic_ = input_topic;
  long_name_ = long_name;
  short_name_ = short_name;

  // Initialize subscription
  std::function<void(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)>
    func = std::bind(&InputManager::setObjects, this, std::placeholders::_1);
  sub_objects_ = node_.create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    input_topic_, rclcpp::QoS{1}, func);

  // Initialize latency statistics
  expected_rate_ = 10.0;
  latency_mean_ = 0.10;  // [s]
  latency_var_ = 0.0;
  interval_mean_ = 1 / expected_rate_;
  interval_var_ = 0.0;
}

void InputManager::setObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  const auto & object = *msg;
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

InputManagers::InputManagers(rclcpp::Node & node) : node_(node)
{
}

void InputManagers::init(
  const std::vector<std::string> & input_topics, const std::vector<std::string> & long_names,
  const std::vector<std::string> & short_names)
{
  input_size_ = input_topics.size();
  for (size_t i = 0; i < input_size_; ++i) {
    InputManager input_manager(node_);
    input_manager.init(input_topics[i], long_names[i], short_names[i]);
    input_managers_.push_back(std::make_shared<InputManager>(input_manager));
  }
}
