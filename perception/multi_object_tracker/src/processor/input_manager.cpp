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

namespace multi_object_tracker
{

InputStream::InputStream(rclcpp::Node & node, size_t & index) : node_(node), index_(index)
{
}

void InputStream::init(
  const std::string & input_topic, const std::string & long_name, const std::string & short_name)
{
  // Initialize parameters
  input_topic_ = input_topic;
  long_name_ = long_name;
  short_name_ = short_name;

  // Initialize queue
  objects_que_.clear();

  // Initialize latency statistics
  expected_rate_ = 10.0;
  latency_mean_ = 0.18;  // [s]
  latency_var_ = 0.0;
  interval_mean_ = 1 / expected_rate_;
  interval_var_ = 0.0;

  latest_measurement_time_ = node_.now();
  latest_message_time_ = node_.now();
}

bool InputStream::getTimestamps(
  rclcpp::Time & latest_measurement_time, rclcpp::Time & latest_message_time) const
{
  if (!is_time_initialized_) {
    return false;
  }
  latest_measurement_time = latest_measurement_time_;
  latest_message_time = latest_message_time_;
  return true;
}

void InputStream::onMessage(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  // // debug message
  // RCLCPP_INFO(
  //   node_.get_logger(), "InputStream::onMessage Received %s message from %s at %d.%d",
  //   long_name_.c_str(), input_topic_.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec);

  const DetectedObjects objects = *msg;
  objects_que_.push_back(objects);
  if (objects_que_.size() > que_size_) {
    objects_que_.pop_front();
  }

  // RCLCPP_INFO(node_.get_logger(), "InputStream::onMessage Que size: %zu", objects_que_.size());

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
  latest_measurement_time_ = objects.header.stamp;
  if (!is_time_initialized_) is_time_initialized_ = true;

  // Calculate latency
  const double latency = (latest_message_time_ - latest_measurement_time_).seconds();

  // Update latency statistics
  latency_mean_ = (1.0 - gain) * latency_mean_ + gain * latency;
  const double latency_delta = latency - latency_mean_;
  latency_var_ = (1.0 - gain) * latency_var_ + gain * latency_delta * latency_delta;

  // trigger the function if it is set
  if (func_trigger_) {
    func_trigger_(index_);
  }
}

void InputStream::getObjectsOlderThan(
  const rclcpp::Time & object_latest_time, const rclcpp::Time & object_oldest_time,
  std::vector<std::pair<size_t, DetectedObjects>> & objects_list)
{
  assert(object_latest_time.nanoseconds() > object_oldest_time.nanoseconds());

  for (const auto & object : objects_que_) {
    const rclcpp::Time object_time = rclcpp::Time(object.header.stamp);

    // remove objects older than the specified duration
    if (object_time < object_oldest_time) {
      objects_que_.pop_front();
      continue;
    }

    // Add the object if the object is older than the specified latest time
    if (object_latest_time >= object_time) {
      std::pair<size_t, DetectedObjects> object_pair(index_, object);
      objects_list.push_back(object_pair);
      // remove the object from the queue
      objects_que_.pop_front();
    }
  }
  // RCLCPP_INFO(
  //   node_.get_logger(), "InputStream::getObjectsOlderThan %s gives %zu objects",
  //   long_name_.c_str(), objects.size());
}

InputManager::InputManager(rclcpp::Node & node) : node_(node)
{
  latest_object_time_ = node_.now();
}

void InputManager::init(
  const std::vector<std::string> & input_topics, const std::vector<std::string> & long_names,
  const std::vector<std::string> & short_names)
{
  // Check input sizes
  input_size_ = input_topics.size();
  if (input_size_ == 0) {
    RCLCPP_ERROR(node_.get_logger(), "InputManager::init No input streams");
    return;
  }
  assert(input_size_ == long_names.size());
  assert(input_size_ == short_names.size());

  sub_objects_array_.resize(input_size_);

  for (size_t i = 0; i < input_size_; i++) {
    InputStream input_stream(node_, i);
    input_stream.init(input_topics.at(i), long_names.at(i), short_names.at(i));
    input_stream.setTriggerFunction(
      std::bind(&InputManager::onTrigger, this, std::placeholders::_1));
    input_streams_.push_back(std::make_shared<InputStream>(input_stream));

    // Set subscription
    RCLCPP_INFO(
      node_.get_logger(), "InputManager::init Initializing %s input stream from %s",
      long_names.at(i).c_str(), input_topics.at(i).c_str());
    std::function<void(const DetectedObjects::ConstSharedPtr msg)> func =
      std::bind(&InputStream::onMessage, input_streams_.at(i), std::placeholders::_1);
    sub_objects_array_.at(i) =
      node_.create_subscription<DetectedObjects>(input_topics.at(i), rclcpp::QoS{1}, func);
  }

  is_initialized_ = true;
}

void InputManager::onTrigger(const size_t & index) const
{
  // input stream index of 0 is the target(main) input stream
  const size_t target_idx = 0;

  // when the main stream triggers, call the trigger function
  if (index == target_idx && func_trigger_) {
    func_trigger_();
  }
}

void InputManager::getObjectTimeInterval(
  const rclcpp::Time & now, rclcpp::Time & object_latest_time, rclcpp::Time & object_oldest_time)
{
  {
    // ANALYSIS: Get the streams statistics
    std::string long_name, short_name;
    double latency_mean, latency_var, interval_mean, interval_var;
    rclcpp::Time latest_measurement_time, latest_message_time;
    for (const auto & input_stream : input_streams_) {
      input_stream->getNames(long_name, short_name);
      input_stream->getTimeStatistics(latency_mean, latency_var, interval_mean, interval_var);
      if (!input_stream->getTimestamps(latest_measurement_time, latest_message_time)) {
        continue;
      }
      double latency_message = (now - latest_message_time).seconds();
      double latency_measurement = (now - latest_measurement_time).seconds();
      RCLCPP_INFO(
        node_.get_logger(),
        "InputManager::getObjects %s: latency mean: %f, std: %f, interval mean: "
        "%f, std: %f, latest measurement latency: %f, latest message latency: %f",
        long_name.c_str(), latency_mean, std::sqrt(latency_var), interval_mean,
        std::sqrt(interval_var), latency_measurement, latency_message);
    }
  }

  // Get proper latency
  constexpr double target_latency = 0.15;  // [s], measurement to tracking latency
                                           // process latency of a main detection + margin

  constexpr double acceptable_latency =
    0.35;  // [s], acceptable band from the target latency, larger than the target latency
  object_latest_time = now - rclcpp::Duration::from_seconds(target_latency);
  object_oldest_time = now - rclcpp::Duration::from_seconds(acceptable_latency);

  // if the object_oldest_time is older than the latest object time, set it to the latest object
  // time
  object_oldest_time =
    object_oldest_time > latest_object_time_ ? object_oldest_time : latest_object_time_;
}

bool InputManager::getObjects(
  const rclcpp::Time & now, std::vector<std::pair<size_t, DetectedObjects>> & objects_list)
{
  if (!is_initialized_) {
    RCLCPP_INFO(node_.get_logger(), "InputManager::getObjects Input manager is not initialized");
    return false;
  }

  // Clear the objects
  objects_list.clear();

  // Get the time interval for the objects
  rclcpp::Time object_latest_time, object_oldest_time;
  getObjectTimeInterval(now, object_latest_time, object_oldest_time);

  // Get objects from all input streams
  // adds-up to the objects vector for efficient processing
  for (const auto & input_stream : input_streams_) {
    input_stream->getObjectsOlderThan(object_latest_time, object_oldest_time, objects_list);
  }

  // Sort objects by timestamp
  std::sort(
    objects_list.begin(), objects_list.end(),
    [](const std::pair<size_t, DetectedObjects> & a, const std::pair<size_t, DetectedObjects> & b) {
      return (rclcpp::Time(a.second.header.stamp) - rclcpp::Time(b.second.header.stamp)).seconds() <
             0;
    });

  RCLCPP_INFO(
    node_.get_logger(), "InputManager::getObjects Got %zu objects from input streams",
    objects_list.size());

  // Update the latest object time
  if (!objects_list.empty()) {
    latest_object_time_ = rclcpp::Time(objects_list.back().second.header.stamp);
  }

  return !objects_list.empty();
}

}  // namespace multi_object_tracker
