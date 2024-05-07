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
///////////////////////////
/////// InputStream ///////
///////////////////////////
InputStream::InputStream(rclcpp::Node & node, uint & index) : node_(node), index_(index)
{
}

void InputStream::init(const InputChannel & input_channel)
{
  // Initialize parameters
  input_topic_ = input_channel.input_topic;
  long_name_ = input_channel.long_name;
  short_name_ = input_channel.short_name;
  expected_interval_ = input_channel.expected_interval;  // [s]

  // Initialize queue
  objects_que_.clear();

  // Initialize latency statistics
  latency_mean_ = input_channel.expected_latency;  // [s]
  latency_var_ = 0.0;
  interval_mean_ = expected_interval_;  // [s] (initial value)
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
  const DetectedObjects objects = *msg;
  objects_que_.push_back(objects);
  if (objects_que_.size() > que_size_) {
    objects_que_.pop_front();
  }

  // update the timing statistics
  rclcpp::Time now = node_.now();
  rclcpp::Time objects_time(objects.header.stamp);
  updateTimingStatus(now, objects_time);

  // trigger the function if it is set
  if (func_trigger_) {
    func_trigger_(index_);
  }
}

void InputStream::updateTimingStatus(const rclcpp::Time & now, const rclcpp::Time & objects_time)
{
  // Filter parameters
  constexpr double gain = 0.05;

  // Calculate interval, Update interval statistics
  if (is_time_initialized_) {
    const double interval = (now - latest_message_time_).seconds();
    // Check if the interval is regular
    // The interval is considered regular if it is within 0.5 and 1.5 times the expected interval
    bool is_interval_regular =
      interval > 0.5 * expected_interval_ && interval < 1.5 * expected_interval_;

    if (is_interval_regular) {
      interval_mean_ = (1.0 - gain) * interval_mean_ + gain * interval;
      const double interval_delta = interval - interval_mean_;
      interval_var_ = (1.0 - gain) * interval_var_ + gain * interval_delta * interval_delta;
    }
  }

  // Update time
  latest_message_time_ = now;
  latest_measurement_time_ =
    latest_measurement_time_ < objects_time ? objects_time : latest_measurement_time_;
  if (!is_time_initialized_) is_time_initialized_ = true;

  // Update latency statistics
  const double latency = (latest_message_time_ - objects_time).seconds();
  latency_mean_ = (1.0 - gain) * latency_mean_ + gain * latency;
  const double latency_delta = latency - latency_mean_;
  latency_var_ = (1.0 - gain) * latency_var_ + gain * latency_delta * latency_delta;
}

void InputStream::getObjectsOlderThan(
  const rclcpp::Time & object_latest_time, const rclcpp::Time & object_oldest_time,
  ObjectsList & objects_list)
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
      std::pair<uint, DetectedObjects> object_pair(index_, object);
      objects_list.push_back(object_pair);
      // remove the object from the queue
      objects_que_.pop_front();
    }
  }
}

////////////////////////////
/////// InputManager ///////
////////////////////////////
InputManager::InputManager(rclcpp::Node & node) : node_(node)
{
  latest_object_time_ = node_.now();
}

void InputManager::init(const std::vector<InputChannel> & input_channels)
{
  // Check input sizes
  input_size_ = input_channels.size();
  if (input_size_ == 0) {
    RCLCPP_ERROR(node_.get_logger(), "InputManager::init No input streams");
    return;
  }

  sub_objects_array_.resize(input_size_);

  for (size_t i = 0; i < input_size_; i++) {
    uint index(i);
    InputStream input_stream(node_, index);
    input_stream.init(input_channels[i]);
    input_stream.setTriggerFunction(
      std::bind(&InputManager::onTrigger, this, std::placeholders::_1));
    input_streams_.push_back(std::make_shared<InputStream>(input_stream));

    // Set subscription
    RCLCPP_INFO(
      node_.get_logger(), "InputManager::init Initializing %s input stream from %s",
      input_channels[i].long_name.c_str(), input_channels[i].input_topic.c_str());
    std::function<void(const DetectedObjects::ConstSharedPtr msg)> func =
      std::bind(&InputStream::onMessage, input_streams_.at(i), std::placeholders::_1);
    sub_objects_array_.at(i) = node_.create_subscription<DetectedObjects>(
      input_channels[i].input_topic, rclcpp::QoS{1}, func);
  }

  is_initialized_ = true;
}

void InputManager::onTrigger(const uint & index) const
{
  // when the target stream triggers, call the trigger function
  if (index == target_stream_idx_ && func_trigger_) {
    func_trigger_();
  }
}

void InputManager::getObjectTimeInterval(
  const rclcpp::Time & now, rclcpp::Time & object_latest_time,
  rclcpp::Time & object_oldest_time) const
{
  object_latest_time =
    now - rclcpp::Duration::from_seconds(
            target_stream_latency_ -
            0.1 * target_stream_latency_std_);  // object_latest_time with 0.1 sigma safety margin
  // check the target stream can be included in the object time interval
  if (input_streams_.at(target_stream_idx_)->isTimeInitialized()) {
    rclcpp::Time latest_measurement_time =
      input_streams_.at(target_stream_idx_)->getLatestMeasurementTime();
    // if the object_latest_time is newer than the next expected message time, set it older
    // than the next expected message time
    rclcpp::Time next_expected_message_time =
      latest_measurement_time +
      rclcpp::Duration::from_seconds(
        target_stream_interval_ -
        1.0 *
          target_stream_interval_std_);  // next expected message time with 1 sigma safety margin
    object_latest_time = object_latest_time > next_expected_message_time
                           ? next_expected_message_time
                           : object_latest_time;

    // if the object_latest_time is older than the latest measurement time, set it as the latest
    // object time
    object_latest_time =
      object_latest_time < latest_measurement_time ? latest_measurement_time : object_latest_time;
  }

  object_oldest_time = object_latest_time - rclcpp::Duration::from_seconds(1.0);
  // if the object_oldest_time is older than the latest object time, set it to the latest object
  // time
  object_oldest_time =
    object_oldest_time > latest_object_time_ ? object_oldest_time : latest_object_time_;
}

void InputManager::optimizeTimings()
{
  double max_latency_mean = 0.0;
  uint selected_stream_idx = 0;
  double selected_stream_latency_std = 0.1;
  double selected_stream_interval = 0.1;
  double selected_stream_interval_std = 0.01;

  {
    // ANALYSIS: Get the streams statistics
    // select the stream that has the maximum latency
    double latency_mean, latency_var, interval_mean, interval_var;
    for (const auto & input_stream : input_streams_) {
      if (!input_stream->isTimeInitialized()) continue;
      input_stream->getTimeStatistics(latency_mean, latency_var, interval_mean, interval_var);
      if (latency_mean > max_latency_mean) {
        max_latency_mean = latency_mean;
        selected_stream_idx = input_stream->getIndex();
        selected_stream_latency_std = std::sqrt(latency_var);
        selected_stream_interval = interval_mean;
        selected_stream_interval_std = std::sqrt(interval_var);
      }

      /* DEBUG */
      std::string long_name, short_name;
      rclcpp::Time latest_measurement_time, latest_message_time;
      input_stream->getNames(long_name, short_name);
      input_stream->getTimestamps(latest_measurement_time, latest_message_time);
      double latency_message = (node_.now() - latest_message_time).seconds();
      double latency_measurement = (node_.now() - latest_measurement_time).seconds();
      RCLCPP_INFO(
        node_.get_logger(),
        "InputManager::getObjects %s: latency mean: %f, std: %f, interval mean: "
        "%f, std: %f, latest measurement latency: %f, latest message latency: %f",
        long_name.c_str(), latency_mean, std::sqrt(latency_var), interval_mean,
        std::sqrt(interval_var), latency_measurement, latency_message);
    }
  }

  // Set the target stream index, which has the maximum latency
  // trigger will be called next time
  // if no stream is initialized, the target stream index will be 0 and wait for the initialization
  target_stream_idx_ = selected_stream_idx;
  target_stream_latency_ = max_latency_mean;
  target_stream_latency_std_ = selected_stream_latency_std;
  target_stream_interval_ = selected_stream_interval;
  target_stream_interval_std_ = selected_stream_interval_std;
}

bool InputManager::getObjects(const rclcpp::Time & now, ObjectsList & objects_list)
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

  // Optimize the target stream, latency, and its band
  // The result will be used for the next time, so the optimization is after getting the time
  // interval
  optimizeTimings();

  // Get objects from all input streams
  // adds up to the objects vector for efficient processing
  for (const auto & input_stream : input_streams_) {
    input_stream->getObjectsOlderThan(object_latest_time, object_oldest_time, objects_list);
  }

  // Sort objects by timestamp
  std::sort(
    objects_list.begin(), objects_list.end(),
    [](const std::pair<uint, DetectedObjects> & a, const std::pair<uint, DetectedObjects> & b) {
      return (rclcpp::Time(a.second.header.stamp) - rclcpp::Time(b.second.header.stamp)).seconds() <
             0;
    });

  // Update the latest object time
  if (!objects_list.empty()) {
    latest_object_time_ = rclcpp::Time(objects_list.back().second.header.stamp);
  }

  return !objects_list.empty();
}

}  // namespace multi_object_tracker
