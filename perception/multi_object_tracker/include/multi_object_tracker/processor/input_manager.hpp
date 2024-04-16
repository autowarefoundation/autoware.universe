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

#ifndef MULTI_OBJECT_TRACKER__PROCESSOR__INPUT_MANAGER_HPP_
#define MULTI_OBJECT_TRACKER__PROCESSOR__INPUT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace multi_object_tracker
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

class InputStream
{
public:
  explicit InputStream(rclcpp::Node & node, size_t & index);

  void init(
    const std::string & input_topic, const std::string & long_name, const std::string & short_name);

  void setQueueSize(size_t que_size) { que_size_ = que_size; }
  void setTriggerFunction(std::function<void(const size_t &)> func_trigger)
  {
    func_trigger_ = func_trigger;
  }

  void onMessage(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);

  void getObjectsOlderThan(
    const rclcpp::Time & object_latest_time, const rclcpp::Time & object_oldest_time,
    std::vector<std::pair<size_t, DetectedObjects>> & objects);
  void getNames(std::string & long_name, std::string & short_name)
  {
    long_name = long_name_;
    short_name = short_name_;
  }
  size_t getObjectsCount() const { return objects_que_.size(); }
  void getTimeStatistics(
    double & latency_mean, double & latency_var, double & interval_mean,
    double & interval_var) const
  {
    latency_mean = latency_mean_;
    latency_var = latency_var_;
    interval_mean = interval_mean_;
    interval_var = interval_var_;
  }
  bool getTimestamps(
    rclcpp::Time & latest_measurement_time, rclcpp::Time & latest_message_time) const;

private:
  rclcpp::Node & node_;
  size_t index_;

  std::string input_topic_;
  std::string long_name_;
  std::string short_name_;

  size_t que_size_{30};
  std::deque<DetectedObjects> objects_que_;

  std::function<void(const size_t &)> func_trigger_;

  bool is_time_initialized_{false};
  double expected_rate_;
  double latency_mean_;
  double latency_var_;
  double interval_mean_;
  double interval_var_;

  rclcpp::Time latest_measurement_time_;
  rclcpp::Time latest_message_time_;
};

class InputManager
{
public:
  explicit InputManager(rclcpp::Node & node);

  void init(
    const std::vector<std::string> & input_topics, const std::vector<std::string> & long_names,
    const std::vector<std::string> & short_names);
  void setTriggerFunction(std::function<void()> func_trigger) { func_trigger_ = func_trigger; }

  void onTrigger(const size_t & index) const;

  void getObjectTimeInterval(
    const rclcpp::Time & now, rclcpp::Time & object_latest_time, rclcpp::Time & object_oldest_time);
  bool getObjects(
    const rclcpp::Time & now, std::vector<std::pair<size_t, DetectedObjects>> & objects_list);

private:
  rclcpp::Node & node_;
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> sub_objects_array_{};

  bool is_initialized_{false};
  rclcpp::Time latest_object_time_;

  size_t input_size_;
  std::vector<std::shared_ptr<InputStream>> input_streams_;

  std::function<void()> func_trigger_;
};

}  // namespace multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER__PROCESSOR__INPUT_MANAGER_HPP_
