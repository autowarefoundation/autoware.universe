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
#include <memory>
#include <string>
#include <vector>

class InputStream
{
public:
  explicit InputStream(rclcpp::Node & node);

  void init(
    const std::string & input_topic, const std::string & long_name, const std::string & short_name);

  void setObjects(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);
  void getObjectsOlderThan(
    const rclcpp::Time & time, const double duration,
    std::vector<autoware_auto_perception_msgs::msg::DetectedObjects> & objects);

private:
  rclcpp::Node & node_;

  std::string input_topic_;
  std::string long_name_;
  std::string short_name_;

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr sub_objects_;

  size_t que_size_{20};
  std::deque<autoware_auto_perception_msgs::msg::DetectedObjects> objects_que_;

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

  bool getObjects(
    const rclcpp::Time & now,
    std::vector<autoware_auto_perception_msgs::msg::DetectedObjects> & objects);

private:
  rclcpp::Node & node_;

  size_t input_size_;
  std::vector<std::shared_ptr<InputStream>> input_streams_;
};

#endif  // MULTI_OBJECT_TRACKER__PROCESSOR__INPUT_MANAGER_HPP_
