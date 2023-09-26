// Copyright 2023 Tier IV, Inc.
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

// ===== Note =====
// This is a util class implementation of the logger_config_component provided by ROS2
// https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_config_component.cpp
//
// When ROS2 officially supports the set_logger_level option in release version, this class can be
// removed.
// https://github.com/ros2/ros2/issues/1355

#ifndef TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_

#include "logging_demo/srv/config_logger.hpp"

#include <rclcpp/rclcpp.hpp>

using logging_demo::srv::ConfigLogger;

class LoggerLevelConfigure
{
public:
  explicit LoggerLevelConfigure(rclcpp::Node * node);

private:
  rclcpp::Logger ros_logger_;
  rclcpp::Service<ConfigLogger>::SharedPtr srv_config_logger_;

  void onLoggerConfigService(
    const ConfigLogger::Request::SharedPtr request,
    const ConfigLogger::Response::SharedPtr response);
};

#endif  // TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_
