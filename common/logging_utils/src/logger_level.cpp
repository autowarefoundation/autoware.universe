// Copyright 2023 The Autoware Contributors
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

#include "logger_level.hpp"

#include <string>

namespace logging_utils
{

LoggerLevel::LoggerLevel(const rclcpp::NodeOptions & options) : Node("logger_level", options)
{
  const auto name = declare_parameter<std::string>("name");
  const auto level = declare_parameter<std::string>("level");
  int severity;

  // Get logging severity.
  {
    const auto ret = rcutils_logging_severity_level_from_string(
      level.c_str(), rcl_get_default_allocator(), &severity);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to get log level: " << level);
      return;
    }
  }

  // Set logging severity.
  {
    const auto ret = rcutils_logging_set_logger_level(name.c_str(), severity);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set log level");
      return;
    }
  }
}

}  // namespace logging_utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(logging_utils::LoggerLevel)
