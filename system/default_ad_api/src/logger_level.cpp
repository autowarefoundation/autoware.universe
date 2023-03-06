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

namespace default_ad_api
{

LoggerLevel::LoggerLevel(const rclcpp::NodeOptions & options) : Node("logger_level", options)
{
  const std::string name = "default_ad_api.container";
  const auto level = RCUTILS_LOG_SEVERITY_WARN;

  const auto ret = rcutils_logging_set_logger_level(name.c_str(), level);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(get_logger(), "Error log level: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::LoggerLevel)
