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

#include "autoware/control_validator/control_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node_options.hpp>

#include <gtest/gtest.h>

rclcpp::NodeOptions make_node_options()
{
  rclcpp::NodeOptions options;
  auto shared_directory =
    ament_index_cpp::get_package_share_directory("autoware_control_validator");
  return options;
}

TEST(test_suite_name, test_name)
{
  using autoware::control_validator::ControlValidator;

  rclcpp::NodeOptions options;
  auto node = std::make_shared<ControlValidator>(options);
}
