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

#include "src/debug.hpp"
#include "src/scene.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

using autoware::behavior_velocity_planner::RunOutDebug;

class TestRunOutDebug : public ::testing::Test
{
public:
  void SetUp() override
  {
    run_out_debug_ptr_ = std::make_shared<RunOutDebug>(RunOutDebug()) rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    run_out_debug_ptr_ = nullptr;
    rclcpp::shutdown();
  }

  std::shared_ptr<RunOutDebug> run_out_debug_ptr_;
};
