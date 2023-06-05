// Copyright 2023 Autoware Foundation
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

#include "ekf_localizer/ekf_localizer.hpp"
#include <gtest/gtest.h>
#include <chrono>

TEST(Simple1DFilter, InitializationAndAccess)
{
  Simple1DFilter filter;
  auto initial_time = std::chrono::system_clock::now();
  int64_t time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(initial_time.time_since_epoch()).count();
  rclcpp::Time rclcpp_time(time_in_ns);

  filter.init(10.0, 2.0, rclcpp_time);

  EXPECT_EQ(filter.get_x(), 10.0);
}

TEST(Simple1DFilter, Update)
{
  Simple1DFilter filter;
  auto initial_time = std::chrono::system_clock::now();
  int64_t time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(initial_time.time_since_epoch()).count();
  rclcpp::Time rclcpp_time(time_in_ns);
  
  filter.init(10.0, 2.0, rclcpp_time);
  auto new_time = initial_time + std::chrono::seconds(1);
  int64_t new_time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(new_time.time_since_epoch()).count();
  rclcpp::Time new_rclcpp_time(new_time_in_ns);

  filter.update(12.0, 3.0, new_rclcpp_time);

  EXPECT_NEAR(filter.get_x(), 10.8, 1e-9); // Considering precision upto 9 decimal places
}

TEST(Simple1DFilter, MahalanobisGate)
{
  Simple1DFilter filter(2.0);
  auto initial_time = std::chrono::system_clock::now();
  int64_t time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(initial_time.time_since_epoch()).count();
  rclcpp::Time rclcpp_time(time_in_ns);

  filter.init(10.0, 2.0, rclcpp_time);
  auto new_time = initial_time + std::chrono::seconds(1);
  int64_t new_time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(new_time.time_since_epoch()).count();
  rclcpp::Time new_rclcpp_time(new_time_in_ns);

  filter.update(15.0, 1.5, new_rclcpp_time); // This should be ignored due to gate
  
  EXPECT_EQ(filter.get_x(), 10.0); // The update should have been ignored, so x_ should still be 10.0
}