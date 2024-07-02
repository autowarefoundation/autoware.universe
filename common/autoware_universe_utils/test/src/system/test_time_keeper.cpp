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

#include "autoware/universe_utils/system/time_keeper.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

TEST(system, TimeKeeper)
{
  using autoware::universe_utils::ScopedStopWatch;
  using autoware::universe_utils::TimeKeeper;

  rclcpp::Node node{"sample_node"};

  TimeKeeper time_keeper(&node);

  time_keeper.start_track("main_func");

  {  // funcA
    ScopedStopWatch ss{"funcA", time_keeper};
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  {  // funcB
    ScopedStopWatch ss{"funcB", time_keeper};
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {  // funcC
      ScopedStopWatch ss{"funcC", time_keeper};
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  time_keeper.end_track("main_func");
  ASSERT_ANY_THROW(time_keeper.report(true));
}
