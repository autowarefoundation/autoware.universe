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

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

TEST(system, TimeKeeper)
{
  using autoware::universe_utils::TimeKeeper;

  TimeKeeper time_keeper(rclcpp::Node{"sample_node"});

  time_keeper.start("main_func");

  {  // funcA
    const auto auto_stop_watch{time_keeper.track("funcA")};
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  {  // funcB
    const auto auto_stop_watch{time_keeper.track("funcB")};
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {  // funcC
      const auto auto_stop_watch{time_keeper.track("funcC")};
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  ASSERT_ANY_THROW(time_keeper.end("main_func"));
}
