// Copyright 2020 Autoware Foundation
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

/**
 * @file mem_monitor_node.cpp
 * @brief Memory monitor node class
 */

#include "rclcpp/rclcpp.hpp"
#include "system_monitor/mem_monitor/mem_monitor.hpp"
#include "system_monitor/utils.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<MemMonitor> monitor = std::make_shared<MemMonitor>("mem_monitor", options);
  spin_and_update(monitor, std::chrono::seconds(1U));
  rclcpp::shutdown();
  return 0;
}
