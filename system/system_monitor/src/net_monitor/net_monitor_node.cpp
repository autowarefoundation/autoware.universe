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
 * @file net_monitor_node.cpp
 * @brief Net monitor node class
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "system_monitor/net_monitor/net_monitor.hpp"
#include "system_monitor/utils.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<NetMonitor> monitor = std::make_shared<NetMonitor>("net_monitor", options);
  spin_and_update(monitor, std::chrono::seconds(1U));
  monitor->shutdown_nl80211();
  rclcpp::shutdown();
  return 0;
}
