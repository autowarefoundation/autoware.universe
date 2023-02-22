// Copyright 2022 TIER IV, Inc.
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

#include "timing_violation_monitor/timing_violation_monitor_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <signal.h>

static void signal_handler(int signum)
{
  printf("signal_handler: caught signal %d\n", signum);
  if (signum == SIGINT) {
    printf("--- SIGINT ---\n");
    exit(1);
  }
}

int main(int argc, char ** argv)
{
  if (signal(SIGINT, signal_handler) == SIG_ERR) {
    printf("## Error caught signal\n");
    exit(-1);
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<timing_violation_monitor::TimingViolationMonitor>();
  node->registerNodeToDebug(node);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
