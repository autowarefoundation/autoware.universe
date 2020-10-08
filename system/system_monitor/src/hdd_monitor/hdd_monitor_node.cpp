/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file hdd_monitor_node.cpp
 * @brief HDD monitor node class
 */

#include <system_monitor/hdd_monitor/hdd_monitor.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "hdd_monitor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  HDDMonitor monitor(nh, pnh);
  monitor.run();

  return 0;
}
