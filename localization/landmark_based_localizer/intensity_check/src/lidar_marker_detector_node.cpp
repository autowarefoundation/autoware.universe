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

#include <rclcpp/rclcpp.hpp>

#include "intensity_check/lidar_marker_detector_core.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ptr = std::make_shared<LidarMarkerDetector>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_ptr);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
