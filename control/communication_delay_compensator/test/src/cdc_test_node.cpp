/*
* Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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


#include <memory>
#include "cdc_test_node.hpp"

std::shared_ptr<CommDelayNode> makeComDelayComNode()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("communication_delay_compensator");

  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "--params-file",
     share_dir + "/param/communication_delay_compensator.param.yaml",
     "--params-file", share_dir + "/params/test_vehicle_info.yaml"});

  auto node = std::make_shared<CommDelayNode>(node_options);

  // Enable all logging in the node
  auto ret = rcutils_logging_set_logger_level(
    node->get_logger().get_name(),
    RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  { std::cout << "Failed to set logging severity to DEBUG\n"; }

  return node;
}




