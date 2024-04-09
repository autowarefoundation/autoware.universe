// Copyright 2024 TIER IV, inc.
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

#include "object_merger/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <glog/logging.h>

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);  // NOLINT
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto object_association_merger = std::make_shared<object_association::ObjectAssociationMergerNode>(options);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(object_association_merger);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
