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

#ifndef MACROS__CREATE_NODE_HPP_
#define MACROS__CREATE_NODE_HPP_

#include <memory>

#define CREATE_SINGLE_THREAD_NODE(Target) \
  CREATE_NODE(Target, rclcpp::executors::SingleThreadedExecutor)
#define CREATE_MULTI_THREAD_NODE(Target) \
  CREATE_NODE(Target, rclcpp::executors::MultiThreadedExecutor)

#define CREATE_NODE(Target, Executor)       \
  int main(int argc, char ** argv)          \
  {                                         \
    rclcpp::init(argc, argv);               \
    Executor executor;                      \
    auto node = std::make_shared<Target>(); \
    executor.add_node(node);                \
    executor.spin();                        \
    executor.remove_node(node);             \
    rclcpp::shutdown();                     \
  }

#endif  // MACROS__CREATE_NODE_HPP_
