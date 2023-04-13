// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <rclcpp/node.hpp>

namespace refine_optimizer
{
struct RefineConfig
{
  RefineConfig(rclcpp::Node * node);

  RefineConfig(bool verbose = false, int max_iteration = 50, double euler_bound = 0.1)
  : verbose_(verbose), max_iteration_(max_iteration), euler_bound_(euler_bound)
  {
    long_bound_ = 0.1;
    late_bound_ = 1.0;
    height_bound_ = 0.1;
  }
  bool verbose_;
  int max_iteration_;
  double euler_bound_;
  double long_bound_;
  double late_bound_;
  double height_bound_;
};

}  // namespace refine_optimizer