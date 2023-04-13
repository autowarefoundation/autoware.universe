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

#include "refine_optimizer/config.hpp"

namespace refine_optimizer
{
RefineConfig::RefineConfig(rclcpp::Node * node)
{
  verbose_ = node->declare_parameter<bool>("refine.verbose", false);
  max_iteration_ = node->declare_parameter<int>("refine.max_iteration", 30);
  euler_bound_ = node->declare_parameter<double>("refine.euler_bound", 0.1);

  long_bound_ = node->declare_parameter<double>("refine.long_bound", 0.1);
  late_bound_ = node->declare_parameter<double>("refine.late_bound", 1.0);
  height_bound_ = node->declare_parameter<double>("refine.height_bound", 0.1);
}

}  // namespace refine_optimizer