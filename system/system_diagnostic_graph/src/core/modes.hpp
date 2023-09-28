// Copyright 2023 The Autoware Contributors
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

#ifndef CORE__MODES_HPP_
#define CORE__MODES_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace system_diagnostic_graph
{

class OperationModes
{
public:
  explicit OperationModes(rclcpp::Node * node);

private:
  rclcpp::Publisher<OperationModeAvailability>::SharedPtr pub_modes_;

  UnitNode * stop_mode;
  UnitNode * autonomous_mode;
  UnitNode * local_mode;
  UnitNode * remote_mode;
  UnitNode * emergency_stop_mrm;
  UnitNode * comfortable_stop_mrm;
  UnitNode * pull_over_mrm;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__MODES_HPP_
