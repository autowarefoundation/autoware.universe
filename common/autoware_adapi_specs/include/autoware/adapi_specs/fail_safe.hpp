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

#ifndef AUTOWARE__ADAPI_SPECS__FAIL_SAFE_HPP_
#define AUTOWARE__ADAPI_SPECS__FAIL_SAFE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>

namespace autoware::adapi_specs::fail_safe
{

struct MrmState
{
  using Message = autoware_adapi_v1_msgs::msg::MrmState;
  static constexpr char name[] = "/api/fail_safe/mrm_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::adapi_specs::fail_safe

#endif  // AUTOWARE__ADAPI_SPECS__FAIL_SAFE_HPP_
