// Copyright 2020 Tier IV, Inc.
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

#include "rclcpp/rclcpp.hpp"

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

namespace autoware_api
{
class AutowareIvStopReasonAggregator
{
public:
  AutowareIvStopReasonAggregator(rclcpp::Node& node, const double timeout);
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr updateStopReasonArray(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr);

private:
  void applyUpdate(const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr);
  bool checkMatchingReason(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_stop_reason_array,
    const autoware_planning_msgs::msg::StopReasonArray & stop_reason_array);
  void applyTimeOut();
  void appendStopReasonToArray(
    const autoware_planning_msgs::msg::StopReason & stop_reason,
    autoware_planning_msgs::msg::StopReasonArray * stop_reason_array);
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr makeStopReasonArray();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double timeout_;
  std::vector<autoware_planning_msgs::msg::StopReasonArray> stop_reason_array_vec_;
};

}  // namespace autoware_api
