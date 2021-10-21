// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__APPROVAL_HANDLER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__APPROVAL_HANDLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace behavior_path_planner
{
class ApprovalHandler
{
public:
  void setCurrentApproval(const behavior_path_planner::BoolStamped & approval)
  {
    approval_ = approval;
  }

  bool isApproved() const
  {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    const auto now = clock.now();
    const auto thresh_sec = 0.5;
    if (approval_.data && (now - approval_.stamp).seconds() < thresh_sec) {
      if ((now - last_clear_time_).seconds() > thresh_sec) {
        return true;
      }
    }
    return false;
  }

  bool isWaitingApproval() const {return is_waiting_approval_;}
  void waitApproval() {is_waiting_approval_ = true;}
  void clearWaitApproval() {is_waiting_approval_ = false;}
  void clearApproval()
  {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    last_clear_time_ = clock.now();
    approval_.data = false;
  }

private:
  BoolStamped approval_{false};
  bool is_waiting_approval_{true};
  rclcpp::Time last_clear_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__APPROVAL_HANDLER_HPP_
