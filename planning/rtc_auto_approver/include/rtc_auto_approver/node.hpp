// Copyright 2022 Tier IV, Inc.
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

#ifndef RTC_AUTO_APPROVER__NODE_HPP_
#define RTC_AUTO_APPROVER__NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rtc_auto_approver/rtc_auto_approver_interface.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rtc_auto_approver
{
class RTCAutoApproverNode : public rclcpp::Node
{
public:
  explicit RTCAutoApproverNode(const rclcpp::NodeOptions & node_options);

private:
  std::vector<std::shared_ptr<RTCAutoApproverInterface>> approvers_;

  std::string BEHAVIOR_PLANNING_NAMESPACE =
    "/planning/scenario_planning/lane_driving/behavior_planning";
  std::string BEHAVIOR_VELOCITY_PLANNER_NAMESPACE = "/behavior_velocity_planner";
  std::string BEHAVIOR_PATH_PLANNER_NAMESPACE = "/behavior_path_planner";

  std::vector<std::string> behavior_velocity_planner_modules_ = {
    "blind_spot",   "crosswalk",        "detection_area",
    "intersection", "no_stopping_area", "traffic_light"};

  std::vector<std::string> behavior_path_planner_modules_ = {
    "lane_change_left", "lane_change_right", "avoidance_left",
    "avoidance_right",  "pull_over",         "pull_out"};
};

}  // namespace rtc_auto_approver

#endif  // RTC_AUTO_APPROVER__NODE_HPP_
