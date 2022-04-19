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

#ifndef RTC_INTERFACE__RTC_INTERFACE_HPP_
#define RTC_INTERFACE__RTC_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/cooperate_command.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace rtc_interface
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::CooperateCommand;
using unique_identifier_msgs::msg::UUID;

class RTCInterface
{
public:
  RTCInterface(rclcpp::Node & node, const std::string & name, const Module & module);
  void publishCooperateStatus();
  void updateCooperateStatus(const UUID & uuid, const bool safe, const double distance);
  void removeCooperateStatus(const UUID & uuid);
  bool isActivated(const UUID & uuid) const;

private:
  void onCooperateCommandService(
    const CooperateCommand::Request::SharedPtr request,
    const CooperateCommand::Response::SharedPtr response);

  rclcpp::Publisher<CooperateStatusArray>::SharedPtr pub_status_;
  rclcpp::Service<CooperateCommand>::SharedPtr srv_command_;

  mutable rclcpp::Clock clock_;
  Module module_;
  CooperateStatusArray registered_status_;
};

}  // namespace rtc_interface

#endif  // RTC_INTERFACE__RTC_INTERFACE_HPP_
