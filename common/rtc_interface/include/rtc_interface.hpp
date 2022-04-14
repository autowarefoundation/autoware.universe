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

#ifndef RTC_INTERFACE_HPP_
#define RTC_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/cooperate_command.hpp"

#include <string>
#include <vector>

namespace rtc_interface
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::CooperateCommand;

class RTCInterface
{
public:
  RTCInterface(rclcpp::Node & node, const std::string & name);
  void addCooperateStatus(const CooperateStatus & status);
  void publishCooperateStatus() const;
  std::vector<bool> isActivated() const;

private:
  void onCooperateCommandService(
    CooperateCommand::Request::ConstSharedPtr request,
    CooperateCommand::Response::ConstSharedPtr response);

  rclcpp::Publisher<CooperateStatus>::SharedPtr pub_status_;
  rclcpp::Service<CooperateCommand>::SharedPtr srv_command_;
};

}  // namespace rtc_interface

#endif  // RTC_INTERFACE_HPP_
