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

#include <rclcpp/rclcpp.hpp>
#include <rtc_interface/rtc_interface.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <random>
#include <string>

namespace behavior_path_planner
{
using rtc_interface::RTCInterface;
using unique_identifier_msgs::msg::UUID;
class ApprovalHandler
{
public:
  explicit ApprovalHandler(rclcpp::Node & node, const std::string & name)
  : clock_{*node.get_clock()},
    logger_{node.get_logger().get_child("[" + name + "]")},
    rtc_interface_left_(node, name + "_left"),
    rtc_interface_right_(node, name + "_right"),
    uuid_left_(generateUUID()),
    uuid_right_(generateUUID())
  {
  }

  bool isApproved() const
  {
    return rtc_interface_left_.isActivated(uuid_left_) ||
           rtc_interface_right_.isActivated(uuid_right_);
  }

  bool isWaitingApproval() const { return is_waiting_approval_; }
  void waitApprovalLeft(const bool safe, const double distance)
  {
    RCLCPP_WARN_STREAM(logger_, "[waitApprovalLeft] : updateCooperateStatus");
    rtc_interface_left_.updateCooperateStatus(uuid_left_, safe, distance, clock_.now());
    RCLCPP_WARN_STREAM(logger_, "[waitApprovalLeft] : publishCooperateStatus");
    rtc_interface_left_.publishCooperateStatus(clock_.now());
    is_waiting_approval_ = true;
  }
  void waitApprovalRight(const bool safe, const double distance)
  {
    rtc_interface_right_.updateCooperateStatus(uuid_right_, safe, distance, clock_.now());
    rtc_interface_right_.publishCooperateStatus(clock_.now());
    is_waiting_approval_ = true;
  }
  void clearWaitApproval()
  {
    rtc_interface_left_.removeCooperateStatus(uuid_left_);
    rtc_interface_right_.removeCooperateStatus(uuid_right_);
    is_waiting_approval_ = false;
  }

private:
  bool is_waiting_approval_{false};

  mutable rclcpp::Clock clock_;

  rclcpp::Logger logger_;

  // for RTC
  RTCInterface rtc_interface_left_;
  RTCInterface rtc_interface_right_;
  UUID uuid_left_;
  UUID uuid_right_;

  UUID generateUUID()
  {
    // Generate random number
    UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);

    return uuid;
  }
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__APPROVAL_HANDLER_HPP_
