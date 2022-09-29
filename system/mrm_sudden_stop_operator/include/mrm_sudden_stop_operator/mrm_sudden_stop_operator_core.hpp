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

#ifndef MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
#define MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware
#include <autoware_adapi_v1_msgs/srv/operate_mrm.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_behavior_status.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

namespace mrm_sudden_stop_operator
{
  using autoware_adapi_v1_msgs::srv::OperateMRM;
  using autoware_adapi_v1_msgs::msg::MRMBehaviorStatus;
  using autoware_auto_control_msgs::msg::AckermannControlCommand;

struct Parameters
{
  int update_rate;  // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;  // [m/s^3]
};

class MRMSuddenStopOperator : public rclcpp::Node
{
public:
  explicit MRMSuddenStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;

  void onControlCommand(AckermannControlCommand::ConstSharedPtr msg);

  // Server
  rclcpp::Service<OperateMRM>::SharedPtr service_operation_;

  void operateSuddenStop(
    const OperateMRM::Request::SharedPtr request,
    const OperateMRM::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<MRMBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;

  void publishStatus() const;
  void publishControlCommand(const AckermannControlCommand & command) const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // States
  MRMBehaviorStatus status_;
  AckermannControlCommand prev_control_cmd_;
  bool is_prev_control_cmd_subscribed_;

  // Algorithm
  AckermannControlCommand calcTargetAcceleration(const AckermannControlCommand & prev_control_cmd) const;

};

}  // namespace mrm_sudden_stop_operator

#endif  // MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
