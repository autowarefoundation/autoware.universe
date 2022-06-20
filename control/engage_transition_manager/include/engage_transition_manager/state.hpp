// Copyright 2022 Autoware Foundation
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

#ifndef ENGAGE_TRANSITION_MANAGER__STATE_HPP_
#define ENGAGE_TRANSITION_MANAGER__STATE_HPP_

#include <engage_transition_manager/data.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_system_msgs/msg/operation_mode.hpp>
#include <tier4_system_msgs/srv/operation_mode_request.hpp>
#include <tier4_vehicle_msgs/srv/control_mode_request.hpp>

namespace engage_transition_manager
{
class EngageStateBase
{
public:
  EngageStateBase(const State state, rclcpp::Node * node);
  ~EngageStateBase() = default;

  virtual State update() = 0;

  State getCurrentState() { return state_; };
  void setData(const Data & data) { data_ = data; };
  void setParam(const StableCheckParam & param) { stable_check_param_ = param; };

protected:
  rclcpp::Client<ControlModeRequest>::SharedPtr srv_mode_change_client_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  State state_;
  Data data_;
  StableCheckParam stable_check_param_;

  State defaultUpdateOnManual();
  bool sendAutonomousModeRequest();
};

class NoneState : public EngageStateBase
{
public:
  NoneState(rclcpp::Node * node) : EngageStateBase(State::STOP, node){};
  State update() override { return defaultUpdateOnManual(); };
};

class RemoteState : public EngageStateBase
{
public:
  RemoteState(rclcpp::Node * node) : EngageStateBase(State::REMOTE_OPERATOR, node){};
  State update() override { return defaultUpdateOnManual(); };
};

class DirectState : public EngageStateBase
{
public:
  DirectState(rclcpp::Node * node) : EngageStateBase(State::MANUAL_DIRECT, node){};
  State update() override { return defaultUpdateOnManual(); };
};

class LocalState : public EngageStateBase
{
public:
  LocalState(rclcpp::Node * node) : EngageStateBase(State::LOCAL_OPERATOR, node){};
  State update() override { return defaultUpdateOnManual(); };
};

class TransitionState : public EngageStateBase
{
public:
  TransitionState(rclcpp::Node * node) : EngageStateBase(State::TRANSITION_TO_AUTO, node){};
  State update() override;

private:
  std::unique_ptr<rclcpp::Time> stable_start_time_;
  bool checkSystemStable();
};

class AutonomousState : public EngageStateBase
{
public:
  AutonomousState(rclcpp::Node * node) : EngageStateBase(State::AUTONOMOUS, node){};
  State update() override;
};

}  // namespace engage_transition_manager

#endif  // ENGAGE_TRANSITION_MANAGER__STATE_HPP_
