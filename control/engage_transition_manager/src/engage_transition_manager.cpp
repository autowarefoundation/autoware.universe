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

#include "engage_transition_manager/engage_transition_manager.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"


#include <algorithm>
#include <cmath>

namespace engage_transition_manager
{

using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::findNearestIndex;


EngageTransitionManager::EngageTransitionManager(const rclcpp::NodeOptions & options)
: Node("engage_transition_manager", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  engage_transition_manager_ = std::make_unique<NoneState>(this);

  pub_operation_mode_ = create_publisher<OperationMode>("engage_state", 1);
  pub_auto_available_ = create_publisher<IsAutonomousAvailable>("is_auto_available", 1);

  sub_vehicle_kinematics_ = create_subscription<Odometry>(
    "kinematics", 1, [this](const Odometry::SharedPtr msg) { data_.kinematics = *msg; });

  sub_trajectory_ = create_subscription<Trajectory>(
    "trajectory", 1, [this](const Trajectory::SharedPtr msg) { data_.trajectory = *msg; });

  srv_mode_change_ = create_service<OperationModeRequest>(
    "operation_mode_request", std::bind(&EngageTransitionManager::onOperationModeRequest, this, _1, _2));

  engage_acceptable_param_.dist_threshold = declare_parameter<double>("engage_acceptable_limits.dist_threshold");
  engage_acceptable_param_.speed_threshold = declare_parameter<double>("engage_acceptable_limits.speed_threshold");
  engage_acceptable_param_.yaw_threshold = declare_parameter<double>("engage_acceptable_limits.yaw_threshold");

  std::cerr << "param_.dist_threshold" << engage_acceptable_param_.dist_threshold << ", yaw_threshold" << engage_acceptable_param_.yaw_threshold << ", speed_threshold" << engage_acceptable_param_.speed_threshold << std::endl;

  stable_check_param_.duration = declare_parameter<double>("stable_check.duration");
  stable_check_param_.dist_threshold = declare_parameter<double>("stable_check.dist_threshold");
  stable_check_param_.speed_threshold = declare_parameter<double>("stable_check.speed_threshold");
  stable_check_param_.yaw_threshold = declare_parameter<double>("stable_check.yaw_threshold");

  std::cerr << "stable_check_param_.duration" << stable_check_param_.duration << "dist_threshold" << stable_check_param_.dist_threshold << ", yaw_threshold" << stable_check_param_.yaw_threshold << ", speed_threshold" << stable_check_param_.speed_threshold << std::endl;


  {
    const auto hz = declare_parameter<double>("frequency_hz");
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&EngageTransitionManager::onTimer, this));
  }
}

void EngageTransitionManager::onOperationModeRequest(
  const OperationModeRequest::Request::SharedPtr request,
  const OperationModeRequest::Response::SharedPtr response)
{
  const auto req_state = toEnum(request->mode);

  if (req_state == State::TRANSITION_TO_AUTO) {
    RCLCPP_WARN(get_logger(), "mode change to TRANSITION_TO_AUTO is not supported. Request ignored.");
    response->success = false;
    return;
  }

  data_.requested_state = toEnum(request->mode);

  // TODO(Horibe) multithreadで回して、遷移できない場合は却下しないといけない
  // 別にmultithreadにしなくても、今遷移できるかここで判断すれば良くない？
  response->success = true;
}

void EngageTransitionManager::onTimer()
{
  data_.is_auto_available = checkEngageAvailable();

  updateState(data_);

  publishData();

  RCLCPP_INFO_STREAM(
    get_logger(), "Timer: engage_available: "
                    << data_.is_auto_available
                    << ", requested_state: " << toStr(data_.requested_state)
                    << ", current state: " << toStr(engage_transition_manager_->getCurrentState()));
}

void EngageTransitionManager::publishData()
{
  const auto time = now();

  OperationMode mode;
  mode.stamp = time;
  mode.mode = toMsg(engage_transition_manager_->getCurrentState());
  pub_operation_mode_->publish(mode);

  IsAutonomousAvailable msg;
  msg.stamp = time;
  msg.is_autonomous_available = data_.is_auto_available;
  pub_auto_available_->publish(msg);
}


bool EngageTransitionManager::checkEngageAvailable()
{

  constexpr auto dist_max = 5.0;
  constexpr auto yaw_max = M_PI_4;

  if (data_.trajectory.points.size() < 2) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: trajectory size must be > 2");
    return false;
  }

  const auto closest_idx =
    findNearestIndex(data_.trajectory.points, data_.kinematics.pose.pose, dist_max, yaw_max);
  if (!closest_idx) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: closest point not found");
    return false;  // closest trajectory point not found.
  }
  const auto closest_point = data_.trajectory.points.at(*closest_idx);

  // check for lateral deviation
  const auto lateral_deviation = calcDistance2d(closest_point.pose, data_.kinematics.pose.pose);
  if (lateral_deviation > engage_acceptable_param_.dist_threshold) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: lateral deviation is too large: %f", lateral_deviation);
    return false;
  }

  // check for yaw deviation
  const auto yaw_deviation = calcYawDeviation(closest_point.pose, data_.kinematics.pose.pose);
  if (yaw_deviation > engage_acceptable_param_.yaw_threshold) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: yaw deviation is too large: %f", yaw_deviation);
    return false;
  }

  // check for speed deviation
  const auto speed_deviation =
    std::abs(closest_point.longitudinal_velocity_mps - data_.kinematics.twist.twist.linear.x);
  if (speed_deviation > engage_acceptable_param_.speed_threshold) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: speed deviation is too large: %f", speed_deviation);
    return false;
  }

  return true;
}

State EngageTransitionManager::updateState(const Data & data)
{
  const auto current_state = engage_transition_manager_->getCurrentState();

  engage_transition_manager_->setData(data);
  const auto next_state = engage_transition_manager_->update();

  // no state change
  if (next_state == current_state) {
    return current_state;
  }

  // transit state
  switch (next_state) {
    case State::NONE:
      engage_transition_manager_ = std::make_unique<NoneState>(this);
      break;
    case State::REMOTE:
      engage_transition_manager_ = std::make_unique<RemoteState>(this);
      break;
    case State::DIRECT:
      engage_transition_manager_ = std::make_unique<DirectState>(this);
      break;
    case State::LOCAL:
      engage_transition_manager_ = std::make_unique<LocalState>(this);
      break;
    case State::TRANSITION_TO_AUTO:
      engage_transition_manager_ = std::make_unique<TransitionState>(this);
      break;
    case State::AUTONOMOUS:
      engage_transition_manager_ = std::make_unique<AutonomousState>(this);
      break;
  }

  if (next_state != engage_transition_manager_->getCurrentState()) {
    throw std::runtime_error("engage_transition_manager: unexpected state change!");
  }

  return engage_transition_manager_->getCurrentState();
}

}  // namespace engage_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(engage_transition_manager::EngageTransitionManager)
