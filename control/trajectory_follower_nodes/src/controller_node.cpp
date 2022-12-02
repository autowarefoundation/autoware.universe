// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "trajectory_follower_nodes/controller_node.hpp"

// TODO(murooka) revert pure_pursuit later
// #include "pure_pursuit/pure_pursuit_lateral_controller.hpp"
#include "trajectory_follower/mpc_lateral_controller.hpp"
#include "trajectory_follower/pid_longitudinal_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
Controller::Controller(const rclcpp::NodeOptions & node_options) : Node("controller", node_options)
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period", 0.03);
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec", 0.5);

  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter("lateral_controller_mode", "mpc_follower"));
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller_ = std::make_shared<trajectory_follower::MpcLateralController>(*this);
      break;
    }
    // case LateralControllerMode::PURE_PURSUIT: {
    //   lateral_controller_ = std::make_shared<pure_pursuit::PurePursuitLateralController>(*this);
    //   break;
    // }
    default:
      throw std::domain_error("[LateralController] invalid algorithm");
  }

  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter("longitudinal_controller_mode", "pid"));
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller_ =
        std::make_shared<trajectory_follower::PidLongitudinalController>(*this);
      break;
    }
    default:
      throw std::domain_error("[LongitudinalController] invalid algorithm");
  }

  sub_ref_path_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", rclcpp::QoS{1}, std::bind(&Controller::onTrajectory, this, _1));
  sub_steering_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "~/input/current_steering", rclcpp::QoS{1}, std::bind(&Controller::onSteering, this, _1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/current_odometry", rclcpp::QoS{1}, std::bind(&Controller::onOdometry, this, _1));
  sub_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/current_accel", rclcpp::QoS{1}, std::bind(&Controller::onAccel, this, _1));
  control_cmd_pub_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  debug_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }
}

Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc_follower") return LateralControllerMode::MPC;
  // if (controller_mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;

  return LateralControllerMode::INVALID;
}

Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;

  return LongitudinalControllerMode::INVALID;
}

void Controller::onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  current_trajectory_ptr_ = msg;
}

void Controller::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odometry_ptr_ = msg;
}

void Controller::onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  current_steering_ptr_ = msg;
}

void Controller::onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  current_accel_ptr_ = msg;
}

bool Controller::isTimeOut()
{
  const auto now = this->now();
  if ((now - lateral_output_->control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - longitudinal_output_->control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
}

boost::optional<trajectory_follower::InputData> Controller::createInputData() const
{
  if (
    !current_trajectory_ptr_ || !current_odometry_ptr_ || !current_steering_ptr_ ||
    !current_accel_ptr_) {
    // RCL
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;

  return input_data;
}

void Controller::initialize(const trajectory_follower::InputData & input_data)
{
  const bool is_lat_initialized = lateral_controller_->initialize(input_data);
  if (!is_lat_initialized) {
    // RCL
    return;
  }

  const bool is_lon_initialized = longitudinal_controller_->initialize(input_data);
  if (!is_lon_initialized) {
    // RCL
    return;
  }

  is_initialized_ = true;
}

void Controller::callbackTimerControl()
{
  // Since the longitudinal uses the convergence information of the steer
  // with the current trajectory, it is necessary to run the lateral first.
  // TODO(kosuke55): Do not depend on the order of execution.

  // 1. create input data
  const auto input_data = createInputData();
  if (!input_data) {
    return;
  }

  // 2. check if controllers are ready
  if (!lateral_controller_->isReady() || !longitudinal_controller_->isReady()) {
    return;
  }

  // 3. check if controllers are initialized
  if (!is_initialized_) {
    initialize(*input_data);
    return;
  }

  // 4. run controllers
  const auto lat_out = lateral_controller_->run(*input_data);
  const auto lon_out = longitudinal_controller_->run(*input_data);

  // 5. sync with each other controllers
  longitudinal_controller_->sync(lat_out.sync_data);
  lateral_controller_->sync(lon_out.sync_data);

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if (isTimeOut()) return;

  // 6. publish control command
  autoware_auto_control_msgs::msg::AckermannControlCommand out;
  out.stamp = this->now();
  out.lateral = lat_out.control_cmd;
  out.longitudinal = lon_out.control_cmd;
  control_cmd_pub_->publish(out);

  publishDebugMarker();
}

void Controller::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray debug_marker_array{};

  // steer converged marker
  {
    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", this->now(), "steer_converged", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      tier4_autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      tier4_autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.99));
    marker.pose = input_data_.current_odometry_ptr->pose.pose;

    std::stringstream ss;
    const double current = input_data_.current_steering_ptr->steering_tire_angle;
    const double cmd = lateral_output_->control_cmd.steering_tire_angle;
    const double diff = current - cmd;
    ss << "current:" << current << " cmd:" << cmd << " diff:" << diff
       << (lateral_output_->sync_data.is_steer_converged ? " (converged)" : " (not converged)");
    marker.text = ss.str();

    debug_marker_array.markers.push_back(marker);
  }

  debug_marker_pub_->publish(debug_marker_array);
}

}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_nodes::Controller)
