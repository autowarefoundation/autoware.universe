// Copyright 2021 - 2025 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef AUTOWARE__CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_NODE_HPP_
#define AUTOWARE__CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_NODE_HPP_

#include "autoware/control_performance_analysis/control_performance_analysis_core.hpp"
#include "autoware_control_performance_analysis/msg/driving_monitor_stamped.hpp"
#include "autoware_control_performance_analysis/msg/error_stamped.hpp"

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <utility>

namespace autoware::control_performance_analysis
{
using autoware_control_msgs::msg::Control;
using autoware_control_performance_analysis::msg::DrivingMonitorStamped;
using autoware_control_performance_analysis::msg::ErrorStamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

class ControlPerformanceAnalysisNode : public rclcpp::Node
{
public:
  explicit ControlPerformanceAnalysisNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscribers and Local Variable Assignment
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;  // subscribe to trajectory
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;  // subscribe to steering control value
  rclcpp::Subscription<Odometry>::SharedPtr sub_velocity_;    // subscribe to velocity
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_vehicle_steering_;

  // Publishers
  rclcpp::Publisher<ErrorStamped>::SharedPtr pub_error_msg_;  // publish error message
  rclcpp::Publisher<DrivingMonitorStamped>::SharedPtr
    pub_driving_msg_;  // publish driving status message

  // Node Methods
  bool isDataReady() const;  // check if data arrive
  static bool isValidTrajectory(const Trajectory & traj);

  // Callback Methods
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onControlRaw(const Control::ConstSharedPtr control_msg);
  void onVecSteeringMeasured(const SteeringReport::ConstSharedPtr meas_steer_msg);
  void onVelocity(const Odometry::ConstSharedPtr msg);

  // Parameters
  Params param_{};  // wheelbase, control period and feedback coefficients.
  // State holder
  Control::ConstSharedPtr last_control_cmd_;
  double d_control_cmd_{0};

  // Subscriber Parameters
  Trajectory::ConstSharedPtr current_trajectory_ptr_;  // ConstPtr to local traj.
  Control::ConstSharedPtr current_control_msg_ptr_;
  SteeringReport::ConstSharedPtr current_vec_steering_msg_ptr_;
  Odometry::ConstSharedPtr current_odom_ptr_;
  PoseStamped::ConstSharedPtr current_pose_;  // pose of the vehicle, x, y, heading

  // prev states
  Trajectory prev_traj;
  Control prev_cmd;
  SteeringReport prev_steering;

  // Algorithm
  std::unique_ptr<ControlPerformanceAnalysisCore> control_performance_core_ptr_;
};
}  // namespace autoware::control_performance_analysis

#endif  // AUTOWARE__CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_NODE_HPP_
