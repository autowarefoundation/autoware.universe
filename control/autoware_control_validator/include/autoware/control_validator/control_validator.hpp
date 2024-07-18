// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_

#include "autoware/control_validator/debug_marker.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware_vehicle_info_utils/vehicle_info.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include <autoware_control_validator/msg/control_validator_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace autoware::control_validator
{
using autoware_control_validator::msg::ControlValidatorStatus;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using nav_msgs::msg::Odometry;

struct ValidationParams
{
  double max_distance_deviation_threshold;
};

/**
 * @class ControlValidator
 */
class ControlValidator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit ControlValidator(const rclcpp::NodeOptions & options);

  /**
   * @brief Callback function for predicted trajectory
   */
  void on_predicted_trajectory(const Trajectory::ConstSharedPtr msg);

  /**
   * @brief Calculate the maximum lateral distance between the reference trajectory and predicted
   trajectory
    * @param reference_trajectory reference trajectory
    * @param predicted_trajectory predicted trajectory
   */
  std::pair<double, bool> calc_deviation_and_condition(
    const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory) const;

private:
  void setup_diag();

  void setup_parameters();

  bool is_data_ready();

  void validate(const Trajectory & predicted_trajectory);

  void publish_debug_info();

  void display_status();

  void set_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const;

  // Subscribers and publishers
  rclcpp::Subscription<Trajectory>::SharedPtr sub_predicted_traj_;
  universe_utils::InterProcessPollingSubscriber<Odometry>::SharedPtr sub_kinematics_;
  universe_utils::InterProcessPollingSubscriber<Trajectory>::SharedPtr sub_reference_traj_;
  rclcpp::Publisher<ControlValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // system parameters
  int64_t diag_error_count_threshold_ = 0;
  bool display_on_terminal_ = true;

  Updater diag_updater_{this};

  ControlValidatorStatus validation_status_;
  ValidationParams validation_params_;  // for thresholds

  vehicle_info_utils::VehicleInfo vehicle_info_;

  static bool is_all_valid(const ControlValidatorStatus & status);

  Trajectory::ConstSharedPtr current_reference_trajectory_;
  Trajectory::ConstSharedPtr current_predicted_trajectory_;

  Odometry::ConstSharedPtr current_kinematics_;

  std::shared_ptr<ControlValidatorDebugMarkerPublisher> debug_pose_publisher_;
};
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
