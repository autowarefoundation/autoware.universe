// Copyright 2018-2019 Autoware Foundation
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

#include "raw_vehicle_cmd_converter/node.hpp"

#include <rclcpp/logging.hpp>

#include <functional>

using std::placeholders::_1;

AccelMapConverter::AccelMapConverter() : Node("raw_vehicle_cmd_converter_node"), accel_map_(get_logger()), brake_map_(get_logger())
{
  pub_cmd_ = this->create_publisher<autoware_vehicle_msgs::msg::RawVehicleCommand>("/vehicle/raw_vehicle_cmd", rclcpp::QoS{1});
  sub_cmd_ = this->create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>("/control/vehicle_cmd", 1, std::bind(&AccelMapConverter::callbackVehicleCmd, this, _1));
  sub_velocity_ =
    this->create_subscription<geometry_msgs::msg::TwistStamped>("/localization/twist", 1, std::bind(&AccelMapConverter::callbackVelocity, this, _1));

  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);

  /* parameters for accel/brake map */
  std::string csv_path_accel_map, csv_path_brake_map;
  csv_path_accel_map = declare_parameter("csv_path_accel_map", std::string("empty"));
  csv_path_brake_map = declare_parameter("csv_path_brake_map", std::string("empty"));
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }
}

void AccelMapConverter::callbackVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_velocity_ptr_ = std::make_shared<double>(msg->twist.linear.x);
}

void AccelMapConverter::callbackVehicleCmd(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr vehicle_cmd_ptr)
{
  if (!current_velocity_ptr_ || !acc_map_initialized_) {
    return;
  }

  double desired_throttle = 0.0;
  double desired_brake = 0.0;
  calculateAccelMap(
    *current_velocity_ptr_, vehicle_cmd_ptr->control.acceleration, &desired_throttle,
    &desired_brake);

  autoware_vehicle_msgs::msg::RawVehicleCommand output;
  output.header = vehicle_cmd_ptr->header;
  output.shift = vehicle_cmd_ptr->shift;
  output.emergency = vehicle_cmd_ptr->emergency;
  output.control.steering_angle = vehicle_cmd_ptr->control.steering_angle;
  output.control.steering_angle_velocity = vehicle_cmd_ptr->control.steering_angle_velocity;
  output.control.throttle = desired_throttle;
  output.control.brake = desired_brake;

  pub_cmd_->publish(output);
}

void AccelMapConverter::calculateAccelMap(
  const double current_velocity, const double desired_acc, double * desired_throttle,
  double * desired_brake)
{
  // throttle mode
  if (!accel_map_.getThrottle(desired_acc, std::abs(current_velocity), *desired_throttle)) {
    // brake mode
    *desired_throttle = 0.0;
    brake_map_.getBrake(desired_acc, std::abs(current_velocity), *desired_brake);
  }
  *desired_throttle = std::min(std::max(*desired_throttle, 0.0), max_throttle_);
  *desired_brake = std::min(std::max(*desired_brake, 0.0), max_brake_);
}
