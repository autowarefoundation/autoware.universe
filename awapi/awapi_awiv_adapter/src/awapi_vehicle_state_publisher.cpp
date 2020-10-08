/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <awapi_awiv_adapter/awapi_vehicle_state_publisher.h>

namespace autoware_api
{
AutowareIvVehicleStatePublisher::AutowareIvVehicleStatePublisher()
: nh_(), pnh_("~"), prev_accel_(0.0)
{
  // publisher
  pub_state_ = pnh_.advertise<autoware_api_msgs::AwapiVehicleStatus>("output/vehicle_status", 1);
}

void AutowareIvVehicleStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::AwapiVehicleStatus status = initVehicleStatus();

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getPoseInfo(aw_info.current_pose_ptr, &status);
  getSteerInfo(aw_info.steer_ptr, &status);
  getVehicleCmdInfo(aw_info.vehicle_cmd_ptr, &status);
  getTurnSignalInfo(aw_info.turn_signal_ptr, &status);
  getTwistInfo(aw_info.twist_ptr, &status);
  getGearInfo(aw_info.gear_ptr, &status);
  getBatteryInfo(aw_info.battery_ptr, &status);
  getGpsInfo(aw_info.nav_sat_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

autoware_api_msgs::AwapiVehicleStatus AutowareIvVehicleStatePublisher::initVehicleStatus()
{
  autoware_api_msgs::AwapiVehicleStatus status;
  // set default value
  if (std::numeric_limits<float>::has_quiet_NaN) {
    status.energy_level = std::numeric_limits<float>::quiet_NaN();
  }
  return status;
}

void AutowareIvVehicleStatePublisher::getPoseInfo(
  const std::shared_ptr<geometry_msgs::PoseStamped> & pose_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!pose_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] current pose is nullptr");
    return;
  }

  // get pose
  status->pose = pose_ptr->pose;

  // convert quaternion to euler
  double yaw, pitch, roll;
  tf2::getEulerYPR(pose_ptr->pose.orientation, yaw, pitch, roll);
  status->eulerangle.yaw = yaw;
  status->eulerangle.pitch = pitch;
  status->eulerangle.roll = roll;
}

void AutowareIvVehicleStatePublisher::getSteerInfo(
  const autoware_vehicle_msgs::Steering::ConstPtr & steer_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!steer_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] steer is nullptr");
    return;
  }

  // get steer
  status->steering = steer_ptr->data;

  // get steer vel
  if (previous_steer_ptr_) {
    //calculate steer vel from steer
    const double ds = steer_ptr->data - previous_steer_ptr_->data;
    const double dt =
      std::max((steer_ptr->header.stamp - previous_steer_ptr_->header.stamp).toSec(), 1e-03);
    const double steer_vel = ds / dt;

    //apply lowpass filter
    const double lowpass_steer =
      lowpass_filter(steer_vel, prev_steer_vel_, steer_vel_lowpass_gain_);
    prev_steer_vel_ = lowpass_steer;
    status->steering_velocity = lowpass_steer;
  }
  previous_steer_ptr_ = steer_ptr;
}
void AutowareIvVehicleStatePublisher::getVehicleCmdInfo(
  const autoware_vehicle_msgs::VehicleCommand::ConstPtr & vehicle_cmd_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!vehicle_cmd_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] vehicle cmd is nullptr");
    return;
  }

  // get command
  status->target_acceleration = vehicle_cmd_ptr->control.acceleration;
  status->target_velocity = vehicle_cmd_ptr->control.velocity;
  status->target_steering = vehicle_cmd_ptr->control.steering_angle;
  status->target_steering_velocity = vehicle_cmd_ptr->control.steering_angle_velocity;
}

void AutowareIvVehicleStatePublisher::getTurnSignalInfo(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr & turn_signal_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!turn_signal_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] turn signal is nullptr");
    return;
  }

  //get turn singnal
  status->turn_signal = turn_signal_ptr->data;
}

void AutowareIvVehicleStatePublisher::getTwistInfo(
  const geometry_msgs::TwistStamped::ConstPtr & twist_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!twist_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] twist is nullptr");
    return;
  }

  // get twist
  status->velocity = twist_ptr->twist.linear.x;
  status->angular_velocity = twist_ptr->twist.angular.z;

  // get accel
  if (previous_twist_ptr_) {
    //calculate accleration from velocity
    const double dv = twist_ptr->twist.linear.x - previous_twist_ptr_->twist.linear.x;
    const double dt =
      std::max((twist_ptr->header.stamp - previous_twist_ptr_->header.stamp).toSec(), 1e-03);
    const double accel = dv / dt;

    // apply lowpass filter
    const double lowpass_accel = lowpass_filter(accel, prev_accel_, accel_lowpass_gain_);
    prev_accel_ = lowpass_accel;
    status->acceleration = lowpass_accel;
  }
  previous_twist_ptr_ = twist_ptr;
}

void AutowareIvVehicleStatePublisher::getGearInfo(
  const autoware_vehicle_msgs::ShiftStamped::ConstPtr & gear_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!gear_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] gear is nullptr");
    return;
  }

  // get gear (shift)
  status->gear = gear_ptr->shift.data;
}

void AutowareIvVehicleStatePublisher::getBatteryInfo(
  const std_msgs::Float32::ConstPtr & battery_ptr, autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!battery_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] battery is nullptr");
    return;
  }

  // get battery
  status->energy_level = battery_ptr->data;
}

void AutowareIvVehicleStatePublisher::getGpsInfo(
  const sensor_msgs::NavSatFix::ConstPtr & nav_sat_ptr,
  autoware_api_msgs::AwapiVehicleStatus * status)
{
  if (!nav_sat_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvVehicleStatePublisher] nav_sat(gps) is nullptr");
    return;
  }

  // get position (latlon)
  status->latlon.lat = nav_sat_ptr->latitude;
  status->latlon.lon = nav_sat_ptr->longitude;
  status->latlon.alt = nav_sat_ptr->altitude;
}

}  // namespace autoware_api
