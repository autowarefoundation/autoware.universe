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

#include <autoware_api_msgs/AwapiVehicleStatus.h>
#include <awapi_awiv_adapter/awapi_autoware_util.h>

namespace autoware_api
{
class AutowareIvVehicleStatePublisher
{
public:
  AutowareIvVehicleStatePublisher();
  void statePublisher(const AutowareInfo & aw_info);

private:
  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher pub_state_;

  void getPoseInfo(
    const std::shared_ptr<geometry_msgs::PoseStamped> & pose_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getSteerInfo(
    const autoware_vehicle_msgs::Steering::ConstPtr & steer_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getVehicleCmdInfo(
    const autoware_vehicle_msgs::VehicleCommand::ConstPtr & vehicle_cmd_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getTurnSignalInfo(
    const autoware_vehicle_msgs::TurnSignal::ConstPtr & turn_signal_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getTwistInfo(
    const geometry_msgs::TwistStamped::ConstPtr & twist_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getGearInfo(
    const autoware_vehicle_msgs::ShiftStamped::ConstPtr & gear_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);
  void getGpsInfo(
    const sensor_msgs::NavSatFix::ConstPtr & nav_sat_ptr,
    autoware_api_msgs::AwapiVehicleStatus * status);

  //parameters
  geometry_msgs::TwistStamped::ConstPtr previous_twist_ptr_;
  autoware_vehicle_msgs::Steering::ConstPtr previous_steer_ptr_;
  double prev_accel_;
  double prev_steer_vel_;

  // defined value
  const double accel_lowpass_gain_ = 0.2;
  const double steer_vel_lowpass_gain_ = 0.2;
};

}  // namespace autoware_api
