/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H
#define VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H

#include <memory>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "autoware_control_msgs/ControlCommandStamped.h"
#include "autoware_vehicle_msgs/VehicleCommand.h"
#include "vehicle_cmd_gate/vehicle_cmd_filter.h"

class VehicleCmdGate
{
public:
  VehicleCmdGate();
  ~VehicleCmdGate() = default;

private:
  void ctrlCmdCallback(const autoware_control_msgs::ControlCommandStamped & msg);
  void engageCallback(const std_msgs::Bool msg);

  double getDt();
  void processCommand();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Publisher shift_cmd_pub_;
  ros::Subscriber control_cmd_sub_;
  ros::Subscriber engage_sub_;

  std::shared_ptr<ros::Time> prev_time_;

  autoware_vehicle_msgs::VehicleCommand prev_vehicle_cmd_;
  bool is_engaged_;

  VehicleCmdFilter filter;
};

#endif  // VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H
