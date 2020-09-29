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

#include <shift_decider/shift_decider.h>

ShiftDecider::ShiftDecider()
{
  pub_shift_cmd_ = pnh_.advertise<autoware_vehicle_msgs::ShiftStamped>("output/shift_cmd", 1, true);
  sub_control_cmd_ = pnh_.subscribe("input/control_cmd", 1, &ShiftDecider::onControlCmd, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &ShiftDecider::onTimer, this);
}

void ShiftDecider::onControlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr msg)
{
  control_cmd_ = msg;
}

void ShiftDecider::onTimer(const ros::TimerEvent & e)
{
  if (!control_cmd_) return;

  updateCurrentShiftCmd();
  pub_shift_cmd_.publish(shift_cmd_);
}

void ShiftDecider::updateCurrentShiftCmd()
{
  shift_cmd_.header.stamp = ros::Time::now();
  constexpr double vel_threshold = 0.01;  // to prevent chattering
  if (control_cmd_->control.velocity > vel_threshold) {
    shift_cmd_.shift.data = autoware_vehicle_msgs::Shift::DRIVE;
  } else if (control_cmd_->control.velocity < -vel_threshold) {
    shift_cmd_.shift.data = autoware_vehicle_msgs::Shift::REVERSE;
  }
}
