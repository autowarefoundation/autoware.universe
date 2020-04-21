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
// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_can_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/6/1
 */

#include "can_msgs/Frame.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

ros::Publisher pub;

uint16_t counter;
int16_t angular_velocity_x_raw = 0;
int16_t angular_velocity_y_raw = 0;
int16_t angular_velocity_z_raw = 0;
int16_t acceleration_x_raw = 0;
int16_t acceleration_y_raw = 0;
int16_t acceleration_z_raw = 0;

sensor_msgs::Imu imu_msg;

void receive_CAN(const can_msgs::Frame::ConstPtr & msg)
{
  if (msg->id == 0x319) {
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = ros::Time::now();

    counter = msg->data[1] + (msg->data[0] << 8);
    angular_velocity_x_raw = msg->data[3] + (msg->data[2] << 8);
    imu_msg.angular_velocity.x =
      angular_velocity_x_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    angular_velocity_y_raw = msg->data[5] + (msg->data[4] << 8);
    imu_msg.angular_velocity.y =
      angular_velocity_y_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    angular_velocity_z_raw = msg->data[7] + (msg->data[6] << 8);
    imu_msg.angular_velocity.z =
      angular_velocity_z_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    ROS_INFO("IMU Counter = %d", counter);
  }
  if (msg->id == 0x31A) {
    acceleration_x_raw = msg->data[3] + (msg->data[2] << 8);
    imu_msg.linear_acceleration.x = acceleration_x_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    acceleration_y_raw = msg->data[5] + (msg->data[4] << 8);
    imu_msg.linear_acceleration.y = acceleration_y_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    acceleration_z_raw = msg->data[7] + (msg->data[6] << 8);
    imu_msg.linear_acceleration.z = acceleration_z_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    pub.publish(imu_msg);
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tag_serial_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/can/imu", 100, receive_CAN);
  pub = n.advertise<sensor_msgs::Imu>("/imu/data_raw", 100);
  ros::spin();

  return 0;
}
