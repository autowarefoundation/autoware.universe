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

#include <ros/ros.h>

struct VehicleInfo
{
  VehicleInfo()
  : wheel_radius(0.0),
    wheel_width(0.0),
    wheel_base(0.0),
    wheel_tread(0.0),
    front_overhang(0.0),
    rear_overhang(0.0),
    left_overhang(0.0),
    right_overhang(0.0),
    vehicle_height(0.0)
  {
  }

  double wheel_radius;
  double wheel_width;
  double wheel_base;
  double wheel_tread;
  double front_overhang;
  double rear_overhang;
  double left_overhang;
  double right_overhang;
  double vehicle_height;
};

struct AdditionalVehicleInfo
{
  AdditionalVehicleInfo()
  : vehicle_length(0.0),
    vehicle_width(0.0),
    min_longitudinal_offset(0.0),
    max_longitudinal_offset(0.0),
    min_lateral_offset(0.0),
    max_lateral_offset(0.0),
    min_height_offset(0.0),
    max_height_offset(0.0)
  {
  }

  double vehicle_length;
  double vehicle_width;
  double min_longitudinal_offset;
  double max_longitudinal_offset;
  double min_lateral_offset;
  double max_lateral_offset;
  double min_height_offset;
  double max_height_offset;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "additional_vehicle_info_generator");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  VehicleInfo vehicle_info;
  nh.getParam("wheel_radius", vehicle_info.wheel_radius);
  nh.getParam("wheel_width", vehicle_info.wheel_width);
  nh.getParam("wheel_base", vehicle_info.wheel_base);
  nh.getParam("wheel_tread", vehicle_info.wheel_tread);
  nh.getParam("front_overhang", vehicle_info.front_overhang);
  nh.getParam("rear_overhang", vehicle_info.rear_overhang);
  nh.getParam("left_overhang", vehicle_info.left_overhang);
  nh.getParam("right_overhang", vehicle_info.right_overhang);
  nh.getParam("vehicle_height", vehicle_info.vehicle_height);

  ROS_INFO("wheel_radius: %lf", vehicle_info.wheel_radius);
  ROS_INFO("wheel_width: %lf", vehicle_info.wheel_width);
  ROS_INFO("wheel_base: %lf", vehicle_info.wheel_base);
  ROS_INFO("wheel_tread: %lf", vehicle_info.wheel_tread);
  ROS_INFO("front_overhang: %lf", vehicle_info.front_overhang);
  ROS_INFO("rear_overhang: %lf", vehicle_info.rear_overhang);
  ROS_INFO("left_overhang: %lf", vehicle_info.left_overhang);
  ROS_INFO("right_overhang: %lf", vehicle_info.right_overhang);
  ROS_INFO("vehicle_height: %lf", vehicle_info.vehicle_height);

  AdditionalVehicleInfo add_vehicle_info;
  add_vehicle_info.vehicle_length =
    vehicle_info.front_overhang + vehicle_info.wheel_base + vehicle_info.rear_overhang;
  add_vehicle_info.vehicle_width = vehicle_info.wheel_tread + vehicle_info.wheel_width +
                                   vehicle_info.left_overhang + vehicle_info.right_overhang;
  add_vehicle_info.min_longitudinal_offset = -vehicle_info.rear_overhang;
  add_vehicle_info.max_longitudinal_offset = vehicle_info.front_overhang + vehicle_info.wheel_base;
  add_vehicle_info.min_lateral_offset =
    -((vehicle_info.wheel_tread + vehicle_info.wheel_width) / 2.0 + vehicle_info.right_overhang);
  add_vehicle_info.max_lateral_offset =
    (vehicle_info.wheel_tread + vehicle_info.wheel_width) / 2.0 + vehicle_info.left_overhang;
  add_vehicle_info.min_height_offset = 0.0;
  add_vehicle_info.max_height_offset = vehicle_info.vehicle_height;

  nh.setParam("vehicle_length", add_vehicle_info.vehicle_length);
  nh.setParam("vehicle_width", add_vehicle_info.vehicle_width);
  nh.setParam("min_longitudinal_offset", add_vehicle_info.min_longitudinal_offset);
  nh.setParam("max_longitudinal_offset", add_vehicle_info.max_longitudinal_offset);
  nh.setParam("min_lateral_offset", add_vehicle_info.min_lateral_offset);
  nh.setParam("max_lateral_offset", add_vehicle_info.max_lateral_offset);
  nh.setParam("min_height_offset", add_vehicle_info.min_height_offset);
  nh.setParam("max_height_offset", add_vehicle_info.max_height_offset);

  ROS_INFO("vehicle_length: %lf", add_vehicle_info.vehicle_length);
  ROS_INFO("vehicle_width: %lf", add_vehicle_info.vehicle_width);
  ROS_INFO("min_longitudinal_offset: %lf", add_vehicle_info.min_longitudinal_offset);
  ROS_INFO("max_longitudinal_offset: %lf", add_vehicle_info.max_longitudinal_offset);
  ROS_INFO("min_lateral_offset: %lf", add_vehicle_info.min_lateral_offset);
  ROS_INFO("max_lateral_offset: %lf", add_vehicle_info.max_lateral_offset);
  ROS_INFO("min_height_offset: %lf", add_vehicle_info.min_height_offset);
  ROS_INFO("max_height_offset: %lf", add_vehicle_info.max_height_offset);

  return 0;
}