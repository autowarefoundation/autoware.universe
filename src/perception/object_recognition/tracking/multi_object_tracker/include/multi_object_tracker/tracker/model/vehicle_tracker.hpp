/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#pragma once
#include "autoware_perception_msgs/DynamicObject.h"
#include "tracker_base.hpp"

class VehicleTracker : public Tracker
{
private:
  autoware_perception_msgs::DynamicObject object_;
  double filtered_yaw_;
  double yaw_filter_gain_;
  bool is_fixed_yaw_;
  double filtered_dim_x_;
  double filtered_dim_y_;
  double dim_filter_gain_;
  bool is_fixed_dim_;
  double filtered_posx_;
  double filtered_posy_;
  double pos_filter_gain_;
  double filtered_vx_;
  double filtered_vy_;
  double v_filter_gain_;
  double filtered_area_;
  double area_filter_gain_;
  double last_measurement_posx_;
  double last_measurement_posy_;
  ros::Time last_update_time_;
  ros::Time last_measurement_time_;

public:
  VehicleTracker(const ros::Time & time, const autoware_perception_msgs::DynamicObject & object);

  bool predict(const ros::Time & time) override;
  bool measure(
    const autoware_perception_msgs::DynamicObject & object, const ros::Time & time) override;
  bool getEstimatedDynamicObject(
    const ros::Time & time, autoware_perception_msgs::DynamicObject & object) override;
  virtual ~VehicleTracker(){};
};
