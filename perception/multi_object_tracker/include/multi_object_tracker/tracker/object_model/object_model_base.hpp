// Copyright 2024 TIER IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_

#include <cmath>

namespace
{
  constexpr double const_g = 9.81; 
  double deg2rad(const double deg)
  {
    return deg * M_PI / 180.0;
  }
}  // namespace

namespace object_model
{

enum class ObjectModelType
{
    NORMAL_VEHICLE,
    BIG_VEHICLE,
    BICYCLE,
    PEDESTRIAN,
    STATIC,
    UNKNOWN
};
struct ObjectSize
{
  double length; // [m]
  double width; // [m]
  double height; // [m]
};
struct ObjectSizeLimit
{
  double length_min; // [m]
  double length_max; // [m]
  double width_min; // [m]
  double width_max; // [m]
  double height_min; // [m]
  double height_max; // [m]
};
struct MotionProcessNoise
{
  double acc_long; // [m/s^2]
  double acc_lat; // [m/s^2]
  double yaw_rate_min; // [rad/s]
  double yaw_rate_max; // [rad/s]
  double slip_rate_min; 
  double slip_rate_max;

};
struct MotionProcessLimit
{
  double slip_angle_max;
  double acc_long_max;
  double acc_lat_max;
  double vel_long_max;
};
struct StateCovariance
{
  double pos_x; // [m^2]
  double pos_y; // [m^2]
  double yaw; // [rad^2]
  double yaw_rate; // [rad^2/s^2]
  double acc_long; // [m^2/s^4]
  double acc_lat; // [m^2/s^4]
  double vel_long; // [m^2/s^2]
  double vel_lat; // [m^2/s^2]
};
struct BicycleModelState
{
  double slip_angle; // [rad^2]
  double slip_rate; // [rad^2/s^2]
  double wheel_pos_ratio_front; // [-]
  double wheel_pos_ratio_rear; // [-]
  double wheel_pos_front_min; // [m]
  double wheel_pos_rear_min; // [m]
};

class ObjectModel
{
public:
  ObjectSize size;
  ObjectSizeLimit size_limit;
  MotionProcessNoise process_noise;
  MotionProcessLimit process_limit;
  StateCovariance initial_covariance;
  StateCovariance state_covariance;
  BicycleModelState bicycle_model_state;

  ObjectModel(const ObjectModelType & type) 
  {
    switch (type)
    {
    case ObjectModelType::NORMAL_VEHICLE:
      size.length = 3.0;
      size.width = 2.0;
      size.height = 1.8;
      size_limit.length_min = 2.0;
      size_limit.length_max = 5.0;
      size_limit.width_min = 1.5;
      size_limit.width_max = 3.0;
      size_limit.height_min = 1.5;
      size_limit.height_max = 2.0;
      
      process_noise.acc_long = const_g * 0.35;
      process_noise.acc_lat = const_g * 0.15;
      process_noise.yaw_rate_min = 1.5;
      process_noise.yaw_rate_max = 15.0;
      process_noise.slip_rate_min = 0.3;
      process_noise.slip_rate_max = 10.0;

      process_limit.slip_angle_max = 30;
      process_limit.acc_long_max = 2.0;
      process_limit.acc_lat_max = 2.0;
      process_limit.vel_long_max = 30.0;

      // initial covariance
      initial_covariance.pos_x = 0.5;
      initial_covariance.pos_y = 0.4;
      initial_covariance.yaw = deg2rad(20.0);
      initial_covariance.yaw_rate = deg2rad(10.0);
      initial_covariance.acc_long = 1.0;
      initial_covariance.acc_lat = 1.0;
      initial_covariance.vel_long = 1.0;
      initial_covariance.vel_lat = 1.0;

      // noise model
      state_covariance.pos_x = 0.5;
      state_covariance.pos_y = 0.4;
      state_covariance.yaw = 0.1;
      state_covariance.yaw_rate = 0.1;
      state_covariance.acc_long = 0.1;
      state_covariance.acc_lat = 0.1;
      state_covariance.vel_long = 0.1;
      state_covariance.vel_lat = 0.1;
      break;
    }
  }
  virtual ~ObjectModel() = default;
};



ObjectModel normal_vehicle(ObjectModelType::NORMAL_VEHICLE);



}  // namespace object_model

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_