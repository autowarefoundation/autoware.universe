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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__OBJECT_MODEL__OBJECT_MODEL_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__OBJECT_MODEL__OBJECT_MODEL_HPP_

#include <cmath>

namespace
{
constexpr double const_g = 9.81;

template <typename T>
constexpr T sq(const T x)
{
  return x * x;
}

template <typename T>
constexpr T deg2rad(const T deg)
{
  return deg * static_cast<T>(M_PI) / static_cast<T>(180.0);
}

// cspell: ignore kmph
template <typename T>
constexpr T kmph2mps(const T kmph)
{
  return kmph * static_cast<T>(1000.0) / static_cast<T>(3600.0);
}

}  // namespace

namespace object_model
{

enum class ObjectModelType { NormalVehicle, BigVehicle, Bicycle, Pedestrian, Unknown };
struct ObjectSize
{
  double length{0.0};  // [m]
  double width{0.0};   // [m]
  double height{0.0};  // [m]
};
struct ObjectSizeLimit
{
  double length_min{0.0};  // [m]
  double length_max{0.0};  // [m]
  double width_min{0.0};   // [m]
  double width_max{0.0};   // [m]
  double height_min{0.0};  // [m]
  double height_max{0.0};  // [m]
};
struct MotionProcessNoise
{
  double acc_long{0.0};      // [m/s^2] uncertain longitudinal acceleration
  double acc_lat{0.0};       // [m/s^2] uncertain lateral acceleration
  double yaw_rate_min{0.0};  // [rad/s] uncertain yaw change rate, minimum
  double yaw_rate_max{0.0};  // [rad/s] uncertain yaw change rate, maximum
};
struct MotionProcessLimit
{
  double acc_long_max{0.0};  // [m/s^2]
  double acc_lat_max{0.0};   // [m/s^2]
  double vel_long_max{0.0};  // [m/s]
  double vel_lat_max{0.0};   // [m/s]
};
struct StateCovariance
{
  double pos_x{0.0};     // [m^2]
  double pos_y{0.0};     // [m^2]
  double yaw{0.0};       // [rad^2]
  double yaw_rate{0.0};  // [rad^2/s^2]
  double vel_long{0.0};  // [m^2/s^2]
  double vel_lat{0.0};   // [m^2/s^2]
  double acc_long{0.0};  // [m^2/s^4]
  double acc_lat{0.0};   // [m^2/s^4]
};
struct BicycleModelState
{
  double slip_angle_max{0.0};         // [rad] max slip angle
  double slip_rate_cov_min{0.0};      // [rad/s] uncertain slip angle change rate, minimum
  double slip_rate_cov_max{0.0};      // [rad/s] uncertain slip angle change rate, maximum
  double wheel_pos_ratio_front{0.0};  // [-]
  double wheel_pos_ratio_rear{0.0};   // [-]
  double wheel_pos_front_min{0.0};    // [m]
  double wheel_pos_rear_min{0.0};     // [m]
};

class ObjectModel
{
public:
  ObjectSize init_size;
  ObjectSizeLimit size_limit;
  MotionProcessNoise process_noise;
  MotionProcessLimit process_limit;
  StateCovariance initial_covariance;
  StateCovariance measurement_covariance;
  BicycleModelState bicycle_state;

  explicit ObjectModel(const ObjectModelType & type)
  {
    switch (type) {
      case ObjectModelType::NormalVehicle:
        init_size.length = 3.0;
        init_size.width = 2.0;
        init_size.height = 1.8;
        size_limit.length_min = 1.0;
        size_limit.length_max = 20.0;
        size_limit.width_min = 1.0;
        size_limit.width_max = 5.0;
        size_limit.height_min = 1.0;
        size_limit.height_max = 5.0;

        process_noise.acc_long = const_g * 0.35;
        process_noise.acc_lat = const_g * 0.15;
        process_noise.yaw_rate_min = 1.5;   // deg2rad(1.5);
        process_noise.yaw_rate_max = 15.0;  // deg2rad(15.0);

        process_limit.acc_long_max = const_g;
        process_limit.acc_lat_max = const_g;
        process_limit.vel_long_max = kmph2mps(100.0);

        // initial covariance
        initial_covariance.pos_x = sq(1.0);
        initial_covariance.pos_y = sq(0.3);
        initial_covariance.yaw = sq(deg2rad(25.0));
        initial_covariance.vel_long = sq(kmph2mps(1000.0));
        initial_covariance.vel_lat = sq(0.2);

        // measurement noise model
        measurement_covariance.pos_x = sq(0.5);
        measurement_covariance.pos_y = sq(0.4);
        measurement_covariance.yaw = sq(deg2rad(20.0));
        measurement_covariance.vel_long = sq(1.0);

        // bicycle motion model
        bicycle_state.slip_angle_max = 30.0;     // deg2rad(30.0);
        bicycle_state.slip_rate_cov_min = 0.3;   // sq(deg2rad(0.3));
        bicycle_state.slip_rate_cov_max = 10.0;  // sq(deg2rad(10.0));
        bicycle_state.wheel_pos_ratio_front = 0.3;
        bicycle_state.wheel_pos_ratio_rear = 0.25;
        bicycle_state.wheel_pos_front_min = 1.0;
        bicycle_state.wheel_pos_rear_min = 1.0;
        break;

      case ObjectModelType::BigVehicle:
        init_size.length = 10.0;
        init_size.width = 2.5;
        init_size.height = 3.0;
        size_limit.length_min = 1.0;
        size_limit.length_max = 20.0;
        size_limit.width_min = 1.0;
        size_limit.width_max = 5.0;
        size_limit.height_min = 1.0;
        size_limit.height_max = 5.0;

        process_noise.acc_long = const_g * 0.35;
        process_noise.acc_lat = const_g * 0.15;
        process_noise.yaw_rate_min = deg2rad(1.5);
        process_noise.yaw_rate_max = deg2rad(15.0);

        process_limit.acc_long_max = const_g;
        process_limit.acc_lat_max = const_g;
        process_limit.vel_long_max = kmph2mps(100.0);

        // initial covariance
        initial_covariance.pos_x = sq(1.0);
        initial_covariance.pos_y = sq(0.3);
        initial_covariance.yaw = sq(deg2rad(25.0));
        initial_covariance.vel_long = sq(kmph2mps(1000.0));
        initial_covariance.vel_lat = sq(0.2);

        // measurement noise model
        measurement_covariance.pos_x = sq(0.5);
        measurement_covariance.pos_y = sq(0.4);
        measurement_covariance.yaw = sq(deg2rad(20.0));
        measurement_covariance.vel_long = sq(1.0);

        // bicycle motion model
        bicycle_state.slip_angle_max = deg2rad(30.0);
        bicycle_state.slip_rate_cov_min = sq(deg2rad(0.3));
        bicycle_state.slip_rate_cov_max = sq(deg2rad(10.0));
        bicycle_state.wheel_pos_ratio_front = 0.3;
        bicycle_state.wheel_pos_ratio_rear = 0.25;
        bicycle_state.wheel_pos_front_min = 1.0;
        bicycle_state.wheel_pos_rear_min = 1.0;
        break;

      case ObjectModelType::Bicycle:
        init_size.length = 2.0;
        init_size.width = 0.7;
        init_size.height = 1.0;
        size_limit.length_min = 1.0;
        size_limit.length_max = 20.0;
        size_limit.width_min = 1.0;
        size_limit.width_max = 5.0;
        size_limit.height_min = 1.0;
        size_limit.height_max = 5.0;

        process_noise.acc_long = const_g * 0.35;
        process_noise.acc_lat = const_g * 0.15;
        process_noise.yaw_rate_min = deg2rad(1.5);
        process_noise.yaw_rate_max = deg2rad(15.0);

        process_limit.acc_long_max = const_g;
        process_limit.acc_lat_max = const_g;
        process_limit.vel_long_max = kmph2mps(100.0);

        // initial covariance
        initial_covariance.pos_x = sq(1.0);
        initial_covariance.pos_y = sq(0.3);
        initial_covariance.yaw = sq(deg2rad(25.0));
        initial_covariance.vel_long = sq(kmph2mps(1000.0));
        initial_covariance.vel_lat = sq(0.2);

        // measurement noise model
        measurement_covariance.pos_x = sq(0.5);
        measurement_covariance.pos_y = sq(0.4);
        measurement_covariance.yaw = sq(deg2rad(20.0));
        measurement_covariance.vel_long = sq(1.0);

        // bicycle motion model
        bicycle_state.slip_angle_max = deg2rad(30.0);
        bicycle_state.slip_rate_cov_min = sq(deg2rad(0.3));
        bicycle_state.slip_rate_cov_max = sq(deg2rad(10.0));
        bicycle_state.wheel_pos_ratio_front = 0.3;
        bicycle_state.wheel_pos_ratio_rear = 0.25;
        bicycle_state.wheel_pos_front_min = 1.0;
        bicycle_state.wheel_pos_rear_min = 1.0;
        break;

      case ObjectModelType::Pedestrian:
        init_size.length = 0.5;
        init_size.width = 0.5;
        init_size.height = 1.7;
        size_limit.length_min = 0.3;
        size_limit.length_max = 2.0;
        size_limit.width_min = 0.3;
        size_limit.width_max = 1.0;
        size_limit.height_min = 1.0;
        size_limit.height_max = 2.0;

        process_noise.acc_long = const_g * 0.35;
        process_noise.acc_lat = const_g * 0.15;
        process_noise.yaw_rate_min = deg2rad(1.5);
        process_noise.yaw_rate_max = deg2rad(15.0);

        process_limit.acc_long_max = const_g;
        process_limit.acc_lat_max = const_g;
        process_limit.vel_long_max = kmph2mps(100.0);

        // initial covariance
        initial_covariance.pos_x = sq(1.0);
        initial_covariance.pos_y = sq(0.3);
        initial_covariance.yaw = sq(deg2rad(25.0));
        initial_covariance.vel_long = sq(kmph2mps(1000.0));
        initial_covariance.vel_lat = sq(0.2);

        // measurement noise model
        measurement_covariance.pos_x = sq(0.5);
        measurement_covariance.pos_y = sq(0.4);
        measurement_covariance.yaw = sq(deg2rad(20.0));
        measurement_covariance.vel_long = sq(1.0);
        break;

      default:
        break;
    }
  }
  virtual ~ObjectModel() = default;
};

// create static objects by using ObjectModel class
static const ObjectModel normal_vehicle(ObjectModelType::NormalVehicle);
static const ObjectModel big_vehicle(ObjectModelType::BigVehicle);
static const ObjectModel bicycle(ObjectModelType::Bicycle);
static const ObjectModel pedestrian(ObjectModelType::Pedestrian);

}  // namespace object_model

#endif  // MULTI_OBJECT_TRACKER__TRACKER__OBJECT_MODEL__OBJECT_MODEL_HPP_
