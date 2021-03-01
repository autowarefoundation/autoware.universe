// Copyright 2020 Tier IV, Inc.
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
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_

#include "kalman_filter/kalman_filter.hpp"
#include "autoware_perception_msgs/msg/dynamic_object.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"

class BigVehicleTracker : public Tracker
{
private:
  autoware_perception_msgs::msg::DynamicObject object_;
  rclcpp::Logger logger_;

private:
  KalmanFilter ekf_;
  rclcpp::Time last_update_time_;
  enum IDX
  {
    X = 0,
    Y = 1,
    YAW = 2,
    VX = 3,
    WZ = 4,
  };
  char dim_x_ = 5;
  float process_noise_covariance_pos_x_;
  float process_noise_covariance_pos_y_;
  float process_noise_covariance_yaw_;
  float process_noise_covariance_wz_;
  float process_noise_covariance_vx_;
  float initial_measurement_noise_covariance_vx_;
  float initial_measurement_noise_covariance_wz_;
  // if use_measurement_covariance_ is false, use following params
  bool use_measurement_covariance_;
  float measurement_noise_covariance_pos_x_;
  float measurement_noise_covariance_pos_y_;
  float measurement_noise_covariance_yaw_;
  float initial_measurement_noise_covariance_pos_x_;
  float initial_measurement_noise_covariance_pos_y_;
  float initial_measurement_noise_covariance_yaw_;

  double max_vx_;
  double max_wz_;

private:
  struct BoundingBox
  {
    double width;
    double length;
    double height;
  };
  BoundingBox bounding_box_;

public:
  BigVehicleTracker(
    const rclcpp::Time & time,
    const autoware_perception_msgs::msg::DynamicObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool predict(const double dt, KalmanFilter & ekf);
  bool measure(
    const autoware_perception_msgs::msg::DynamicObject & object,
    const rclcpp::Time & time) override;
  bool measureWithPose(const autoware_perception_msgs::msg::DynamicObject & object);
  bool measureWithShape(const autoware_perception_msgs::msg::DynamicObject & object);
  bool getEstimatedDynamicObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) override;
  virtual ~BigVehicleTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_
