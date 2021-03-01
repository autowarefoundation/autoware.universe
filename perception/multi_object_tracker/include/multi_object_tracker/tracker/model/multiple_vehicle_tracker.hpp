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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__MULTIPLE_VEHICLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__MULTIPLE_VEHICLE_TRACKER_HPP_

#include "kalman_filter/kalman_filter.hpp"
#include "multi_object_tracker/tracker/model/big_vehicle_tracker.hpp"
#include "multi_object_tracker/tracker/model/normal_vehicle_tracker.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware_perception_msgs/msg/dynamic_object.hpp"

#include "rclcpp/time.hpp"

class MultipleVehicleTracker : public Tracker
{
private:
  NormalVehicleTracker normal_vehicle_tracker_;
  BigVehicleTracker big_vehicle_tracker_;

public:
  MultipleVehicleTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_perception_msgs::msg::DynamicObject & object,
    const rclcpp::Time & time) override;
  bool getEstimatedDynamicObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) override;
  virtual ~MultipleVehicleTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__MULTIPLE_VEHICLE_TRACKER_HPP_
