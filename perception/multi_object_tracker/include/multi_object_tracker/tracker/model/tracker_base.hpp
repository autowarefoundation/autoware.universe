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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

class Tracker
{
protected:
  unique_identifier_msgs::msg::UUID getUUID() const { return uuid_; }
  void setType(int type) { type_ = type; }

private:
  unique_identifier_msgs::msg::UUID uuid_;
  int type_;
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  rclcpp::Time last_update_with_measurement_time_;

public:
  Tracker(const rclcpp::Time & time, const int type);
  virtual ~Tracker() {}
  bool updateWithMeasurement(
    const autoware_perception_msgs::msg::DynamicObject & object,
    const rclcpp::Time & measurement_time);
  bool updateWithoutMeasurement();
  int getType() const { return type_; }
  int getNoMeasurementCount() const { return no_measurement_count_; }
  int getTotalNoMeasurementCount() const { return total_no_measurement_count_; }
  int getTotalMeasurementCount() const { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate(const rclcpp::Time & current_time) const
  {
    return (current_time - last_update_with_measurement_time_).seconds();
  }
  virtual geometry_msgs::msg::PoseWithCovariance getPoseWithCovariance(
    const rclcpp::Time & time) const;

  /*
   *　Pure virtual function
   */

protected:
  virtual bool measure(
    const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time) = 0;

public:
  virtual bool getEstimatedDynamicObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) const = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
