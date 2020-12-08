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

#ifndef MULTI_OBJECT_TRACKER_TRACKER_BASE_HPP_
#define MULTI_OBJECT_TRACKER_TRACKER_BASE_HPP_

#include "autoware_perception_msgs/msg/dynamic_object.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "rclcpp/rclcpp.hpp"

class Tracker
{
protected:
  unique_identifier_msgs::msg::UUID getUUID() {return uuid_;}
  void setType(int type) {type_ = type;}

private:
  unique_identifier_msgs::msg::UUID uuid_;
  int type_;
  int no_measurement_count_;
  int total_measurement_count_;
  rclcpp::Time last_update_with_measurement_time_;

public:
  Tracker(const rclcpp::Time & time, const int type);
  virtual ~Tracker() {}
  bool updateWithMeasurement(
    const autoware_perception_msgs::msg::DynamicObject & object,
    const rclcpp::Time & measurement_time);
  bool updateWithoutMeasurement();
  int getType() {return type_;}
  int getNoMeasurementCount() {return no_measurement_count_;}
  int getTotalMeasurementCount() {return total_measurement_count_;}
  double getElapsedTimeFromLastUpdate(const rclcpp::Time & current_time)
  {
    return (current_time - last_update_with_measurement_time_).seconds();
  }
  virtual geometry_msgs::msg::Point getPosition(const rclcpp::Time & time);
  virtual double getArea(const rclcpp::Time & time);

  /*
   *　Pure virtual function
   */

protected:
  virtual bool measure(
    const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time) = 0;

public:
  virtual bool getEstimatedDynamicObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object) = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;
};

#endif
