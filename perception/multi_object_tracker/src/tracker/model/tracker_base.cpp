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
 *
 * v1.0 Yukihiro Saito
 */

#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "multi_object_tracker/utils/utils.hpp"

#include <algorithm>
#include <random>

Tracker::Tracker(const rclcpp::Time & time, const int type)
: type_(type),
  no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time)
{
  // Generate random number
  std::mt19937 gen(std::random_device{} ());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.uuid.begin(), uuid_.uuid.end(), bit_eng);
}

bool Tracker::updateWithMeasurement(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const rclcpp::Time & measurement_time)
{
  no_measurement_count_ = 0;
  ++total_measurement_count_;
  last_update_with_measurement_time_ = measurement_time;
  measure(object, measurement_time);
  return true;
}

bool Tracker::updateWithoutMeasurement()
{
  ++no_measurement_count_;
  return true;
}

geometry_msgs::msg::Point Tracker::getPosition(const rclcpp::Time & time)
{
  autoware_perception_msgs::msg::DynamicObject object;
  getEstimatedDynamicObject(time, object);
  geometry_msgs::msg::Point position;
  position.x = object.state.pose_covariance.pose.position.x;
  position.y = object.state.pose_covariance.pose.position.y;
  position.z = object.state.pose_covariance.pose.position.z;
  return position;
}

double Tracker::getArea(const rclcpp::Time & time)
{
  autoware_perception_msgs::msg::DynamicObject object;
  getEstimatedDynamicObject(time, object);
  return utils::getArea(object.shape);
}
