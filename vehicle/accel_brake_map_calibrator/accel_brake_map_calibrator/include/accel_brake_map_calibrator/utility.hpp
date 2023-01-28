// Copyright 2023 Tier IV, Inc.
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

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR__UTILITY_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR__UTILITY_HPP_

#include "accel_brake_map_calibrator/data.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <limits>
#include <queue>
#include <string>
#include <vector>
namespace accel_brake_map_calibrator
{
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;

bool isTimeout(
  const builtin_interfaces::msg::Time & stamp, const double timeout_sec, const rclcpp::Time & now);

bool isTimeout(
  const DataStampedPtr & data_stamped, const double timeout_sec, const rclcpp::Time & now);

double lowpass(const double original, const double current, const double gain = 0.8);

void pushDataToQue(
  const TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
  std::queue<TwistStamped::ConstSharedPtr> * que);

double getAverage(const std::vector<double> & vec);

double getStandardDeviation(const std::vector<double> & vec);

OccupancyGrid getOccMsg(
  const std::string frame_id, const double height, const double width, const double resolution,
  const std::vector<int8_t> & map_value, const rclcpp::Time & stamp);

template <class T>
void pushDataToVec(const T data, const std::size_t max_size, std::vector<T> * vec)
{
  vec->emplace_back(data);
  while (vec->size() > max_size) {
    vec->erase(vec->begin());
  }
}

template <class T>
T getNearestTimeDataFromVec(const T base_data, const double back_time, const std::vector<T> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = rclcpp::Time(base_data->header.stamp).seconds() - back_time;
  T nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = rclcpp::Time(data->header.stamp).seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

DataStampedPtr getNearestTimeDataFromVec(
  DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec);

}  // namespace accel_brake_map_calibrator

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR__UTILITY_HPP_
